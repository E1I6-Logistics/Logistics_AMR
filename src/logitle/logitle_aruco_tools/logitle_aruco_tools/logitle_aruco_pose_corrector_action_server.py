#!/usr/bin/env python3
"""ROS 2 Action server for ArUco-based map->odom pose correction."""

import argparse
import math
import os
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from logitle_aruco_msgs.action import CorrectPoseWithAruco

from logitle_aruco_tools.logitle_aruco_pose_viewer import (
    ARUCO_DICTS,
    make_detector_params,
    rotation_matrix_to_quaternion,
)
from logitle_aruco_tools.logitle_aruco_tf_corrector import (
    T_to_transform,
    angle_diff,
    matrix_to_quaternion,
    median_angle,
    quaternion_to_matrix,
    transform_to_T,
)
from logitle_aruco_tools.logitle_marker_localization import (
    camera_mount_T,
    invert,
    make_T,
    map_odom_correction,
    matrix_to_rpy,
    rpy_to_matrix,
)
from logitle_aruco_tools.logitle_marker_map import load_marker_map


RESULT_SUCCESS = CorrectPoseWithAruco.Result.RESULT_SUCCESS
RESULT_TIMEOUT = CorrectPoseWithAruco.Result.RESULT_TIMEOUT
RESULT_CANCELED = CorrectPoseWithAruco.Result.RESULT_CANCELED
RESULT_MARKER_LOST = CorrectPoseWithAruco.Result.RESULT_MARKER_LOST
RESULT_INVALID_GOAL = CorrectPoseWithAruco.Result.RESULT_INVALID_GOAL
RESULT_NO_ODOM_TF = CorrectPoseWithAruco.Result.RESULT_NO_ODOM_TF
RESULT_UNSTABLE_SAMPLES = CorrectPoseWithAruco.Result.RESULT_UNSTABLE_SAMPLES


def positive_or_default(value, default):
    return float(value) if float(value) > 0.0 else float(default)


def int_or_default(value, default):
    return int(value) if int(value) > 0 else int(default)


def bool_arg(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"expected a boolean value, got {value!r}")


def default_robot_namespace():
    return ""


def scoped_topic(namespace, name):
    return f"/{namespace}/{name}" if namespace else f"/{name}"


def scoped_frame(namespace, name):
    return f"{namespace}/{name}" if namespace else name


def circular_std_deg(angles_rad):
    if len(angles_rad) == 0:
        return 0.0
    angles = np.asarray(angles_rad, dtype=float)
    mean_cos = float(np.mean(np.cos(angles)))
    mean_sin = float(np.mean(np.sin(angles)))
    r = max(1e-12, min(1.0, math.hypot(mean_cos, mean_sin)))
    return float(math.degrees(math.sqrt(-2.0 * math.log(r))))


class ArucoTfListenerNode(Node):
    """Receive TF updates on a lightweight single-threaded executor."""

    def __init__(self):
        # Ignore launch remaps such as __node so this helper keeps its own name.
        super().__init__("aruco_tf_listener", use_global_arguments=False)
        self.tf_buffer = Buffer()
        self.tf_listener = None
        self.listener_lock = threading.Lock()

    def start_listening(self):
        """Subscribe to /tf and /tf_static only while a correction goal runs."""
        with self.listener_lock:
            if self.tf_listener is None:
                self.tf_buffer.clear()
                self.tf_listener = TransformListener(self.tf_buffer, self)

    def stop_listening(self):
        with self.listener_lock:
            if self.tf_listener is not None:
                self.tf_listener.unregister()
                self.tf_listener = None


class ArucoPoseCorrectorActionServer(Node):
    def __init__(self, args, tf_node):
        super().__init__("aruco_pose_corrector_action_server")
        self.tf_node = tf_node
        self.args = args
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.active_tf_lock = threading.Lock()
        self.active_map_odom = None
        self.latest_poses = {}
        self.pose_subs = {}
        self.pose_input_lock = threading.Lock()
        self.active_marker_id = None
        self.camera_info_sub = None
        self.image_sub = None
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_shape = None
        self.detection_period_sec = 1.0 / max(float(args.detection_rate_hz), 1.0)
        self.last_detection_time = 0.0

        self.markers = load_marker_map(args.marker_map)
        self.validate_camera_mount_args(args)
        self.T_base_camera = camera_mount_T(
            args.camera_x,
            args.camera_y,
            args.camera_z,
            pitch_deg=args.camera_pitch,
            yaw_deg=args.camera_yaw,
            roll_deg=args.camera_roll,
        )
        if args.pose_source == "camera":
            dictionary_id = ARUCO_DICTS[args.dictionary]
            self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.detector_params = make_detector_params()
        else:
            self.dictionary = None
            self.detector_params = None

        self.tf_buffer = tf_node.tf_buffer
        self.tf_broadcaster = TransformBroadcaster(self)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            args.initialpose_topic,
            10,
        )
        # Created only while a direct map->odom TF is active (publish_tf mode).
        self.tf_publish_timer = None
        self.action_server = ActionServer(
            self,
            CorrectPoseWithAruco,
            args.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"Serving {args.action_name}; marker_map={args.marker_map}; "
            f"pose_source={args.pose_source}; image_topic={args.image_topic}; "
            f"pose_topic_template={args.pose_topic_template}; "
            f"initialpose_topic={args.initialpose_topic}; "
            f"allow_tf_publish={args.allow_tf_publish}; "
            f"camera=({args.camera_x:.3f}, {args.camera_y:.3f}, {args.camera_z:.3f}, "
            f"pitch={args.camera_pitch:.1f}deg)"
        )

    def goal_callback(self, goal_request):
        marker_id = int(goal_request.marker_id) if goal_request.marker_id > 0 else self.args.marker_id
        if marker_id not in self.markers:
            self.get_logger().warn(f"Rejecting goal: marker_id={marker_id} is not in marker_map")
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.goal_active:
                self.get_logger().warn("Rejecting goal: another pose correction is already active")
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        marker_id = int(goal.marker_id) if goal.marker_id > 0 else self.args.marker_id
        marker = self.markers.get(marker_id)
        if marker is None:
            result = self.make_result(False, RESULT_INVALID_GOAL, f"marker_id={marker_id} not in marker_map")
            goal_handle.abort()
            self.release_goal()
            return result

        required_samples = int_or_default(goal.required_samples, self.args.required_samples)
        timeout_sec = positive_or_default(goal.timeout_sec, self.args.timeout_sec)
        outlier_z_max = positive_or_default(goal.outlier_z_max, self.args.outlier_z_max)
        outlier_tilt_deg = positive_or_default(goal.outlier_tilt_deg, self.args.outlier_tilt_deg)
        check_yaw = bool(goal.check_yaw)
        expected_yaw_deg = float(goal.expected_base_yaw_deg)
        yaw_tolerance_deg = positive_or_default(goal.yaw_tolerance_deg, self.args.yaw_tolerance_deg)

        self.tf_node.start_listening()
        self.start_pose_input(marker_id)
        self.get_logger().info(
            f"Accepted pose correction goal: marker_id={marker_id} publish_tf={goal.publish_tf} "
            f"check_yaw={check_yaw} expected_yaw={expected_yaw_deg:+.1f}deg "
            f"required_samples={required_samples} timeout={timeout_sec:.1f}s"
        )

        odom_window = deque(maxlen=required_samples)
        base_window = deque(maxlen=required_samples)
        accepted = 0
        rejected = 0
        last_pose_was_seen = False
        last_feedback_time = 0.0
        last_pose_index = -1
        last_base_pose = None
        last_yaw_error_deg = 0.0
        last_state = "waiting_for_marker"
        method_counts = {"two_marker": 0, "single_marker": 0}
        started = time.time()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result = self.make_result(
                    False,
                    RESULT_CANCELED,
                    "Pose correction canceled.",
                    accepted,
                    rejected,
                    last_base_pose,
                    self.median_map_odom(odom_window),
                )
                goal_handle.canceled()
                self.release_goal()
                return result

            now = time.time()
            if now - started > timeout_sec:
                result_code = RESULT_NO_ODOM_TF if last_state == "waiting_for_odom_tf" else RESULT_TIMEOUT
                if result_code == RESULT_NO_ODOM_TF:
                    message = "Pose correction timed out waiting for odom->base_link TF."
                elif last_state == "rejecting_yaw":
                    message = (
                        "Pose correction timed out because samples were rejected by yaw. "
                        "Check marker yaw, expected_base_yaw_deg, yaw_tolerance_deg, or set check_yaw=false."
                    )
                elif last_state == "rejecting_outliers":
                    message = (
                        "Pose correction timed out because samples were rejected as outliers. "
                        "Check marker map values, marker size, camera extrinsic, and outlier thresholds."
                    )
                else:
                    message = "Pose correction timed out."
                result = self.make_result(
                    False,
                    result_code,
                    message,
                    accepted,
                    rejected,
                    last_base_pose,
                    self.median_map_odom(odom_window),
                )
                goal_handle.abort()
                self.release_goal()
                return result

            pose = self.get_latest_pose(marker_id)
            marker_visible = pose is not None and (now - pose["received_at"]) <= self.args.pose_timeout
            if not marker_visible:
                if now - last_feedback_time >= self.args.feedback_period:
                    state = "waiting_for_marker" if not last_pose_was_seen else "marker_lost"
                    last_state = state
                    goal_handle.publish_feedback(
                        self.make_feedback(state, False, accepted, rejected, last_base_pose, last_yaw_error_deg)
                    )
                    last_feedback_time = now
                time.sleep(self.args.control_period)
                continue

            last_pose_was_seen = True
            if pose["index"] == last_pose_index:
                time.sleep(self.args.control_period)
                continue
            last_pose_index = pose["index"]

            sample = self.compute_sample(
                pose["msg"],
                marker,
                outlier_z_max,
                outlier_tilt_deg,
                check_yaw,
                expected_yaw_deg,
                yaw_tolerance_deg,
                aux=pose.get("aux"),
            )
            if sample["accepted"]:
                accepted += 1
                method_counts[sample.get("method", "single_marker")] += 1
                last_base_pose = sample["T_map_base"]
                last_yaw_error_deg = sample["yaw_error_deg"]
                base_window.append(sample["T_map_base"])
                odom_window.append(sample["T_map_odom"])
                state = "collecting_samples"
            else:
                rejected += 1
                last_yaw_error_deg = sample["yaw_error_deg"]
                state = sample["state"]
            last_state = state

            if now - last_feedback_time >= self.args.feedback_period:
                goal_handle.publish_feedback(
                    self.make_feedback(state, True, accepted, rejected, last_base_pose, last_yaw_error_deg)
                )
                last_feedback_time = now

            if accepted >= required_samples and len(odom_window) >= required_samples:
                T_map_base = self.median_map_base(base_window)
                T_map_odom = self.median_map_odom(odom_window)
                xy_std, yaw_std_deg = self.sample_spread(base_window)
                if xy_std > self.args.max_xy_std or yaw_std_deg > self.args.max_yaw_std_deg:
                    result = self.make_result(
                        False,
                        RESULT_UNSTABLE_SAMPLES,
                        (
                            "Pose correction rejected because accepted samples are unstable "
                            f"(xy_std={xy_std:.3f}m, yaw_std={yaw_std_deg:.2f}deg)."
                        ),
                        accepted,
                        rejected,
                        T_map_base,
                        T_map_odom,
                        xy_std,
                        yaw_std_deg,
                    )
                    goal_handle.abort()
                    self.release_goal()
                    return result

                if goal.publish_tf:
                    if self.should_publish_tf_directly():
                        with self.active_tf_lock:
                            self.active_map_odom = np.array(T_map_odom, copy=True)
                        self.ensure_tf_publish_timer()
                        self.publish_active_tf()
                        message = "Pose correction succeeded; map->odom TF will continue publishing."
                    else:
                        self.clear_active_tf()
                        self.publish_initialpose(T_map_base)
                        message = (
                            "Pose correction succeeded; published /initialpose for AMCL/Nav2. "
                            "Direct map->odom TF publishing is disabled."
                        )
                else:
                    message = "Pose correction succeeded in dry-run mode."
                message += (
                    f" [pose samples: two_marker={method_counts['two_marker']}, "
                    f"single_marker={method_counts['single_marker']}]"
                )

                result = self.make_result(
                    True,
                    RESULT_SUCCESS,
                    message,
                    accepted,
                    rejected,
                    T_map_base,
                    T_map_odom,
                    xy_std,
                    yaw_std_deg,
                )
                goal_handle.succeed()
                self.release_goal()
                return result

            time.sleep(self.args.control_period)

        result = self.make_result(
            False,
            RESULT_MARKER_LOST,
            "ROS shutdown before pose correction completed.",
            accepted,
            rejected,
            last_base_pose,
            self.median_map_odom(odom_window),
        )
        goal_handle.abort()
        self.release_goal()
        return result

    def release_goal(self):
        self.stop_pose_input()
        self.tf_node.stop_listening()
        with self.goal_lock:
            self.goal_active = False

    def validate_camera_mount_args(self, args):
        numeric = {
            "camera_x": args.camera_x,
            "camera_y": args.camera_y,
            "camera_z": args.camera_z,
            "camera_pitch": args.camera_pitch,
            "camera_yaw": args.camera_yaw,
            "camera_roll": args.camera_roll,
            "max_xy_std": args.max_xy_std,
            "max_yaw_std_deg": args.max_yaw_std_deg,
        }
        for name, value in numeric.items():
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite, got {value!r}")
        if args.camera_z <= 0.0:
            raise ValueError(f"camera_z must be positive, got {args.camera_z}")
        if args.max_xy_std <= 0.0:
            raise ValueError(f"max_xy_std must be positive, got {args.max_xy_std}")
        if args.max_yaw_std_deg <= 0.0:
            raise ValueError(f"max_yaw_std_deg must be positive, got {args.max_yaw_std_deg}")

    def should_publish_tf_directly(self):
        if not self.args.allow_tf_publish:
            return False
        if self.args.force_tf_publish:
            return True
        if self.has_active_tf():
            return True
        return not self.map_odom_transform_available()

    def has_active_tf(self):
        with self.active_tf_lock:
            return self.active_map_odom is not None

    def map_odom_transform_available(self):
        try:
            self.tf_buffer.lookup_transform(
                self.args.map_frame,
                self.args.odom_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
            self.get_logger().warn(
                f"Existing {self.args.map_frame}->{self.args.odom_frame} TF detected; "
                "publishing /initialpose instead of a second map->odom publisher."
            )
            return True
        except TransformException:
            return False

    def ensure_tf_publish_timer(self):
        with self.active_tf_lock:
            if self.tf_publish_timer is None:
                self.tf_publish_timer = self.create_timer(
                    0.1,
                    self.publish_active_tf,
                    callback_group=self.callback_group,
                )

    def clear_active_tf(self):
        with self.active_tf_lock:
            self.active_map_odom = None
            timer = self.tf_publish_timer
            self.tf_publish_timer = None
        if timer is not None:
            self.destroy_timer(timer)

    def publish_active_tf(self):
        with self.active_tf_lock:
            T_map_odom = None if self.active_map_odom is None else np.array(self.active_map_odom, copy=True)
        if T_map_odom is None:
            return
        self.tf_broadcaster.sendTransform(
            T_to_transform(
                T_map_odom,
                self.get_clock().now().to_msg(),
                self.args.map_frame,
                self.args.odom_frame,
            )
        )

    def publish_initialpose(self, T_map_base):
        if T_map_base is None:
            return

        qx, qy, qz, qw = matrix_to_quaternion(T_map_base[:3, :3])
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.map_frame
        msg.pose.pose.position.x = float(T_map_base[0, 3])
        msg.pose.pose.position.y = float(T_map_base[1, 3])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        xy_var = float(self.args.initialpose_xy_std) ** 2
        yaw_var = math.radians(float(self.args.initialpose_yaw_std_deg)) ** 2
        msg.pose.covariance[0] = xy_var
        msg.pose.covariance[7] = xy_var
        msg.pose.covariance[35] = yaw_var
        self.initialpose_pub.publish(msg)

    def ensure_pose_subscription(self, marker_id):
        if marker_id in self.pose_subs:
            return

        topic = self.pose_topic_for_marker(marker_id)
        self.pose_subs[marker_id] = self.create_subscription(
            PoseStamped,
            topic,
            lambda msg, marker_id=marker_id: self.pose_cb(marker_id, msg),
            10,
            callback_group=self.callback_group,
        )
        self.get_logger().info(f"Subscribed to {topic} for marker_id={marker_id}")

    def start_pose_input(self, marker_id):
        with self.lock:
            self.latest_poses.pop(marker_id, None)
        self.last_detection_time = 0.0

        if self.args.pose_source == "topic":
            self.ensure_pose_subscription(marker_id)
            with self.pose_input_lock:
                self.active_marker_id = marker_id
            return

        with self.pose_input_lock:
            self.active_marker_id = marker_id
            if self.camera_info_sub is None:
                self.camera_info_sub = self.create_subscription(
                    CameraInfo,
                    self.args.camera_info_topic,
                    self.camera_info_cb,
                    10,
                    callback_group=self.callback_group,
                )
            if self.image_sub is None:
                self.image_sub = self.create_subscription(
                    Image,
                    self.args.image_topic,
                    self.image_cb,
                    10,
                    callback_group=self.callback_group,
                )
        self.get_logger().info(
            f"Started on-demand ArUco detection for marker_id={marker_id} "
            f"from {self.args.image_topic}"
        )

    def stop_pose_input(self):
        if self.args.pose_source == "topic":
            with self.pose_input_lock:
                marker_id = self.active_marker_id
                self.active_marker_id = None
            if marker_id in self.pose_subs:
                self.destroy_subscription(self.pose_subs.pop(marker_id))
                self.get_logger().info(f"Unsubscribed from pose input for marker_id={marker_id}")
            return

        with self.pose_input_lock:
            marker_id = self.active_marker_id
            self.active_marker_id = None
            image_sub = self.image_sub
            camera_info_sub = self.camera_info_sub
            self.image_sub = None
            self.camera_info_sub = None

        if image_sub is not None:
            self.destroy_subscription(image_sub)
        if camera_info_sub is not None:
            self.destroy_subscription(camera_info_sub)
        if marker_id is not None:
            self.get_logger().info(f"Stopped on-demand ArUco detection for marker_id={marker_id}")

    def pose_topic_for_marker(self, marker_id):
        template = self.args.pose_topic_template
        try:
            return template.format(marker_id=marker_id)
        except (KeyError, IndexError, ValueError):
            self.get_logger().warn(f"Invalid pose topic template '{template}', using it literally")
            return template

    def pose_cb(self, marker_id, msg):
        with self.pose_input_lock:
            if self.args.pose_source == "topic" and self.active_marker_id != marker_id:
                return
        self.store_latest_pose(marker_id, msg)

    def store_latest_pose(self, marker_id, msg, aux=None):
        with self.lock:
            prev = self.latest_poses.get(marker_id, {})
            self.latest_poses[marker_id] = {
                "index": int(prev.get("index", -1)) + 1,
                "received_at": time.time(),
                "msg": msg,
                "aux": aux,
            }

    def camera_info_cb(self, msg):
        if len(msg.k) != 9 or msg.k[0] == 0.0:
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float32)
        self.camera_info_shape = (msg.width, msg.height)

    def image_cb(self, msg):
        with self.pose_input_lock:
            marker_id = self.active_marker_id
        if marker_id is None:
            return

        now = time.monotonic()
        if now - self.last_detection_time < self.detection_period_sec:
            return
        self.last_detection_time = now

        if self.camera_matrix is None:
            if self.args.approx_camera_info:
                self.set_approx_camera_info(msg.width, msg.height)
            else:
                return
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame,
            self.dictionary,
            parameters=self.detector_params,
        )
        if ids is None:
            return

        ids_list = ids.flatten().tolist()
        if marker_id not in ids_list:
            return

        idx = ids_list.index(marker_id)
        marker_size = self.marker_size_for(marker_id)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corners[idx]],
            marker_size,
            self.camera_matrix,
            self.dist_coeffs,
        )
        pose = self.make_pose_msg(msg.header.stamp, rvecs[0][0], tvecs[0][0])

        # A small marker seen at an angle has two plausible orientations (IPPE
        # ambiguity), so keep both. Translations of other map markers in view
        # allow a flip-free two-marker pose.
        candidates = self.estimate_marker_candidates(corners[idx], marker_size)
        others = {}
        for j, other_id in enumerate(ids_list):
            if j == idx or other_id not in self.markers:
                continue
            other_candidates = self.estimate_marker_candidates(corners[j], self.marker_size_for(other_id))
            if other_candidates:
                others[int(other_id)] = other_candidates[0][1]
        self.store_latest_pose(marker_id, pose, aux={"candidates": candidates, "others": others})

    def estimate_marker_candidates(self, corner, marker_size):
        """Return [(R, t, reprojection_error), ...] for both IPPE solutions."""
        half = marker_size / 2.0
        object_points = np.array(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float32,
        )
        image_points = np.asarray(corner, dtype=np.float32).reshape(4, 2)
        try:
            count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error:
            return []
        errors = np.ravel(errors) if errors is not None else np.zeros(count)
        candidates = []
        for i in range(count):
            R, _ = cv2.Rodrigues(rvecs[i])
            candidates.append((R, np.asarray(tvecs[i], dtype=float).reshape(3), float(errors[i])))
        return candidates

    def marker_size_for(self, marker_id):
        marker = self.markers.get(marker_id)
        if marker is not None and marker.size > 0.0:
            return float(marker.size)
        return float(self.args.marker_size)

    def make_pose_msg(self, stamp, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.args.pose_frame_id
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def set_approx_camera_info(self, width, height):
        if width <= 0 or height <= 0:
            return

        hfov = math.radians(self.args.approx_horizontal_fov_deg)
        fx = width / (2.0 * math.tan(hfov / 2.0))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)
        self.camera_info_shape = (width, height)
        self.get_logger().warn(
            "Using approximate camera intrinsics for on-demand ArUco detection; "
            "calibrate camera before final TF correction."
        )

    def get_latest_pose(self, marker_id):
        with self.lock:
            pose = self.latest_poses.get(marker_id)
            return dict(pose) if pose is not None else None

    def compute_sample(
        self,
        msg,
        marker,
        outlier_z_max,
        outlier_tilt_deg,
        check_yaw,
        expected_yaw_deg,
        yaw_tolerance_deg,
        aux=None,
    ):
        try:
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.args.odom_frame,
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.args.tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().warn(f"waiting for {self.args.odom_frame}->{self.args.base_frame}: {exc}")
            return {
                "accepted": False,
                "state": "waiting_for_odom_tf",
                "yaw_error_deg": 0.0,
            }

        T_map_base_raw, method = self.solve_map_base(msg, marker, aux, check_yaw, expected_yaw_deg)
        roll, pitch, yaw = matrix_to_rpy(T_map_base_raw[:3, :3])
        base_z = float(T_map_base_raw[2, 3])
        tilt_limit = math.radians(outlier_tilt_deg)
        z_bad = abs(base_z) > outlier_z_max
        tilt_bad = abs(roll) > tilt_limit or abs(pitch) > tilt_limit

        expected_yaw = math.radians(expected_yaw_deg)
        yaw_error_deg = abs(math.degrees(angle_diff(yaw, expected_yaw)))
        yaw_bad = check_yaw and yaw_error_deg > yaw_tolerance_deg

        if z_bad or tilt_bad or yaw_bad:
            state = "rejecting_outliers"
            if yaw_bad:
                state = "rejecting_yaw"
            return {
                "accepted": False,
                "state": state,
                "yaw_error_deg": float(yaw_error_deg),
            }

        x = float(T_map_base_raw[0, 3])
        y = float(T_map_base_raw[1, 3])
        T_map_base = make_T(rpy_to_matrix(0.0, 0.0, yaw), [x, y, 0.0])
        T_odom_base = transform_to_T(tf_odom_base)
        T_map_odom = map_odom_correction(T_map_base, T_odom_base)

        return {
            "accepted": True,
            "state": "collecting_samples",
            "T_map_base": T_map_base,
            "T_map_odom": T_map_odom,
            "yaw_error_deg": float(yaw_error_deg),
            "method": method,
        }

    def solve_map_base(self, msg, marker, aux, check_yaw, expected_yaw_deg):
        aux = aux or {}
        candidates = aux.get("candidates") or []
        others = aux.get("others") or {}

        T_pair = self.two_marker_map_base(marker, candidates, others)
        if T_pair is not None:
            return T_pair, "two_marker"

        if candidates:
            options = []
            for R, t, error in candidates:
                T = marker.T_map_marker @ invert(make_T(R, t)) @ invert(self.T_base_camera)
                options.append((T, error))
            if check_yaw:
                expected_yaw = math.radians(expected_yaw_deg)
                T, _ = min(
                    options,
                    key=lambda option: abs(angle_diff(matrix_to_rpy(option[0][:3, :3])[2], expected_yaw)),
                )
            else:
                T, _ = min(options, key=lambda option: option[1])
            return T, "single_marker"

        T_camera_marker = self.pose_to_T(msg)
        return marker.T_map_marker @ invert(T_camera_marker) @ invert(self.T_base_camera), "single_marker"

    def two_marker_map_base(self, marker, candidates, others):
        """Planar map->base pose from two marker positions; orientation-free."""
        if not candidates or not others:
            return None
        m1 = np.asarray(marker.T_map_marker[:2, 3], dtype=float)
        b1 = (self.T_base_camera @ np.append(candidates[0][1], 1.0))[:2]
        for other_id, t_other in others.items():
            other = self.markers.get(other_id)
            if other is None:
                continue
            m2 = np.asarray(other.T_map_marker[:2, 3], dtype=float)
            b2 = (self.T_base_camera @ np.append(t_other, 1.0))[:2]
            dm = m2 - m1
            db = b2 - b1
            map_dist = float(np.linalg.norm(dm))
            seen_dist = float(np.linalg.norm(db))
            if map_dist < self.args.two_marker_min_dist:
                continue
            if abs(map_dist - seen_dist) > self.args.two_marker_dist_tolerance:
                self.get_logger().warn(
                    f"Skipping two-marker pose with ID{other_id}: map distance {map_dist:.3f}m "
                    f"vs seen {seen_dist:.3f}m; check marker_map",
                    throttle_duration_sec=5.0,
                )
                continue
            yaw = math.atan2(dm[1], dm[0]) - math.atan2(db[1], db[0])
            c, s_ = math.cos(yaw), math.sin(yaw)
            Rz = np.array([[c, -s_], [s_, c]])
            xy = ((m1 - Rz @ b1) + (m2 - Rz @ b2)) / 2.0
            return make_T(rpy_to_matrix(0.0, 0.0, yaw), [float(xy[0]), float(xy[1]), 0.0])
        return None

    def pose_to_T(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        return make_T(
            quaternion_to_matrix(q.x, q.y, q.z, q.w),
            [p.x, p.y, p.z],
        )

    @staticmethod
    def median_map_odom(window):
        if not window:
            return None
        xs = [float(T[0, 3]) for T in window]
        ys = [float(T[1, 3]) for T in window]
        yaws = [matrix_to_rpy(T[:3, :3])[2] for T in window]
        return make_T(
            rpy_to_matrix(0.0, 0.0, median_angle(np.array(yaws))),
            [float(np.median(xs)), float(np.median(ys)), 0.0],
        )

    @staticmethod
    def median_map_base(window):
        if not window:
            return None
        xs = [float(T[0, 3]) for T in window]
        ys = [float(T[1, 3]) for T in window]
        yaws = [matrix_to_rpy(T[:3, :3])[2] for T in window]
        return make_T(
            rpy_to_matrix(0.0, 0.0, median_angle(np.array(yaws))),
            [float(np.median(xs)), float(np.median(ys)), 0.0],
        )

    @staticmethod
    def sample_spread(window):
        if not window:
            return 0.0, 0.0
        xs = np.array([float(T[0, 3]) for T in window], dtype=float)
        ys = np.array([float(T[1, 3]) for T in window], dtype=float)
        yaws = np.array([matrix_to_rpy(T[:3, :3])[2] for T in window], dtype=float)
        xy_std = float(math.hypot(float(np.std(xs)), float(np.std(ys))))
        return xy_std, circular_std_deg(yaws)

    def make_feedback(self, state, marker_visible, accepted, rejected, T_map_base, yaw_error_deg):
        feedback = CorrectPoseWithAruco.Feedback()
        feedback.state = state
        feedback.marker_visible = bool(marker_visible)
        feedback.accepted_samples = int(accepted)
        feedback.rejected_samples = int(rejected)
        feedback.latest_yaw_error_deg = float(yaw_error_deg)
        if T_map_base is not None:
            feedback.latest_map_base_x = float(T_map_base[0, 3])
            feedback.latest_map_base_y = float(T_map_base[1, 3])
            feedback.latest_map_base_yaw_deg = float(math.degrees(matrix_to_rpy(T_map_base[:3, :3])[2]))
        return feedback

    def make_result(
        self,
        success,
        result_code,
        message,
        accepted=0,
        rejected=0,
        T_map_base=None,
        T_map_odom=None,
        map_base_xy_std=0.0,
        map_base_yaw_std_deg=0.0,
    ):
        result = CorrectPoseWithAruco.Result()
        result.success = bool(success)
        result.result_code = int(result_code)
        result.message = message
        result.accepted_samples = int(accepted)
        result.rejected_samples = int(rejected)
        result.map_base_xy_std = float(map_base_xy_std)
        result.map_base_yaw_std_deg = float(map_base_yaw_std_deg)

        if T_map_base is not None:
            result.map_base_x = float(T_map_base[0, 3])
            result.map_base_y = float(T_map_base[1, 3])
            result.map_base_yaw_deg = float(math.degrees(matrix_to_rpy(T_map_base[:3, :3])[2]))

        if T_map_odom is not None:
            result.map_odom_x = float(T_map_odom[0, 3])
            result.map_odom_y = float(T_map_odom[1, 3])
            result.map_odom_yaw_deg = float(math.degrees(matrix_to_rpy(T_map_odom[:3, :3])[2]))

        return result

    def destroy_node(self):
        if self.tf_publish_timer is not None:
            self.tf_publish_timer.cancel()
        self.stop_pose_input()
        self.action_server.destroy()
        super().destroy_node()


def parse_args():
    robot_namespace = default_robot_namespace()
    parser = argparse.ArgumentParser()
    parser.add_argument("--action-name", default=scoped_topic(robot_namespace, "aruco_correct_pose"))
    parser.add_argument(
        "--pose-source",
        choices=("camera", "topic"),
        default="camera",
        help=(
            "camera: subscribe to image topics and run ArUco detection only while an Action goal is active. "
            "topic: consume an external pose topic, also subscribed only while a goal is active."
        ),
    )
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--detection-rate-hz", type=float, default=10.0)
    parser.add_argument("--dictionary", choices=sorted(ARUCO_DICTS), default="5X5_1000")
    parser.add_argument(
        "--pose-topic-template",
        default=scoped_topic(robot_namespace, "aruco/id{marker_id}/pose_camera"),
    )
    parser.add_argument("--pose-frame-id", default="camera_optical_frame")
    parser.add_argument("--marker-map", default="logitle_marker_map.yaml")
    parser.add_argument("--marker-id", type=int, default=24)
    parser.add_argument("--marker-size", type=float, default=0.04)
    parser.add_argument(
        "--approx-camera-info",
        type=bool_arg,
        default=False,
        help="Use approximate intrinsics when camera_info has empty K values.",
    )
    parser.add_argument("--approx-horizontal-fov-deg", type=float, default=62.2)

    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--initialpose-topic", default="/initialpose")
    parser.add_argument("--initialpose-xy-std", type=float, default=0.05)
    parser.add_argument("--initialpose-yaw-std-deg", type=float, default=5.0)
    parser.add_argument(
        "--allow-tf-publish",
        type=bool_arg,
        default=False,
        help=(
            "Allow this action server to publish map->odom directly. "
            "Keep disabled when AMCL/Nav2 owns map->odom."
        ),
    )
    parser.add_argument(
        "--force-tf-publish",
        type=bool_arg,
        default=False,
        help="Force direct map->odom publishing even if another map->odom TF is already visible.",
    )

    parser.add_argument("--camera-x", type=float, default=0.045)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.115)
    parser.add_argument("--camera-pitch", type=float, default=-5.0)
    parser.add_argument("--camera-yaw", type=float, default=0.0)
    parser.add_argument("--camera-roll", type=float, default=0.0)

    parser.add_argument("--required-samples", type=int, default=15)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--outlier-z-max", type=float, default=0.15)
    parser.add_argument("--outlier-tilt-deg", type=float, default=25.0)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=4.0)
    parser.add_argument("--max-xy-std", type=float, default=0.03)
    parser.add_argument("--max-yaw-std-deg", type=float, default=3.0)

    parser.add_argument("--tf-timeout", type=float, default=0.05)
    parser.add_argument("--pose-timeout", type=float, default=0.5)
    parser.add_argument("--feedback-period", type=float, default=0.2)
    parser.add_argument("--two-marker-min-dist", type=float, default=0.15)
    parser.add_argument("--two-marker-dist-tolerance", type=float, default=0.05)
    parser.add_argument("--control-period", type=float, default=0.03)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    rclpy.init()
    tf_node = ArucoTfListenerNode()
    node = ArucoPoseCorrectorActionServer(args, tf_node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    tf_executor = SingleThreadedExecutor()
    tf_executor.add_node(tf_node)
    tf_thread = threading.Thread(target=tf_executor.spin, name="aruco_tf_listener", daemon=True)
    tf_thread.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        tf_executor.shutdown()
        tf_thread.join(timeout=2.0)
        tf_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
