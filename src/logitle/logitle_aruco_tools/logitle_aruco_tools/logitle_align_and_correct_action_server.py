#!/usr/bin/env python3
"""Action server that aligns to an ArUco marker, then corrects pose."""

import argparse
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image

from logitle_aruco_msgs.action import AlignAndCorrectWithAruco, CorrectPoseWithAruco
from logitle_aruco_tools.logitle_aruco_pose_viewer import (
    ARUCO_DICTS,
    make_detector_params,
    rotation_matrix_to_quaternion,
)
from logitle_aruco_tools.logitle_marker_map import load_marker_map


RESULT_SUCCESS = AlignAndCorrectWithAruco.Result.RESULT_SUCCESS
RESULT_TIMEOUT = AlignAndCorrectWithAruco.Result.RESULT_TIMEOUT
RESULT_CANCELED = AlignAndCorrectWithAruco.Result.RESULT_CANCELED
RESULT_MARKER_LOST = AlignAndCorrectWithAruco.Result.RESULT_MARKER_LOST
RESULT_ALIGN_FAILED = AlignAndCorrectWithAruco.Result.RESULT_ALIGN_FAILED
RESULT_CORRECTION_FAILED = AlignAndCorrectWithAruco.Result.RESULT_CORRECTION_FAILED
RESULT_INVALID_GOAL = AlignAndCorrectWithAruco.Result.RESULT_INVALID_GOAL

ACTION_DEFAULT_TARGET_X = 0.0
ACTION_DEFAULT_TARGET_Z = 0.0
LEGACY_ACTION_DEFAULT_TARGET_X = -0.173
LEGACY_ACTION_DEFAULT_TARGET_Z = 0.392
ACTION_DEFAULT_EXPECTED_BASE_YAW_DEG = -87.0
ACTION_DEFAULT_YAW_TOLERANCE_DEG = 3.0

ALIGN_TARGET_PRESETS = {
    24: {
        "node": "N5",
        "target_x": -0.184,
        "target_z": 0.376,
        "check_yaw": True,
        "expected_base_yaw_deg": -87.0,
    },
    25: {
        "node": "N6",
        "target_x": -0.173,
        "target_z": 0.392,
        "check_yaw": True,
        "expected_base_yaw_deg": -87.0,
    },
    26: {
        "node": "N4",
        "target_x": -0.004,
        "target_z": 0.408,
        "check_yaw": False,
        "expected_base_yaw_deg": None,
    },
    27: {
        "node": "N3",
        "target_x": 0.004,
        "target_z": 0.412,
        "check_yaw": False,
        "expected_base_yaw_deg": None,
    },
    29: {
        "node": "N5",
        "target_x": 0.186,
        "target_z": 0.377,
        "check_yaw": True,
        "expected_base_yaw_deg": -87.0,
    },
}


def clamp(value, low, high):
    return max(low, min(high, value))


def positive_or_default(value, default):
    return float(value) if float(value) > 0.0 else float(default)


def finite_or_default(value, default):
    value = float(value)
    return value if math.isfinite(value) else float(default)


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


def is_close(value, default, eps=1e-6):
    return abs(float(value) - float(default)) <= eps


def default_robot_namespace():
    return ""


def scoped_topic(namespace, name):
    return f"/{namespace}/{name}" if namespace else f"/{name}"


def scoped_frame(namespace, name):
    return f"{namespace}/{name}" if namespace else name


def quaternion_to_matrix(q):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def marker_wall_yaw_error(q):
    """Return marker plane left/right angle in the camera optical frame."""
    rotation = quaternion_to_matrix(q)
    normal_x = rotation[0][2]
    normal_z = rotation[2][2]
    if normal_z < 0.0:
        normal_x = -normal_x
        normal_z = -normal_z
    return math.atan2(normal_x, max(abs(normal_z), 1e-6))


class AlignAndCorrectActionServer(Node):
    def __init__(self, args):
        super().__init__("align_and_correct_with_aruco_action_server")
        self.args = args
        self.callback_group = ReentrantCallbackGroup()
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.pose_lock = threading.Lock()
        self.latest_pose = None
        self.pose_sub = None
        self.camera_info_sub = None
        self.image_sub = None
        self.pose_input_lock = threading.Lock()
        self.active_marker_id = None
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_shape = None
        self.markers = load_marker_map(args.marker_map)
        if args.pose_source == "camera":
            dictionary_id = ARUCO_DICTS[args.dictionary]
            self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.detector_params = make_detector_params()
        else:
            self.dictionary = None
            self.detector_params = None

        cmd_type = TwistStamped if args.cmd_vel_stamped else Twist
        self.cmd_pub = self.create_publisher(cmd_type, args.cmd_vel_topic, 10)
        self.correct_client = ActionClient(
            self,
            CorrectPoseWithAruco,
            args.correct_action_name,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            AlignAndCorrectWithAruco,
            args.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"Serving {args.action_name}; correct_action={args.correct_action_name}; "
            f"cmd_vel={args.cmd_vel_topic}; cmd_type={cmd_type.__name__}; "
            f"pose_source={args.pose_source}; image_topic={args.image_topic}"
        )

    def goal_callback(self, goal_request):
        if int(goal_request.marker_id) <= 0 and self.args.marker_id <= 0:
            self.get_logger().warn("Rejecting goal: marker_id must be positive")
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.goal_active:
                self.get_logger().warn("Rejecting goal: another align/correct goal is already active")
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        marker_id = int(goal.marker_id) if goal.marker_id > 0 else self.args.marker_id
        params = self.params_from_goal(goal, marker_id)

        if not self.validate_params(params):
            result = self.make_result(False, RESULT_INVALID_GOAL, "Invalid align/correct goal values.")
            goal_handle.abort()
            self.release_goal()
            return result

        self.start_pose_input(marker_id)
        preset_text = f"; preset_node={params['preset_node']}" if params["preset_node"] else ""
        self.get_logger().info(
            f"Accepted align/correct goal: marker_id={marker_id}; "
            f"target_x={params['target_x']:.3f}m; target_z={params['target_z']:.3f}m; "
            f"check_yaw={params['check_yaw']}; "
            f"expected_base_yaw_deg={params['expected_base_yaw_deg']:.1f}; "
            f"yaw_tolerance_deg={params['yaw_tolerance_deg']:.1f}; "
            f"align_timeout={params['align_timeout_sec']:.1f}s; apply_correction={goal.apply_correction}"
            f"{preset_text}"
        )

        try:
            align_result = self.run_alignment(goal_handle, params)
            if goal_handle.is_cancel_requested:
                result = self.make_result(False, RESULT_CANCELED, "Align/correct canceled.", **align_result)
                goal_handle.canceled()
                return result

            if not align_result["align_success"]:
                code = RESULT_MARKER_LOST if align_result["marker_lost"] else RESULT_ALIGN_FAILED
                result = self.make_result(False, code, align_result["align_message"], **align_result)
                goal_handle.abort()
                return result

            if not goal.apply_correction:
                result = self.make_result(
                    True,
                    RESULT_SUCCESS,
                    "ArUco alignment succeeded; pose correction was skipped.",
                    **align_result,
                )
                goal_handle.succeed()
                return result

            self.stop_pose_input()
            correction = self.run_pose_correction(goal_handle, marker_id, params)
            merged = dict(align_result)
            merged.update(correction)
            if not correction["correction_success"]:
                result = self.make_result(
                    False,
                    RESULT_CORRECTION_FAILED,
                    "Alignment succeeded, but pose correction failed.",
                    **merged,
                )
                goal_handle.abort()
                return result

            result = self.make_result(
                True,
                RESULT_SUCCESS,
                "Alignment and pose correction succeeded.",
                **merged,
            )
            goal_handle.succeed()
            return result
        finally:
            self.stop_robot()
            self.stop_pose_input()
            self.release_goal()

    def params_from_goal(self, goal, marker_id):
        preset = ALIGN_TARGET_PRESETS.get(int(marker_id))
        target_x = finite_or_default(goal.target_x, self.args.target_x)
        target_z = positive_or_default(goal.target_z, self.args.target_z)
        check_yaw = bool(goal.check_yaw)
        expected_base_yaw_deg = finite_or_default(goal.expected_base_yaw_deg, ACTION_DEFAULT_EXPECTED_BASE_YAW_DEG)
        yaw_tolerance_deg = positive_or_default(goal.yaw_tolerance_deg, self.args.yaw_tolerance_deg)

        if preset:
            target_x_omitted = is_close(goal.target_x, ACTION_DEFAULT_TARGET_X) or is_close(
                goal.target_x,
                LEGACY_ACTION_DEFAULT_TARGET_X,
            )
            target_z_omitted = is_close(goal.target_z, ACTION_DEFAULT_TARGET_Z) or is_close(
                goal.target_z,
                LEGACY_ACTION_DEFAULT_TARGET_Z,
            )
            if target_x_omitted:
                target_x = float(preset["target_x"])
            if target_z_omitted:
                target_z = float(preset["target_z"])
            if bool(goal.check_yaw) and preset["check_yaw"] is False:
                check_yaw = False
            if (
                preset["expected_base_yaw_deg"] is not None
                and is_close(goal.expected_base_yaw_deg, ACTION_DEFAULT_EXPECTED_BASE_YAW_DEG)
            ):
                expected_base_yaw_deg = float(preset["expected_base_yaw_deg"])

        return {
            "timeout_sec": positive_or_default(goal.timeout_sec, self.args.timeout_sec),
            "align_timeout_sec": positive_or_default(goal.align_timeout_sec, self.args.align_timeout_sec),
            "correct_timeout_sec": positive_or_default(goal.correct_timeout_sec, self.args.correct_timeout_sec),
            "target_x": target_x,
            "target_z": target_z,
            "x_tolerance": positive_or_default(goal.x_tolerance, self.args.x_tolerance),
            "z_tolerance": positive_or_default(goal.z_tolerance, self.args.z_tolerance),
            "z_min_stop": positive_or_default(goal.z_min_stop, self.args.z_min_stop),
            "stable_sec": positive_or_default(goal.stable_sec, self.args.stable_sec),
            "max_linear": positive_or_default(goal.max_linear, self.args.max_linear),
            "max_angular": positive_or_default(goal.max_angular, self.args.max_angular),
            "kx": positive_or_default(goal.kx, self.args.kx),
            "kz": positive_or_default(goal.kz, self.args.kz),
            "check_wall_yaw": bool(goal.check_wall_yaw),
            "wall_yaw_tolerance": math.radians(
                positive_or_default(goal.wall_yaw_tolerance_deg, self.args.wall_yaw_tolerance_deg)
            ),
            "kyaw": positive_or_default(goal.kyaw, self.args.kyaw),
            "required_samples": int_or_default(goal.required_samples, self.args.required_samples),
            "check_yaw": check_yaw,
            "expected_base_yaw_deg": expected_base_yaw_deg,
            "yaw_tolerance_deg": yaw_tolerance_deg,
            "preset_node": preset["node"] if preset else "",
        }

    def validate_params(self, params):
        for key, value in params.items():
            if key in ("check_wall_yaw", "check_yaw", "preset_node"):
                continue
            if not math.isfinite(float(value)):
                return False
        return (
            params["z_min_stop"] < params["target_z"]
            and params["x_tolerance"] > 0.0
            and params["z_tolerance"] > 0.0
            and params["max_linear"] > 0.0
            and params["max_angular"] > 0.0
        )

    def run_alignment(self, goal_handle, params):
        started = time.time()
        aligned_since = None
        last_marker_seen = False
        last_feedback = 0.0
        final_x = 0.0
        final_z = 0.0
        final_x_error = 0.0
        final_z_error = 0.0
        final_wall_yaw_error = 0.0

        while rclpy.ok():
            now = time.time()
            if goal_handle.is_cancel_requested:
                return self.align_result(
                    False,
                    False,
                    final_x,
                    final_z,
                    final_x_error,
                    final_z_error,
                    final_wall_yaw_error,
                    "Canceled.",
                )

            if now - started > min(params["timeout_sec"], params["align_timeout_sec"]):
                self.stop_robot()
                message = "ArUco alignment timed out."
                if not last_marker_seen:
                    message = "ArUco alignment timed out waiting for marker."
                return self.align_result(
                    False,
                    not last_marker_seen,
                    final_x,
                    final_z,
                    final_x_error,
                    final_z_error,
                    final_wall_yaw_error,
                    message,
                )

            pose = self.get_latest_pose()
            marker_visible = pose is not None and (now - pose["received_at"]) <= self.args.pose_timeout
            if not marker_visible:
                aligned_since = None
                self.stop_robot()
                if now - last_feedback >= self.args.feedback_period:
                    self.publish_feedback(
                        goal_handle,
                        "waiting_for_marker",
                        False,
                        final_x,
                        final_z,
                        final_x_error,
                        final_z_error,
                        final_wall_yaw_error,
                        0.0,
                        0.0,
                        0.0,
                    )
                    last_feedback = now
                time.sleep(self.args.control_period)
                continue

            last_marker_seen = True
            msg = pose["msg"]
            x = float(msg.pose.position.x)
            z = float(msg.pose.position.z)
            wall_yaw_error = marker_wall_yaw_error(msg.pose.orientation)
            linear_x, angular_z, state, x_error, z_error, wall_yaw_error = self.compute_cmd(
                x,
                z,
                wall_yaw_error,
                params,
            )
            final_x = x
            final_z = z
            final_x_error = x_error
            final_z_error = z_error
            final_wall_yaw_error = wall_yaw_error

            if state == "too_close_stop":
                self.stop_robot()
                return self.align_result(
                    False,
                    False,
                    x,
                    z,
                    x_error,
                    z_error,
                    final_wall_yaw_error,
                    "Marker is too close; stopped before alignment.",
                )

            if state == "aligned":
                self.stop_robot()
                if aligned_since is None:
                    aligned_since = now
            else:
                aligned_since = None
                self.publish_cmd(linear_x, angular_z)

            aligned_duration = 0.0 if aligned_since is None else now - aligned_since
            if now - last_feedback >= self.args.feedback_period:
                self.publish_feedback(
                    goal_handle,
                    state,
                    True,
                    x,
                    z,
                    x_error,
                    z_error,
                    wall_yaw_error,
                    linear_x,
                    angular_z,
                    aligned_duration,
                )
                last_feedback = now

            if aligned_since is not None and aligned_duration >= params["stable_sec"]:
                self.stop_robot()
                return self.align_result(
                    True,
                    False,
                    x,
                    z,
                    x_error,
                    z_error,
                    final_wall_yaw_error,
                    "ArUco alignment succeeded.",
                )

            time.sleep(self.args.control_period)

        return self.align_result(
            False,
            False,
            final_x,
            final_z,
            final_x_error,
            final_z_error,
            final_wall_yaw_error,
            "ROS shutdown during alignment.",
        )

    def compute_cmd(self, x, z, wall_yaw_error, params):
        if z < params["z_min_stop"]:
            return (
                0.0,
                0.0,
                "too_close_stop",
                x - params["target_x"],
                z - params["target_z"],
                wall_yaw_error,
            )

        x_error = x - params["target_x"]
        if abs(x_error) <= params["x_tolerance"]:
            x_error = 0.0
        z_error = 0.0 if abs(z - params["target_z"]) <= params["z_tolerance"] else z - params["target_z"]
        if not params["check_wall_yaw"] or abs(wall_yaw_error) <= params["wall_yaw_tolerance"]:
            wall_yaw_error = 0.0
        if x_error == 0.0 and z_error == 0.0 and wall_yaw_error == 0.0:
            return 0.0, 0.0, "aligned", x_error, z_error, wall_yaw_error

        angular_z = clamp(
            -params["kx"] * x_error - params["kyaw"] * wall_yaw_error,
            -params["max_angular"],
            params["max_angular"],
        )
        linear_x = clamp(params["kz"] * z_error, -params["max_linear"], params["max_linear"])
        return linear_x, angular_z, "aligning", x_error, z_error, wall_yaw_error

    def run_pose_correction(self, goal_handle, marker_id, params):
        timeout_sec = float(params["correct_timeout_sec"])
        if not self.correct_client.wait_for_server(timeout_sec=timeout_sec):
            return {
                "correction_success": False,
                "correction_result_code": CorrectPoseWithAruco.Result.RESULT_TIMEOUT,
                "correction_message": f"Correction action server {self.args.correct_action_name} is not available.",
            }

        correct_goal = CorrectPoseWithAruco.Goal()
        correct_goal.marker_id = int(marker_id)
        correct_goal.publish_tf = True
        correct_goal.timeout_sec = float(timeout_sec)
        correct_goal.required_samples = int(params["required_samples"])
        correct_goal.check_yaw = bool(params["check_yaw"])
        correct_goal.expected_base_yaw_deg = float(params["expected_base_yaw_deg"])
        correct_goal.yaw_tolerance_deg = float(params["yaw_tolerance_deg"])

        send_future = self.correct_client.send_goal_async(
            correct_goal,
            feedback_callback=lambda feedback: self.publish_correction_feedback(goal_handle, feedback.feedback),
        )
        goal_response = self.wait_for_future(send_future, timeout_sec)
        if goal_response is None or not goal_response.accepted:
            return {
                "correction_success": False,
                "correction_result_code": CorrectPoseWithAruco.Result.RESULT_INVALID_GOAL,
                "correction_message": "Correction goal was rejected.",
            }

        result_future = goal_response.get_result_async()
        result_response = self.wait_for_future(result_future, timeout_sec)
        if result_response is None:
            return {
                "correction_success": False,
                "correction_result_code": CorrectPoseWithAruco.Result.RESULT_TIMEOUT,
                "correction_message": "Correction action timed out.",
            }

        result = result_response.result
        return {
            "correction_success": bool(result.success),
            "correction_result_code": int(result.result_code),
            "correction_message": str(result.message),
            "map_base_x": float(result.map_base_x),
            "map_base_y": float(result.map_base_y),
            "map_base_yaw_deg": float(result.map_base_yaw_deg),
            "map_base_xy_std": float(result.map_base_xy_std),
            "map_base_yaw_std_deg": float(result.map_base_yaw_std_deg),
        }

    def wait_for_future(self, future, timeout_sec):
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout=max(0.1, float(timeout_sec))):
            return None
        return future.result()

    def publish_correction_feedback(self, goal_handle, feedback):
        out = AlignAndCorrectWithAruco.Feedback()
        out.state = "correcting_pose"
        out.marker_visible = bool(feedback.marker_visible)
        out.accepted_samples = int(feedback.accepted_samples)
        out.rejected_samples = int(feedback.rejected_samples)
        out.correction_state = str(feedback.state)
        goal_handle.publish_feedback(out)

    def publish_feedback(
        self,
        goal_handle,
        state,
        marker_visible,
        marker_x,
        marker_z,
        x_error,
        z_error,
        wall_yaw_error,
        linear_x,
        angular_z,
        aligned_duration,
    ):
        feedback = AlignAndCorrectWithAruco.Feedback()
        feedback.state = state
        feedback.marker_visible = bool(marker_visible)
        feedback.latest_marker_x = float(marker_x)
        feedback.latest_marker_z = float(marker_z)
        feedback.latest_x_error = float(x_error)
        feedback.latest_z_error = float(z_error)
        feedback.latest_wall_yaw_error_deg = float(math.degrees(wall_yaw_error))
        feedback.command_linear_x = float(linear_x)
        feedback.command_angular_z = float(angular_z)
        feedback.aligned_duration_sec = float(aligned_duration)
        feedback.correction_state = ""
        goal_handle.publish_feedback(feedback)

    def align_result(self, success, marker_lost, marker_x, marker_z, x_error, z_error, wall_yaw_error, message):
        return {
            "align_success": bool(success),
            "marker_lost": bool(marker_lost),
            "final_marker_x": float(marker_x),
            "final_marker_z": float(marker_z),
            "final_x_error": float(x_error),
            "final_z_error": float(z_error),
            "final_wall_yaw_error_deg": float(math.degrees(wall_yaw_error)),
            "align_message": str(message),
        }

    def make_result(
        self,
        success,
        result_code,
        message,
        align_success=False,
        correction_success=False,
        final_marker_x=0.0,
        final_marker_z=0.0,
        final_x_error=0.0,
        final_z_error=0.0,
        final_wall_yaw_error_deg=0.0,
        correction_result_code=0,
        correction_message="",
        map_base_x=0.0,
        map_base_y=0.0,
        map_base_yaw_deg=0.0,
        map_base_xy_std=0.0,
        map_base_yaw_std_deg=0.0,
        marker_lost=False,
        align_message="",
    ):
        del marker_lost, align_message
        result = AlignAndCorrectWithAruco.Result()
        result.success = bool(success)
        result.result_code = int(result_code)
        result.message = str(message)
        result.align_success = bool(align_success)
        result.correction_success = bool(correction_success)
        result.final_marker_x = float(final_marker_x)
        result.final_marker_z = float(final_marker_z)
        result.final_x_error = float(final_x_error)
        result.final_z_error = float(final_z_error)
        result.final_wall_yaw_error_deg = float(final_wall_yaw_error_deg)
        result.correction_result_code = int(correction_result_code)
        result.correction_message = str(correction_message)
        result.map_base_x = float(map_base_x)
        result.map_base_y = float(map_base_y)
        result.map_base_yaw_deg = float(map_base_yaw_deg)
        result.map_base_xy_std = float(map_base_xy_std)
        result.map_base_yaw_std_deg = float(map_base_yaw_std_deg)
        return result

    def start_pose_input(self, marker_id):
        with self.pose_lock:
            self.latest_pose = None

        if self.args.pose_source == "topic":
            topic = self.pose_topic_for_marker(marker_id)
            self.pose_sub = self.create_subscription(
                PoseStamped,
                topic,
                self.pose_cb,
                10,
                callback_group=self.callback_group,
            )
            self.get_logger().info(f"Subscribed to {topic} for pickup alignment")
            return

        with self.pose_input_lock:
            self.active_marker_id = int(marker_id)
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
            f"Started on-demand ArUco alignment detection for marker_id={marker_id} "
            f"from {self.args.image_topic}"
        )

    def stop_pose_input(self):
        if self.pose_sub is not None:
            self.destroy_subscription(self.pose_sub)
            self.pose_sub = None
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
        if marker_id is not None and self.args.pose_source == "camera":
            self.get_logger().info(f"Stopped on-demand ArUco alignment detection for marker_id={marker_id}")
        with self.pose_lock:
            self.latest_pose = None

    def pose_cb(self, msg):
        self.store_latest_pose(msg)

    def store_latest_pose(self, msg):
        with self.pose_lock:
            self.latest_pose = {
                "received_at": time.time(),
                "msg": msg,
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
        self.store_latest_pose(self.make_pose_msg(msg.header.stamp, rvecs[0][0], tvecs[0][0]))

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
            "Using approximate camera intrinsics for ArUco alignment; "
            "calibrate camera before final operation."
        )

    def get_latest_pose(self):
        with self.pose_lock:
            return None if self.latest_pose is None else dict(self.latest_pose)

    def pose_topic_for_marker(self, marker_id):
        try:
            return self.args.pose_topic_template.format(marker_id=marker_id)
        except (KeyError, IndexError, ValueError):
            return self.args.pose_topic_template

    def publish_cmd(self, linear_x, angular_z):
        if self.args.cmd_vel_stamped:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = self.args.cmd_frame_id
            cmd.twist.linear.x = float(linear_x)
            cmd.twist.angular.z = float(angular_z)
        else:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        for _ in range(3):
            self.publish_cmd(0.0, 0.0)

    def release_goal(self):
        with self.goal_lock:
            self.goal_active = False

    def destroy_node(self):
        self.stop_robot()
        self.stop_pose_input()
        self.action_server.destroy()
        super().destroy_node()


def parse_args():
    robot_namespace = default_robot_namespace()
    parser = argparse.ArgumentParser()
    parser.add_argument("--action-name", default=scoped_topic(robot_namespace, "aruco_align_and_correct"))
    parser.add_argument("--correct-action-name", default=scoped_topic(robot_namespace, "aruco_correct_pose"))
    parser.add_argument(
        "--pose-source",
        choices=("camera", "topic"),
        default="camera",
        help=(
            "camera: subscribe to image topics only while an Action goal is active. "
            "topic: subscribe to an external pose publisher."
        ),
    )
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--dictionary", choices=sorted(ARUCO_DICTS), default="5X5_1000")
    parser.add_argument(
        "--pose-topic-template",
        default=scoped_topic(robot_namespace, "aruco/id{marker_id}/pose_camera"),
    )
    parser.add_argument("--pose-frame-id", default="camera_optical_frame")
    parser.add_argument("--marker-map", default="logitle_marker_map.yaml")
    parser.add_argument("--marker-id", type=int, default=24)
    parser.add_argument("--marker-size", type=float, default=0.04)
    parser.add_argument("--approx-camera-info", type=bool_arg, default=False)
    parser.add_argument("--approx-horizontal-fov-deg", type=float, default=62.2)

    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--cmd-vel-stamped", type=bool_arg, default=True)
    parser.add_argument("--cmd-frame-id", default="base_footprint")

    parser.add_argument("--target-x", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.0)
    parser.add_argument("--x-tolerance", type=float, default=0.025)
    parser.add_argument("--z-tolerance", type=float, default=0.025)
    parser.add_argument("--z-min-stop", type=float, default=0.25)
    parser.add_argument("--stable-sec", type=float, default=0.5)
    parser.add_argument("--max-linear", type=float, default=0.025)
    parser.add_argument("--max-angular", type=float, default=0.10)
    parser.add_argument("--kx", type=float, default=2.0)
    parser.add_argument("--kz", type=float, default=0.4)
    parser.add_argument("--wall-yaw-tolerance-deg", type=float, default=4.0)
    parser.add_argument("--kyaw", type=float, default=0.8)

    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--align-timeout-sec", type=float, default=25.0)
    parser.add_argument("--correct-timeout-sec", type=float, default=10.0)
    parser.add_argument("--required-samples", type=int, default=15)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=4.0)

    parser.add_argument("--pose-timeout", type=float, default=0.3)
    parser.add_argument("--feedback-period", type=float, default=0.2)
    parser.add_argument("--control-period", type=float, default=0.03)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    rclpy.init()
    node = AlignAndCorrectActionServer(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
