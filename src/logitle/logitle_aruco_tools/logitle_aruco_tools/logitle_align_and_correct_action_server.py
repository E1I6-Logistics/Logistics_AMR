#!/usr/bin/env python3
"""Action server that aligns to an ArUco marker, then corrects pose."""

import argparse
import math
import os
import shutil
import signal
import sys
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from logitle_aruco_msgs.action import AlignAndCorrectWithAruco, CorrectPoseWithAruco
from logitle_aruco_tools.logitle_aruco_pose_viewer import (
    ARUCO_DICTS,
    make_detector_params,
    rotation_matrix_to_quaternion,
)
from logitle_aruco_tools.logitle_marker_localization import camera_mount_T
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
ACTION_DEFAULT_Z_TOLERANCE = 0.007

ALIGN_TARGET_PRESETS = {
    24: {
        "node": "N5",
        "target_x": -0.189,
        "target_z": 0.376,
        "z_tolerance": 0.010,
        "check_yaw": True,
        "expected_base_yaw_deg": -87.0,
        # Two-marker yaw is accurate; alignment fixes only marker x/z, so a
        # laterally offset start can end several degrees off -87.
        "yaw_tolerance_deg": 8.0,
    },
    25: {
        "node": "N6",
        "target_x": -0.157,
        "target_z": 0.384,
        "z_tolerance": 0.010,
        "check_yaw": True,
        "expected_base_yaw_deg": -87.0,
        # Two-marker yaw is accurate; alignment fixes only marker x/z, so a
        # laterally offset start can end several degrees off -87.
        "yaw_tolerance_deg": 8.0,
    },
    28: {
        "node": "N6",
        "target_x": 0.153,
        "target_z": 0.361,
        "z_tolerance": 0.010,
        "check_yaw": True,
        "expected_base_yaw_deg": -87.0,
        # Two-marker yaw is accurate; alignment fixes only marker x/z, so a
        # laterally offset start can end several degrees off -87.
        "yaw_tolerance_deg": 8.0,
    },
    26: {
        "node": "N4",
        "target_x": -0.009,
        "target_z": 0.385,
        "z_tolerance": 0.010,
        "check_yaw": True,
        "expected_base_yaw_deg": 0.0,
    },
    # ID23 (added 2026-10-08, 13.3 cm left of ID26) also selects N4, so a goal
    # with either marker runs the two-marker wall alignment. target_x/z only
    # matter when --wall-align is off: derived from ID26 shifted by the
    # spacing, not measured.
    23: {
        "node": "N4",
        "target_x": -0.142,
        "target_z": 0.385,
        "z_tolerance": 0.010,
        "check_yaw": True,
        "expected_base_yaw_deg": 0.0,
    },
    27: {
        "node": "N3",
        "target_x": 0.001,
        "target_z": 0.397,
        "z_tolerance": 0.010,
        "check_yaw": True,
        "expected_base_yaw_deg": 0.0,
    },
    29: {
        "node": "N5",
        "target_x": 0.185,
        "target_z": 0.372,
        "z_tolerance": 0.010,
        "check_yaw": True,
        "expected_base_yaw_deg": -87.0,
        # Two-marker yaw is accurate; alignment fixes only marker x/z, so a
        # laterally offset start can end several degrees off -87.
        "yaw_tolerance_deg": 8.0,
    },
}

# Wall alignment for nodes with two markers on the same wall (N5/N6).
# Single-marker x/z alignment leaves one degree of freedom free: the robot can
# end rotated and shifted sideways while the marker still reads the target x/z.
# The lidar wall line gives heading and distance; marker bearings give the
# lateral offset. Per-marker tvec depth is not used: its error grows with
# distance and turns into heading and lateral errors.
# Targets are floor-mark measurements (2026-10-07): node mark position along
# the wall from the left marker center, and mark-to-wall distance. The N5
# mark is 2 cm off the logitle_route node 5 coordinate, so the route graph is
# not used here.
WALL_PAIR_TARGETS = {
    "N5": {
        "left_marker": 24,
        "right_marker": 29,
        "center_offset": 0.180,
        "wall_distance": 0.400,
    },
    "N6": {
        "left_marker": 25,
        "right_marker": 28,
        "center_offset": 0.175,
        "wall_distance": 0.410,
        # Measured center spacing; the marker map has 0.303.
        "marker_spacing": 0.299,
    },
    # A node may also use a single marker: set right_marker to None and
    # center_offset to the node position along the wall from that marker
    # center (robot's right +).
    # N3: the x/z preset alignment could not hold heading (marker plane yaw
    # scatters 2-4 deg) and each degree moved the robot about 1 cm sideways.
    # Tape (2026-10-07): floor mark 1.5 cm right of ID27, 41 cm from the wall;
    # the robot stops 1.5 cm further back than the mark on request.
    "N3": {
        "left_marker": 27,
        "right_marker": None,
        "center_offset": 0.015,
        "wall_distance": 0.425,
    },
    # N4: same reason as N3. The robot arm covers ID26 from the right, so ID23
    # was added 13.3 cm to its left (2026-10-08). Tape: floor mark 2.0 cm right
    # of ID26, 40.5 cm from the wall; the robot stops 2 cm behind it on request.
    "N4": {
        "left_marker": 23,
        "right_marker": 26,
        "center_offset": 0.153,
        "wall_distance": 0.425,
        "marker_spacing": 0.133,
    },
}


def fit_wall_line(scan, args):
    """Fit the wall in front of the robot as x = k*y + b in base_footprint (RANSAC).

    Returns (k, b) or None when there are too few points or the fit is poor.
    """
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + math.radians(args.scan_yaw)
    valid = np.isfinite(ranges) & (ranges > scan.range_min)
    x = ranges * np.cos(angles) + args.scan_x
    y = ranges * np.sin(angles)
    mask = valid & (x > args.wall_fit_min_x) & (x < args.wall_fit_max_x) & (np.abs(y) < args.wall_fit_half_width)
    x = x[mask]
    y = y[mask]
    if len(x) < args.wall_fit_min_points:
        return None
    # RANSAC over point pairs: objects in front of the wall (the robot arm,
    # legs) pull a plain least-squares line off the wall.
    inliers = None
    for i in range(len(x)):
        for j in range(i + 1, len(x)):
            if abs(y[j] - y[i]) < 0.05:
                continue
            k = (x[j] - x[i]) / (y[j] - y[i])
            candidate = np.abs(x - (k * y + x[i] - k * y[i])) < args.wall_fit_inlier_tolerance
            if inliers is None or candidate.sum() > inliers.sum():
                inliers = candidate
    if inliers is None or inliers.sum() < args.wall_fit_min_points:
        return None
    k, b = np.polyfit(y[inliers], x[inliers], 1)
    rms = float(np.sqrt(np.mean((x[inliers] - (k * y[inliers] + b)) ** 2)))
    if rms > args.wall_fit_max_rms:
        return None
    return float(k), float(b)


def parse_node_trims(text):
    """Parse "N5:0.015,N6:-0.01" into {"N5": 0.015, "N6": -0.01}; "none" is empty."""
    trims = {}
    for item in text.split(","):
        item = item.strip()
        if not item or item.lower() == "none":
            continue
        node, value = item.split(":")
        trims[node.strip()] = float(value)
    return trims


def ray_wall_along(origin_xy, direction_xy, k, b):
    """Where a horizontal camera ray hits the wall x = k*y + b.

    Returns the hit position along the wall, measured from the foot of the
    base_footprint origin, positive toward the robot's right. The wall is
    vertical, so the horizontal part of the ray hits it at the marker.
    """
    ox, oy = origin_xy
    dx, dy = direction_xy
    denominator = dx - k * dy
    if abs(denominator) < 1e-6:
        return None
    t = (b + k * oy - ox) / denominator
    if t <= 0.0:
        return None
    hit = np.array([ox + t * dx, oy + t * dy])
    foot_y = -k * b / (k * k + 1.0)
    foot = np.array([k * foot_y + b, foot_y])
    right = -np.array([k, 1.0]) / math.sqrt(k * k + 1.0)
    return float(np.dot(hit - foot, right))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def clamp_min_magnitude(value, minimum, low, high):
    """Clamp a non-zero command while keeping it above motor deadband."""
    value = clamp(value, low, high)
    if value == 0.0:
        return 0.0
    return math.copysign(max(abs(value), minimum), value)


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


class AlignImageListenerNode(Node):
    """Receive camera frames on a lightweight single-threaded executor."""

    def __init__(self):
        # Ignore launch remaps such as __node so this helper keeps its own name.
        super().__init__("align_and_correct_image_listener", use_global_arguments=False)


class AlignAndCorrectActionServer(Node):
    def __init__(self, args, image_node):
        super().__init__("align_and_correct_with_aruco_action_server")
        self.image_node = image_node
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
        self.camera_process_lock = threading.Lock()
        self.camera_process = None
        self.camera_process_owned = False
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_shape = None
        self.detection_period_sec = 1.0 / max(float(args.detection_rate_hz), 1.0)
        self.last_detection_time = 0.0
        self.markers = load_marker_map(args.marker_map)
        self.T_base_camera = camera_mount_T(
            args.camera_x,
            args.camera_y,
            args.camera_z,
            pitch_deg=args.camera_pitch,
            yaw_deg=args.camera_yaw,
            roll_deg=args.camera_roll,
        )
        # Wall pair mode: marker bearings, lidar scan and odometry during a goal.
        self.wall_marker_ids = ()
        self.latest_markers = {}
        self.latest_rays = {}
        self.odom_sub = None
        self.latest_odom = None
        self.scan_sub = None
        self.latest_scan = None
        # Pair minus single-marker lateral, per marker, within one goal.
        self.wall_single_offsets = {}
        self.last_wall_heading = None
        if args.pose_source == "camera":
            dictionary_id = ARUCO_DICTS[args.dictionary]
            self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.detector_params = make_detector_params()
        else:
            self.dictionary = None
            self.detector_params = None

        cmd_type = TwistStamped if args.cmd_vel_stamped else Twist
        self.cmd_pub = self.create_publisher(cmd_type, args.cmd_vel_topic, 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, args.initialpose_topic, 10)
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

        self.start_camera_on_goal()
        self.start_pose_input(marker_id, params["wall_pair"])
        preset_text = f"; preset_node={params['preset_node']}" if params["preset_node"] else ""
        if params["wall_pair"]:
            pair = params["wall_pair"]
            preset_text += (
                f"; wall_markers={pair['left_marker']}"
                + (f"+{pair['right_marker']}" if pair["right_marker"] is not None else "")
                + "; "
                f"wall_distance={pair['wall_distance']:.3f}m; "
                f"center_offset={pair['center_offset']:.3f}m"
            )
        self.get_logger().info(
            f"Accepted align/correct goal: marker_id={marker_id}; "
            f"target_x={params['target_x']:.3f}m; target_z={params['target_z']:.3f}m; "
            f"check_yaw={params['check_yaw']}; "
            f"expected_base_yaw_deg={params['expected_base_yaw_deg']:.1f}; "
            f"yaw_tolerance_deg={params['yaw_tolerance_deg']:.1f}; "
            f"z_tolerance={params['z_tolerance']:.3f}m; "
            f"align_timeout={params['align_timeout_sec']:.1f}s; apply_correction={goal.apply_correction}"
            f"{preset_text}"
        )

        try:
            if params["wall_pair"]:
                align_result = self.run_wall_alignment(goal_handle, params)
            else:
                align_result = self.run_alignment(goal_handle, params)
            wall_measurement = align_result.pop("wall_measurement", None)
            if goal_handle.is_cancel_requested:
                result = self.make_result(False, RESULT_CANCELED, "Align/correct canceled.", **align_result)
                goal_handle.canceled()
                return result

            if not align_result["align_success"]:
                code = RESULT_MARKER_LOST if align_result["marker_lost"] else RESULT_ALIGN_FAILED
                self.get_logger().warn(
                    f"Align/correct goal marker_id={marker_id} failed in alignment: {align_result['align_message']}"
                )
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
            if wall_measurement is not None and self.args.wall_initialpose:
                correction = self.wall_pose_correction(params, wall_measurement)
            else:
                correction = self.run_pose_correction(goal_handle, marker_id, params)
            merged = dict(align_result)
            merged.update(correction)
            if not correction["correction_success"]:
                self.get_logger().warn(
                    f"Align/correct goal marker_id={marker_id} failed in pose correction: "
                    f"{correction.get('correction_message', '')}"
                )
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
            self.stop_camera_process()
            self.release_goal()

    def params_from_goal(self, goal, marker_id):
        preset = ALIGN_TARGET_PRESETS.get(int(marker_id))
        target_x = finite_or_default(goal.target_x, self.args.target_x)
        target_z = positive_or_default(goal.target_z, self.args.target_z)
        check_yaw = bool(goal.check_yaw)
        expected_base_yaw_deg = finite_or_default(goal.expected_base_yaw_deg, ACTION_DEFAULT_EXPECTED_BASE_YAW_DEG)
        yaw_tolerance_deg = positive_or_default(goal.yaw_tolerance_deg, self.args.yaw_tolerance_deg)
        z_tolerance = positive_or_default(goal.z_tolerance, self.args.z_tolerance)
        wall_pair = None

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
            if self.args.wall_align and target_x_omitted and target_z_omitted:
                wall_pair = self.wall_pair_for_node(preset["node"])
            if bool(goal.check_yaw) and preset["check_yaw"] is False:
                check_yaw = False
            if (
                preset["expected_base_yaw_deg"] is not None
                and is_close(goal.expected_base_yaw_deg, ACTION_DEFAULT_EXPECTED_BASE_YAW_DEG)
            ):
                expected_base_yaw_deg = float(preset["expected_base_yaw_deg"])
            if (
                preset.get("yaw_tolerance_deg") is not None
                and is_close(goal.yaw_tolerance_deg, ACTION_DEFAULT_YAW_TOLERANCE_DEG)
            ):
                yaw_tolerance_deg = float(preset["yaw_tolerance_deg"])
            if (
                preset.get("z_tolerance") is not None
                and is_close(goal.z_tolerance, ACTION_DEFAULT_Z_TOLERANCE)
            ):
                z_tolerance = float(preset["z_tolerance"])

        return {
            "timeout_sec": positive_or_default(goal.timeout_sec, self.args.timeout_sec),
            "align_timeout_sec": positive_or_default(goal.align_timeout_sec, self.args.align_timeout_sec),
            "correct_timeout_sec": positive_or_default(goal.correct_timeout_sec, self.args.correct_timeout_sec),
            "target_x": target_x,
            "target_z": target_z,
            "x_tolerance": positive_or_default(goal.x_tolerance, self.args.x_tolerance),
            "z_tolerance": z_tolerance,
            "z_min_stop": positive_or_default(goal.z_min_stop, self.args.z_min_stop),
            "stable_sec": positive_or_default(goal.stable_sec, self.args.stable_sec),
            "max_linear": positive_or_default(goal.max_linear, self.args.max_linear),
            "max_angular": positive_or_default(goal.max_angular, self.args.max_angular),
            "min_angular": positive_or_default(goal.min_angular, self.args.min_angular),
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
            "marker_id": int(marker_id),
            "wall_pair": wall_pair,
        }

    def wall_pair_for_node(self, node):
        config = WALL_PAIR_TARGETS.get(node)
        if config is None:
            return None
        marker_ids = [m for m in (config["left_marker"], config["right_marker"]) if m is not None]
        missing = [m for m in marker_ids if m not in self.markers]
        if missing:
            self.get_logger().warn(
                f"Wall alignment for {node} disabled: marker map lacks "
                + ", ".join(f"ID{m}" for m in missing)
            )
            return None
        map_spacing = None
        if config["right_marker"] is not None:
            left = self.markers[config["left_marker"]]
            right = self.markers[config["right_marker"]]
            map_spacing = float(np.linalg.norm(np.asarray(right.position[:2]) - np.asarray(left.position[:2])))
        lateral_trim = parse_node_trims(self.args.pair_lateral_trims).get(node, 0.0)
        heading_trim = math.radians(parse_node_trims(self.args.pair_heading_trims).get(node, 0.0))
        distance_trim = parse_node_trims(self.args.pair_distance_trims).get(node, 0.0)
        target = dict({"marker_spacing": map_spacing}, **config)
        # Per-robot stop distance offset (m, + = further from the wall).
        target["wall_distance"] = config["wall_distance"] + distance_trim
        return dict(
            target,
            node=node,
            lateral_trim=lateral_trim,
            heading_trim=heading_trim,
        )

    def validate_params(self, params):
        for key, value in params.items():
            if key in ("check_wall_yaw", "check_yaw", "preset_node", "wall_pair"):
                continue
            if not math.isfinite(float(value)):
                return False
        return (
            params["z_min_stop"] < params["target_z"]
            and params["x_tolerance"] > 0.0
            and params["z_tolerance"] > 0.0
            and params["max_linear"] > 0.0
            and params["max_angular"] > 0.0
            and params["min_angular"] > 0.0
            and params["min_angular"] <= params["max_angular"]
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
        # Send the stop burst only when switching from motion to stop; repeating
        # it every loop floods /cmd_vel and loads the OpenCR node.
        stopped = False

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
                if not stopped:
                    self.stop_robot()
                    stopped = True
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

            # A marker far off to the side at the first sighting means the wrong
            # marker ID or a robot far from the node; driving toward it can hit
            # whatever stands between (robot1 hit the N3 conveyor on 2026-10-07).
            if (
                not last_marker_seen
                and self.args.max_start_x_error > 0.0
                and abs(x_error) > self.args.max_start_x_error
            ):
                self.stop_robot()
                return self.align_result(
                    False,
                    False,
                    x,
                    z,
                    x_error,
                    z_error,
                    final_wall_yaw_error,
                    f"Marker is {x_error * 100.0:+.1f}cm off center at start "
                    f"(limit {self.args.max_start_x_error * 100.0:.0f}cm); check the marker ID and start position.",
                )
            last_marker_seen = True

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
                if not stopped:
                    self.stop_robot()
                    stopped = True
                if aligned_since is None:
                    aligned_since = now
            else:
                aligned_since = None
                self.publish_cmd(linear_x, angular_z)
                stopped = False

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

        # Do not combine heading and lateral image errors into one command.
        # Turning toward the wall first prevents the two corrections from
        # fighting each other and keeps the marker in view while aligning.
        if wall_yaw_error != 0.0:
            angular_z = clamp_min_magnitude(
                -params["kyaw"] * wall_yaw_error,
                params["min_angular"],
                -params["max_angular"],
                params["max_angular"],
            )
            return 0.0, angular_z, "aligning_wall_yaw", x_error, z_error, wall_yaw_error

        angular_z = clamp_min_magnitude(
            -params["kx"] * x_error,
            params["min_angular"],
            -params["max_angular"],
            params["max_angular"],
        )
        linear_x = clamp(params["kz"] * z_error, -params["max_linear"], params["max_linear"])
        return linear_x, angular_z, "aligning_position", x_error, z_error, wall_yaw_error

    def run_wall_alignment(self, goal_handle, params):
        """Align N5/N6 to the wall with lidar heading/distance and marker bearings.

        Each step stops, measures (lidar wall line for heading and distance,
        marker bearings for the lateral offset), then makes one bounded move:
        ALIGN_YAW, CENTER (turn, short drive, turn back) or APPROACH.

        Decisions use the median of the measurements taken since the last
        move. Near the target (within pair_near_factor times the tolerances)
        the robot measures again instead of moving on one reading, since a
        single reading scatters about as much as the tolerance (worse when
        Nav2 and zenoh load the CPU). The median of pair_stable_count
        measurements in tolerance finishes the alignment.
        """
        pair = params["wall_pair"]
        args = self.args
        started = time.time()
        self.wall_single_offsets = {}
        self.last_wall_heading = None
        wall_failed_since = None
        centering = 0
        samples = []
        stopped = False
        yaw_tolerance = math.radians(args.pair_yaw_tolerance_deg)
        max_turn = math.radians(args.pair_coarse_max_turn_deg)
        prealigned = False
        final = {"x": 0.0, "z": 0.0, "lateral": 0.0, "distance": 0.0, "heading": 0.0}

        def result(success, marker_lost, message):
            self.stop_robot()
            return self.align_result(
                success,
                marker_lost,
                final["x"],
                final["z"],
                final["lateral"],
                final["distance"],
                final["heading"],
                message,
            )

        def feedback(state, visible=True, linear_x=0.0, angular_z=0.0):
            self.publish_feedback(
                goal_handle, state, visible, final["x"], final["z"],
                final["lateral"], final["distance"], final["heading"], linear_x, angular_z, 0.0,
            )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return result(False, False, "Canceled.")
            if time.time() - started > args.pair_timeout_sec:
                return result(False, False, "Wall pair alignment timed out.")

            if not stopped:
                self.stop_robot()
                stopped = True
            measured, error = self.measure_wall(goal_handle, pair)
            if measured is None:
                if error == "no_marker":
                    feedback("waiting_for_marker", visible=False)
                    # The camera takes several seconds to start on a goal;
                    # square up to the wall with the lidar meanwhile.
                    heading = self.last_wall_heading
                    if not prealigned and heading is not None and abs(heading) > yaw_tolerance:
                        prealigned = True
                        self.get_logger().info(
                            f"Wall prealign: heading={math.degrees(heading):+.1f}deg while waiting for markers"
                        )
                        ok, message = self.odom_rotate(goal_handle, clamp(-heading, -max_turn, max_turn))
                        if not ok:
                            return result(False, False, message)
                    continue
                if error == "canceled":
                    continue
                # Scans can drop out briefly (Nav2 collision_monitor reported
                # an invalid source right before a robot3 failure), so retry
                # for a while before giving up.
                if wall_failed_since is None:
                    wall_failed_since = time.time()
                if time.time() - wall_failed_since < args.pair_wall_retry_sec:
                    self.get_logger().warn(f"Wall measure failed, retrying: {error}")
                    feedback("waiting_for_wall")
                    continue
                return result(False, False, error)
            wall_failed_since = None

            marker = self.fresh_marker(params["marker_id"])
            if marker is not None:
                final["x"] = marker["x"]
                final["z"] = marker["z"]
            if measured["distance"] - args.camera_x < params["z_min_stop"]:
                return result(False, False, "Wall is too close; stopped before alignment.")

            samples.append(measured)
            heading = float(np.median([s["heading"] for s in samples]))
            lateral = float(np.median([s["lateral"] for s in samples]))
            distance_error = float(np.median([s["distance"] for s in samples])) - pair["wall_distance"]
            final["lateral"] = lateral
            final["distance"] = distance_error
            final["heading"] = heading

            in_tolerance = (
                abs(heading) <= yaw_tolerance
                and abs(lateral) <= args.pair_lateral_tolerance
                and abs(distance_error) <= args.pair_distance_tolerance
            )
            near = (
                abs(heading) <= yaw_tolerance * args.pair_near_factor
                and abs(lateral) <= args.pair_lateral_tolerance * args.pair_near_factor
                and abs(distance_error) <= args.pair_distance_tolerance * args.pair_near_factor
            )
            # Finishing takes pair_stable_count readings; moving again only
            # needs two, so one stray reading does not trigger a move.
            needed = args.pair_stable_count if in_tolerance else min(2, args.pair_stable_count)
            if (in_tolerance or near) and len(samples) < needed:
                feedback("stable_check")
                time.sleep(args.stable_sec)
                continue
            if in_tolerance:
                self.get_logger().info(
                    f"Wall pair aligned: lateral={lateral * 100.0:+.1f}cm "
                    f"distance_error={distance_error * 100.0:+.1f}cm "
                    f"heading={math.degrees(heading):+.1f}deg centering={centering} "
                    f"markers={measured['markers']} (median of {len(samples)})"
                )
                median = dict(
                    measured,
                    heading=heading,
                    lateral=lateral,
                    distance=distance_error + pair["wall_distance"],
                )
                return dict(result(True, False, "Wall pair alignment succeeded."), wall_measurement=median)
            if len(samples) > 1:
                self.get_logger().info(
                    f"Wall median of {len(samples)}: lateral={lateral * 100.0:+.1f}cm "
                    f"distance_error={distance_error * 100.0:+.1f}cm heading={math.degrees(heading):+.1f}deg"
                )
            samples = []

            lateral_off = abs(lateral) > args.pair_lateral_tolerance
            if abs(heading) > yaw_tolerance and not lateral_off:
                turn = clamp(-heading, -max_turn, max_turn)
                feedback("align_yaw", angular_z=turn)
                ok, message = self.odom_rotate(goal_handle, turn)
            elif lateral_off:
                # The center step's first turn also takes out the heading.
                if centering >= args.pair_max_centering:
                    return result(
                        False,
                        False,
                        f"Lateral offset {lateral * 100.0:.1f} cm remains after {centering} centering moves.",
                    )
                centering += 1
                feedback("center")
                ok, message = self.center_step(
                    goal_handle, lateral, distance_error, heading,
                    distance_error + pair["wall_distance"], params["z_min_stop"]
                )
            else:
                drive = clamp(distance_error, -args.pair_max_approach, args.pair_max_approach)
                feedback("approach_distance", linear_x=drive)
                ok, message = self.odom_drive(goal_handle, drive)

            if not ok:
                return result(False, False, message)

        return result(False, False, "ROS shutdown during alignment.")

    def center_step(self, goal_handle, lateral, distance_error, heading, distance, z_min_stop):
        """Shift sideways: turn toward the center line, drive, turn back.

        When the robot is too far from the wall, the drive also covers the
        remaining distance: the path aims at the target point, or as close to
        it as the turn limit allows. Otherwise it backs out toward the center
        line. Large lateral offsets and long approaches use wider limits so Nav2
        arrival offsets of several centimeters need only a few steps.
        """
        args = self.args
        # The drive has to cover a distance gap longer than one fine step
        # anyway, so it may as well take the wider limits and aim at the target.
        if abs(lateral) > args.pair_coarse_lateral or distance_error > args.pair_max_drive:
            max_turn = math.radians(args.pair_coarse_max_turn_deg)
            max_drive = args.pair_coarse_max_drive
        else:
            max_turn = math.radians(args.pair_max_turn_deg)
            max_drive = args.pair_max_drive

        forward = None
        if distance_error > 0.0:
            needed_turn = math.atan2(abs(lateral), distance_error)
            turn = min(max_turn, needed_turn)
            if needed_turn <= max_turn:
                drive = math.hypot(lateral, distance_error)
            else:
                drive = abs(lateral) / math.sin(turn)
            # Keep the camera at least z_min_stop (plus margin) from the wall.
            room = distance - args.camera_x - z_min_stop - args.pair_wall_margin
            drive = min(drive, max_drive, max(0.0, room) / math.cos(turn))
            # Passing the target distance pushes the markers toward the image
            # edges and needs a step back anyway; back out instead.
            if drive * math.cos(turn) - distance_error <= args.pair_max_overshoot:
                forward = (turn, drive)
            elif distance_error > args.pair_max_drive:
                # Still far behind: backing out only adds distance to cover
                # later. Drive up to the target distance and leave the rest
                # of the lateral for the next step.
                limit = (distance_error + args.pair_max_overshoot) / math.cos(turn)
                forward = (turn, min(drive, limit))
        return_drive = 0.0
        if forward is not None:
            direction = 1.0
            turn, drive = forward
        else:
            direction = -1.0
            # Prefer the full drive at a small angle over a short drive at a
            # wide one: odometry drive errors then barely move the lateral.
            turn = min(max_turn, max(math.radians(1.0), math.asin(min(1.0, abs(lateral) / max_drive))))
            drive = min(max_drive, abs(lateral) / math.sin(turn))
            # Drive straight back in after turning square, covering the
            # distance error too, so one step fixes both without another
            # measurement in between.
            return_drive = drive * math.cos(turn) + distance_error
        if drive < args.maneuver_distance_tolerance:
            return False, "Wall center step has no room to move; check the start position."

        # lateral > 0: robot is right of center, so it must move left.
        path_heading = math.copysign(turn, lateral) * direction
        self.get_logger().info(
            f"Wall center step: lateral={lateral * 100.0:+.1f}cm distance_error={distance_error * 100.0:+.1f}cm "
            f"turn={math.degrees(path_heading - heading):+.1f}deg drive={direction * drive * 100.0:+.1f}cm "
            f"return={return_drive * 100.0:+.1f}cm (expected lateral -{drive * math.sin(turn) * 100.0:.1f}cm)"
        )
        moves = [
            ("turn", path_heading - heading),
            ("drive", direction * drive),
            ("turn", -path_heading),
        ]
        if abs(return_drive) >= args.maneuver_distance_tolerance:
            moves.append(("drive", return_drive))
        for kind, amount in moves:
            if kind == "turn":
                ok, message = self.odom_rotate(goal_handle, amount)
            else:
                ok, message = self.odom_drive(goal_handle, amount)
            if not ok:
                return False, message
        return True, ""

    def odom_rotate(self, goal_handle, angle):
        args = self.args
        start = self.get_odom()
        if start is None:
            return False, f"No odometry on {args.odom_topic} for the wall maneuver."
        target = start["yaw"] + angle
        tolerance = math.radians(args.maneuver_yaw_tolerance_deg)
        started = time.time()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False, "Canceled."
            odom = self.get_odom()
            if odom is None or time.time() - odom["received_at"] > args.odom_timeout:
                return False, "Odometry stopped during the wall maneuver."
            error = wrap_angle(target - odom["yaw"])
            # Odometry lags the motion; stop on the error expected after the lag.
            if abs(error) <= tolerance or abs(error - args.maneuver_lead_sec * odom["angular"]) <= tolerance:
                break
            if time.time() - started > args.maneuver_timeout_sec:
                return False, "Wall maneuver turn timed out."
            command = clamp(
                args.maneuver_kyaw * error - args.maneuver_kd_yaw * odom["angular"],
                -args.maneuver_angular,
                args.maneuver_angular,
            )
            if abs(command) < args.maneuver_min_angular:
                command = math.copysign(args.maneuver_min_angular, error)
            self.publish_cmd(0.0, command)
            time.sleep(args.maneuver_period)
        self.stop_robot()
        return True, ""

    def odom_drive(self, goal_handle, distance):
        args = self.args
        start = self.get_odom()
        if start is None:
            return False, f"No odometry on {args.odom_topic} for the wall maneuver."
        heading = (math.cos(start["yaw"]), math.sin(start["yaw"]))
        started = time.time()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False, "Canceled."
            odom = self.get_odom()
            if odom is None or time.time() - odom["received_at"] > args.odom_timeout:
                return False, "Odometry stopped during the wall maneuver."
            traveled = (odom["x"] - start["x"]) * heading[0] + (odom["y"] - start["y"]) * heading[1]
            error = distance - traveled
            tolerance = args.maneuver_distance_tolerance
            # Odometry lags the motion; stop on the error expected after the lag.
            if abs(error) <= tolerance or abs(error - args.maneuver_lead_sec * odom["linear"]) <= tolerance:
                break
            if time.time() - started > args.maneuver_timeout_sec:
                return False, "Wall maneuver drive timed out."
            linear_x = clamp(
                args.maneuver_kdist * error - args.maneuver_kd_dist * odom["linear"],
                -args.maneuver_linear,
                args.maneuver_linear,
            )
            if abs(linear_x) < args.maneuver_min_linear:
                linear_x = math.copysign(args.maneuver_min_linear, error)
            # Hold the start heading while driving straight.
            angular_z = clamp(
                args.maneuver_kyaw * wrap_angle(start["yaw"] - odom["yaw"]),
                -args.maneuver_min_angular,
                args.maneuver_min_angular,
            )
            self.publish_cmd(linear_x, angular_z)
            time.sleep(args.maneuver_period)
        self.stop_robot()
        return True, ""

    def measure_wall(self, goal_handle, pair):
        """Measure at standstill: lidar wall line plus marker bearings.

        Returns (measurement, None) or (None, error). Heading > 0 means the
        robot is turned left (CCW) from facing the wall; lateral > 0 means it
        is right of the node center; distance is base_footprint to the wall.
        """
        args = self.args
        time.sleep(args.pair_settle_sec)
        with self.pose_lock:
            self.latest_rays = {}
        fits = []
        rejected_scans = 0
        rays = {m: [] for m in (pair["left_marker"], pair["right_marker"]) if m is not None}
        seen_scan = None
        seen_ray = {}
        deadline = time.time() + args.pair_measure_sec
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return None, "canceled"
            with self.pose_lock:
                scan = self.latest_scan
                latest_rays = dict(self.latest_rays)
            if scan is not None and scan is not seen_scan:
                seen_scan = scan
                fit = fit_wall_line(scan, args)
                if fit is None:
                    rejected_scans += 1
                else:
                    fits.append(fit)
            for marker_id, ray in latest_rays.items():
                if marker_id in rays and seen_ray.get(marker_id) != ray["received_at"]:
                    seen_ray[marker_id] = ray["received_at"]
                    rays[marker_id].append(ray["direction"])
            time.sleep(0.02)

        if len(fits) < args.pair_min_scans:
            return None, (
                f"Lidar wall fit failed ({len(fits)} good scans, {rejected_scans} rejected); "
                "check the wall in front of the robot."
            )
        k = float(np.median([fit[0] for fit in fits]))
        b = float(np.median([fit[1] for fit in fits]))
        heading = math.atan(k)
        distance = b * math.cos(heading)

        camera_xy = (float(self.T_base_camera[0, 3]), float(self.T_base_camera[1, 3]))
        hits = {}
        for marker_id, directions in rays.items():
            if len(directions) < 3:
                continue
            along = ray_wall_along(camera_xy, np.mean(np.asarray(directions), axis=0), k, b)
            if along is not None:
                hits[marker_id] = along
        self.last_wall_heading = heading - pair["heading_trim"]
        if not hits:
            return None, "no_marker"
        left_id = pair["left_marker"]
        right_id = pair["right_marker"]
        single_along = {}
        if left_id in hits:
            single_along[left_id] = hits[left_id] + pair["center_offset"]
        if right_id is not None and right_id in hits:
            single_along[right_id] = hits[right_id] - (pair["marker_spacing"] - pair["center_offset"])
        if right_id is None:
            center_along = single_along[left_id]
            source = f"ID{left_id}"
        elif left_id in hits and right_id in hits:
            # Interpolate between the two hits by the map ratio: this cancels a
            # bearing scale error (the 320x240 calibration reads the pair about
            # 6% wider than it is near the image edges).
            fraction = pair["center_offset"] / pair["marker_spacing"]
            center_along = hits[left_id] + (hits[right_id] - hits[left_id]) * fraction
            source = "pair"
            # Remember how far each single-marker estimate is off, so a marker
            # that drops out later (arm, cable) does not make the lateral jump.
            for marker_id, along in single_along.items():
                self.wall_single_offsets[marker_id] = center_along - along
        else:
            marker_id = next(iter(single_along))
            offset = self.wall_single_offsets.get(marker_id)
            center_along = single_along[marker_id] + (offset or 0.0)
            source = f"ID{marker_id} only" + (f" (pair offset {offset * 100.0:+.1f}cm)" if offset is not None else "")
        # Per-robot trim: the lateral this robot reads while it sits on the
        # floor mark (camera calibration error that depends on the node).
        lateral = -center_along - pair["lateral_trim"]
        # Per-robot, per-node lidar heading read while the robot is square to
        # the wall (the wall section in front of the lidar is not perfectly
        # straight). The wall line itself stays as fitted for the lateral.
        heading -= pair["heading_trim"]
        trim_note = f" trim={pair['lateral_trim'] * 100.0:+.1f}cm" if pair["lateral_trim"] else ""
        if pair["heading_trim"]:
            trim_note += f" heading_trim={math.degrees(pair['heading_trim']):+.1f}deg"
        self.get_logger().info(
            f"Wall measure: lateral={lateral * 100.0:+.1f}cm distance={distance:.3f}m "
            f"heading={math.degrees(heading):+.1f}deg markers={source} scans={len(fits)}{trim_note}"
        )
        return {
            "heading": heading,
            "distance": distance,
            "lateral": lateral,
            "markers": source,
        }, None

    def fresh_marker(self, marker_id):
        with self.pose_lock:
            marker = self.latest_markers.get(marker_id)
        if marker is None or time.time() - marker["received_at"] > self.args.pose_timeout:
            return None
        return marker

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.pose_lock:
            self.latest_odom = {
                "received_at": time.time(),
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "yaw": yaw,
                "linear": float(msg.twist.twist.linear.x),
                "angular": float(msg.twist.twist.angular.z),
            }

    def scan_cb(self, msg):
        with self.pose_lock:
            self.latest_scan = msg

    def get_odom(self):
        with self.pose_lock:
            return None if self.latest_odom is None else dict(self.latest_odom)

    def wall_pose_correction(self, params, measured):
        """Publish /initialpose from the wall alignment measurement.

        The marker map gives the wall line and node center; lidar heading and
        distance plus the bearing-based lateral offset place the robot on it.
        This replaces the tvec-based pose corrector for wall pair nodes, whose
        heading and lateral estimates drift with distance.
        """
        pair = params["wall_pair"]
        left = np.asarray(self.markers[pair["left_marker"]].position[:2], dtype=float)
        marker_normal = np.asarray(self.markers[pair["left_marker"]].normal[:2], dtype=float)
        if pair["right_marker"] is None:
            # One marker: the wall runs perpendicular to its normal; the robot
            # faces -normal, so its right is (-n_y, n_x).
            normal = marker_normal / np.linalg.norm(marker_normal)
            along = np.array([-normal[1], normal[0]])
        else:
            right = np.asarray(self.markers[pair["right_marker"]].position[:2], dtype=float)
            along = (right - left) / np.linalg.norm(right - left)
            normal = np.array([along[1], -along[0]])
            if np.dot(marker_normal, normal) < 0.0:
                normal = -normal
        foot = left + pair["center_offset"] * along
        xy = foot + measured["lateral"] * along + measured["distance"] * normal
        yaw = wrap_angle(math.atan2(-normal[1], -normal[0]) + measured["heading"])
        yaw_deg = math.degrees(yaw)
        pose = {
            "map_base_x": float(xy[0]),
            "map_base_y": float(xy[1]),
            "map_base_yaw_deg": yaw_deg,
            "map_base_xy_std": 0.0,
            "map_base_yaw_std_deg": 0.0,
        }

        yaw_error = math.degrees(wrap_angle(yaw - math.radians(params["expected_base_yaw_deg"])))
        if params["check_yaw"] and abs(yaw_error) > params["yaw_tolerance_deg"]:
            return dict(
                pose,
                correction_success=False,
                correction_result_code=CorrectPoseWithAruco.Result.RESULT_UNSTABLE_SAMPLES,
                correction_message=(
                    f"Wall pose yaw {yaw_deg:.1f}deg is {yaw_error:+.1f}deg from expected "
                    f"{params['expected_base_yaw_deg']:.1f}deg."
                ),
            )

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.map_frame
        msg.pose.pose.position.x = float(xy[0])
        msg.pose.pose.position.y = float(xy[1])
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        xy_var = float(self.args.initialpose_xy_std) ** 2
        msg.pose.covariance[0] = xy_var
        msg.pose.covariance[7] = xy_var
        msg.pose.covariance[35] = math.radians(float(self.args.initialpose_yaw_std_deg)) ** 2
        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f"Published wall pose to {self.args.initialpose_topic}: "
            f"x={xy[0]:.3f} y={xy[1]:.3f} yaw={yaw_deg:.1f}deg"
        )
        return dict(
            pose,
            correction_success=True,
            correction_result_code=CorrectPoseWithAruco.Result.RESULT_SUCCESS,
            correction_message=(
                "Published /initialpose from lidar wall line and marker bearings "
                f"(markers={measured['markers']})."
            ),
        )

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

    def start_pose_input(self, marker_id, wall_pair=None):
        with self.pose_lock:
            self.latest_pose = None
            self.latest_markers = {}
            self.latest_rays = {}
            self.latest_odom = None
            self.latest_scan = None
        self.last_detection_time = 0.0
        if wall_pair and self.odom_sub is None:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.args.odom_topic,
                self.odom_cb,
                10,
                callback_group=self.callback_group,
            )
        if wall_pair and self.scan_sub is None:
            self.scan_sub = self.create_subscription(
                LaserScan,
                self.args.scan_topic,
                self.scan_cb,
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )

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
            self.wall_marker_ids = (
                tuple(m for m in (wall_pair["left_marker"], wall_pair["right_marker"]) if m is not None)
                if wall_pair
                else ()
            )
            if self.camera_info_sub is None:
                self.camera_info_sub = self.image_node.create_subscription(
                    CameraInfo,
                    self.args.camera_info_topic,
                    self.camera_info_cb,
                    10,
                )
            if self.image_sub is None:
                # Receive serialized frames and deserialize only the frames that
                # pass the detection rate limit; the camera publishes ~30 fps.
                self.image_sub = self.image_node.create_subscription(
                    Image,
                    self.args.image_topic,
                    self.image_cb,
                    QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=1,
                    ),
                    raw=True,
                )
        self.get_logger().info(
            f"Started on-demand ArUco alignment detection for marker_id={marker_id} "
            f"from {self.args.image_topic}"
        )

    def start_camera_on_goal(self):
        if self.args.pose_source != "camera" or not self.args.camera_auto_start:
            return

        with self.camera_process_lock:
            if self.camera_process is not None:
                if self.camera_process.poll() is None:
                    return
                self.camera_process = None
                self.camera_process_owned = False

            # Reuse a camera that was started separately, for example with
            # `use_camera:=true` or a standalone camera launch.
            camera_node_running = any(
                name == "camera" for name, _namespace in self.get_node_names_and_namespaces()
            )
            if (
                self.count_publishers(self.args.image_topic) > 0
                or self.count_publishers(self.args.camera_info_topic) > 0
                or camera_node_running
            ):
                self.get_logger().info("Using an already running camera process")
                return

            ros2_executable = shutil.which("ros2")
            if ros2_executable is None:
                self.get_logger().error("Cannot start camera: ros2 executable was not found")
                return

            command = [
                ros2_executable,
                "launch",
                self.args.camera_launch_package,
                self.args.camera_launch_file,
            ]
            if self.args.camera_info_url:
                command.append(f"camera_info_url:={self.args.camera_info_url}")

            try:
                self.camera_process = subprocess.Popen(
                    command,
                    start_new_session=True,
                )
                self.camera_process_owned = True
                self.get_logger().info(
                    "Started camera on Action goal: " + " ".join(command)
                )
            except OSError as exc:
                self.camera_process = None
                self.camera_process_owned = False
                self.get_logger().error(f"Failed to start camera on Action goal: {exc}")

    def stop_camera_process(self):
        with self.camera_process_lock:
            process = self.camera_process
            owned = self.camera_process_owned
            self.camera_process = None
            self.camera_process_owned = False

        if process is None or not owned or process.poll() is not None:
            return

        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=self.args.camera_shutdown_timeout_sec)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGKILL)
        self.get_logger().info("Stopped camera started for the completed Action goal")

    def stop_pose_input(self):
        if self.pose_sub is not None:
            self.destroy_subscription(self.pose_sub)
            self.pose_sub = None
        if self.odom_sub is not None:
            self.destroy_subscription(self.odom_sub)
            self.odom_sub = None
        if self.scan_sub is not None:
            self.destroy_subscription(self.scan_sub)
            self.scan_sub = None
        with self.pose_input_lock:
            marker_id = self.active_marker_id
            self.active_marker_id = None
            self.wall_marker_ids = ()
            image_sub = self.image_sub
            camera_info_sub = self.camera_info_sub
            self.image_sub = None
            self.camera_info_sub = None
        if image_sub is not None:
            self.image_node.destroy_subscription(image_sub)
        if camera_info_sub is not None:
            self.image_node.destroy_subscription(camera_info_sub)
        if marker_id is not None and self.args.pose_source == "camera":
            self.get_logger().info(f"Stopped on-demand ArUco alignment detection for marker_id={marker_id}")
        with self.pose_lock:
            self.latest_pose = None
            self.latest_markers = {}
            self.latest_rays = {}
            self.latest_odom = None
            self.latest_scan = None

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

    def image_cb(self, raw_msg):
        with self.pose_input_lock:
            marker_id = self.active_marker_id
            wall_marker_ids = self.wall_marker_ids
        if marker_id is None:
            return

        now = time.monotonic()
        if now - self.last_detection_time < self.detection_period_sec:
            return
        self.last_detection_time = now
        msg = deserialize_message(raw_msg, Image)

        if self.camera_matrix is None:
            if self.args.approx_camera_info:
                self.set_approx_camera_info(msg.width, msg.height)
            else:
                return
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame,
            self.dictionary,
            parameters=self.detector_params,
        )
        if ids is None:
            return

        received_at = time.time()
        for idx, detected_id in enumerate(ids.flatten().tolist()):
            if detected_id != marker_id and detected_id not in wall_marker_ids:
                continue
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[idx]],
                self.marker_size_for(detected_id),
                self.camera_matrix,
                self.dist_coeffs,
            )
            tvec = tvecs[0][0]
            if detected_id == marker_id:
                self.store_latest_pose(self.make_pose_msg(msg.header.stamp, rvecs[0][0], tvec))
            if detected_id in wall_marker_ids:
                # Only the bearing is used; tvec depth is too coarse here.
                direction = self.T_base_camera[:3, :3] @ (np.asarray(tvec, dtype=float) / np.linalg.norm(tvec))
                horizontal = direction[:2] / np.linalg.norm(direction[:2])
                with self.pose_lock:
                    self.latest_markers[detected_id] = {
                        "received_at": received_at,
                        "x": float(tvec[0]),
                        "z": float(tvec[2]),
                    }
                    self.latest_rays[detected_id] = {
                        "received_at": received_at,
                        "direction": (float(horizontal[0]), float(horizontal[1])),
                    }

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
        self.stop_camera_process()
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
            "camera: start/reuse the camera and subscribe to image topics only while an Action goal is active. "
            "topic: subscribe to an external pose publisher."
        ),
    )
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--camera-auto-start", type=bool_arg, default=True)
    parser.add_argument("--camera-launch-package", default="turtlebot3_bringup")
    parser.add_argument("--camera-launch-file", default="camera.launch.py")
    parser.add_argument("--camera-info-url", default="")
    parser.add_argument("--camera-shutdown-timeout-sec", type=float, default=3.0)
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
    parser.add_argument("--approx-camera-info", type=bool_arg, default=False)
    parser.add_argument("--approx-horizontal-fov-deg", type=float, default=62.2)

    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--cmd-vel-stamped", type=bool_arg, default=True)
    parser.add_argument("--cmd-frame-id", default="base_footprint")

    parser.add_argument("--target-x", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.0)
    parser.add_argument("--x-tolerance", type=float, default=0.005)
    parser.add_argument("--z-tolerance", type=float, default=0.007)
    parser.add_argument("--z-min-stop", type=float, default=0.25)
    parser.add_argument("--stable-sec", type=float, default=0.4)
    parser.add_argument("--max-linear", type=float, default=0.030)
    parser.add_argument("--max-angular", type=float, default=0.04)
    parser.add_argument("--min-angular", type=float, default=0.020)
    parser.add_argument("--kx", type=float, default=0.8)
    parser.add_argument("--kz", type=float, default=1.2)
    parser.add_argument("--wall-yaw-tolerance-deg", type=float, default=2.0)
    parser.add_argument("--kyaw", type=float, default=0.6)

    # Camera mount, same convention and defaults as the pose corrector.
    parser.add_argument("--camera-x", type=float, default=0.045)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.115)
    parser.add_argument("--camera-pitch", type=float, default=-5.0)
    parser.add_argument("--camera-yaw", type=float, default=0.0)
    parser.add_argument("--camera-roll", type=float, default=0.0)

    # Wall alignment (WALL_PAIR_TARGETS nodes, preset targets only).
    parser.add_argument("--wall-align", type=bool_arg, default=True)
    # Single lidar scans scatter about +-0.4deg in heading on robot1.
    parser.add_argument("--pair-yaw-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--pair-lateral-tolerance", type=float, default=0.003)
    parser.add_argument("--pair-distance-tolerance", type=float, default=0.005)
    # Bounds for one move: turn angle and drive distance.
    parser.add_argument("--pair-max-turn-deg", type=float, default=12.0)
    parser.add_argument("--pair-max-drive", type=float, default=0.03)
    # Lateral offsets above pair-coarse-lateral, or approaches longer than
    # pair-max-drive, use the wider coarse limits.
    parser.add_argument("--pair-coarse-lateral", type=float, default=0.01)
    parser.add_argument("--pair-coarse-max-turn-deg", type=float, default=20.0)
    parser.add_argument("--pair-coarse-max-drive", type=float, default=0.10)
    # A straight approach (lateral already in tolerance) may cover this much at once.
    parser.add_argument("--pair-max-approach", type=float, default=0.15)
    parser.add_argument("--pair-wall-margin", type=float, default=0.02)
    # A forward center step may pass the target distance by at most this much.
    parser.add_argument("--pair-max-overshoot", type=float, default=0.003)
    parser.add_argument("--pair-max-centering", type=int, default=10)
    # Consecutive in-tolerance measurements needed to finish.
    parser.add_argument("--pair-stable-count", type=int, default=3)
    # Within this many times the tolerances, measure again (up to
    # pair-stable-count readings) before moving, and decide on the median.
    parser.add_argument("--pair-near-factor", type=float, default=2.0)
    parser.add_argument("--pair-timeout-sec", type=float, default=90.0)
    parser.add_argument("--pair-settle-sec", type=float, default=0.3)
    parser.add_argument("--pair-measure-sec", type=float, default=2.0)
    # Lateral read on the floor mark, per node: "N5:0.015,N6:-0.01" [m, robot's right +].
    parser.add_argument("--pair-lateral-trims", default="none")
    # x/z preset alignment refuses to start when the marker is further off
    # center than this at its first sighting [m]; 0 disables the check.
    parser.add_argument("--max-start-x-error", type=float, default=0.15)
    # Lidar heading read while square to the wall, per node: "N5:-1.6" [deg].
    parser.add_argument("--pair-heading-trims", default="none")
    # Stop distance offset per node: "N5:0.005" [m, + = further from the wall].
    parser.add_argument("--pair-distance-trims", default="none")
    parser.add_argument("--pair-min-scans", type=int, default=3)
    # How long lidar wall fit failures are retried before the goal fails [s].
    parser.add_argument("--pair-wall-retry-sec", type=float, default=5.0)
    parser.add_argument("--scan-topic", default="/scan")
    # base_scan position in base_footprint (turtlebot3 burger URDF).
    parser.add_argument("--scan-x", type=float, default=-0.032)
    # Lidar mount yaw [deg, left positive]; turns the scan into base_footprint.
    parser.add_argument("--scan-yaw", type=float, default=0.0)
    parser.add_argument("--wall-fit-min-x", type=float, default=0.25)
    parser.add_argument("--wall-fit-max-x", type=float, default=0.60)
    parser.add_argument("--wall-fit-half-width", type=float, default=0.35)
    parser.add_argument("--wall-fit-inlier-tolerance", type=float, default=0.010)
    parser.add_argument("--wall-fit-min-points", type=int, default=20)
    parser.add_argument("--wall-fit-max-rms", type=float, default=0.005)
    # Wall pair nodes publish /initialpose from the wall measurement instead of
    # calling the tvec-based pose corrector.
    parser.add_argument("--wall-initialpose", type=bool_arg, default=True)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--initialpose-topic", default=scoped_topic(robot_namespace, "initialpose"))
    parser.add_argument("--initialpose-xy-std", type=float, default=0.05)
    parser.add_argument("--initialpose-yaw-std-deg", type=float, default=5.0)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--odom-timeout", type=float, default=0.5)
    parser.add_argument("--maneuver-linear", type=float, default=0.030)
    parser.add_argument("--maneuver-min-linear", type=float, default=0.012)
    parser.add_argument("--maneuver-kdist", type=float, default=1.5)
    parser.add_argument("--maneuver-distance-tolerance", type=float, default=0.001)
    parser.add_argument("--maneuver-angular", type=float, default=0.25)
    parser.add_argument("--maneuver-min-angular", type=float, default=0.05)
    parser.add_argument("--maneuver-kyaw", type=float, default=1.5)
    parser.add_argument("--maneuver-yaw-tolerance-deg", type=float, default=0.2)
    # D terms on the odometry velocity, and how far ahead the stop check looks
    # to cover odometry lag and coasting. Measured 2026-10-07: P only overshot
    # robot3 turns by 0.40 deg and drives by 1.4 mm; lead 0.1 cut that to 0.10
    # deg on robot3, but robot1 still turned 0.6 deg commands by up to 1.2 deg.
    # 0.25 keeps robot1 small turns within about 0.2 deg.
    parser.add_argument("--maneuver-kd-yaw", type=float, default=0.5)
    parser.add_argument("--maneuver-kd-dist", type=float, default=0.5)
    parser.add_argument("--maneuver-lead-sec", type=float, default=0.25)
    parser.add_argument("--maneuver-period", type=float, default=0.05)
    parser.add_argument("--maneuver-timeout-sec", type=float, default=10.0)

    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--align-timeout-sec", type=float, default=25.0)
    parser.add_argument("--correct-timeout-sec", type=float, default=10.0)
    parser.add_argument("--required-samples", type=int, default=15)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=3.0)

    parser.add_argument("--pose-timeout", type=float, default=0.3)
    parser.add_argument("--feedback-period", type=float, default=0.2)
    # Matches the 10 Hz detection rate; a faster loop only resends the same command.
    parser.add_argument("--control-period", type=float, default=0.1)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    rclpy.init()
    image_node = AlignImageListenerNode()
    node = AlignAndCorrectActionServer(args, image_node)
    image_executor = SingleThreadedExecutor()
    image_executor.add_node(image_node)
    image_thread = threading.Thread(
        target=image_executor.spin,
        name="align_image_listener",
        daemon=True,
    )
    image_thread.start()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        image_executor.shutdown()
        image_thread.join(timeout=2.0)
        image_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
