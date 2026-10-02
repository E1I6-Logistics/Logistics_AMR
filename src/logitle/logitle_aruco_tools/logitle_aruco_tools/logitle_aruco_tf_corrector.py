#!/usr/bin/env python3
"""Estimate map->odom correction from an ArUco marker observation.

This node subscribes to /aruco/id24/pose_camera, combines the observed
camera->marker pose with logitle_marker_map.yaml and a measured camera mount offset,
then computes:

    T_map_base = T_map_marker @ inv(T_camera_marker) @ inv(T_base_camera)
    T_map_odom = T_map_base @ inv(T_odom_base)

By default it runs in dry-run mode and only prints the estimated correction.
Use --publish-tf only after logitle_marker_map.yaml and camera mount values are
measured and checked.
"""

import argparse
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

try:
    # Installed as the logitle_aruco_tools ROS package.
    from logitle_aruco_tools.logitle_marker_localization import (
        camera_mount_T,
        invert,
        make_T,
        map_odom_correction,
        matrix_to_rpy,
        pose_str,
        rpy_to_matrix,
    )
    from logitle_aruco_tools.logitle_marker_map import load_marker_map
except ImportError:
    # Run flat next to logitle_marker_localization.py / logitle_marker_map.py (no colcon package).
    from logitle_marker_localization import (
        camera_mount_T,
        invert,
        make_T,
        map_odom_correction,
        matrix_to_rpy,
        pose_str,
        rpy_to_matrix,
    )
    from logitle_marker_map import load_marker_map


def default_robot_namespace():
    return {
        "1": "tb3_0",
        "2": "tb3_1",
        "3": "tb3_2",
    }.get(os.environ.get("USER", "")[-1:], "")


def scoped_topic(namespace, name):
    return f"/{namespace}/{name}" if namespace else f"/{name}"


def scoped_frame(namespace, name):
    return f"{namespace}/{name}" if namespace else name


def quaternion_to_matrix(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = float(np.linalg.norm(q))
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def matrix_to_quaternion(R):
    R = np.asarray(R, dtype=float).reshape(3, 3)
    trace = float(np.trace(R))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=float)
    n = float(np.linalg.norm(q))
    if n == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def transform_to_T(msg):
    t = msg.transform.translation
    q = msg.transform.rotation
    return make_T(
        quaternion_to_matrix(q.x, q.y, q.z, q.w),
        [t.x, t.y, t.z],
    )


def T_to_transform(T, stamp, parent_frame, child_frame):
    qx, qy, qz, qw = matrix_to_quaternion(T[:3, :3])
    out = TransformStamped()
    out.header.stamp = stamp
    out.header.frame_id = parent_frame
    out.child_frame_id = child_frame
    out.transform.translation.x = float(T[0, 3])
    out.transform.translation.y = float(T[1, 3])
    out.transform.translation.z = float(T[2, 3])
    out.transform.rotation.x = float(qx)
    out.transform.rotation.y = float(qy)
    out.transform.rotation.z = float(qz)
    out.transform.rotation.w = float(qw)
    return out


def median_angle(angles_rad):
    """Robust median of a set of angles via component-wise median of unit vectors."""
    cos_med = float(np.median(np.cos(angles_rad)))
    sin_med = float(np.median(np.sin(angles_rad)))
    return math.atan2(sin_med, cos_med)


def angle_diff(a, b):
    """Shortest signed angular difference a-b [rad]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ArucoTfCorrector(Node):
    def __init__(self, args):
        super().__init__("aruco_tf_corrector")
        self.args = args
        self.last_print_time = 0.0
        self.last_reject_log_time = 0.0
        self.last_msg_time = None
        self.window = deque(maxlen=args.median_window)
        self.reject_z_count = 0
        self.reject_tilt_count = 0
        self.reject_yaw_count = 0
        self.accept_count = 0

        markers = load_marker_map(args.marker_map)
        if args.marker_id not in markers:
            raise ValueError(
                f"{args.marker_map}: marker id {args.marker_id} not found. "
                "Add the measured id=24 entry before TF correction."
            )
        self.marker = markers[args.marker_id]
        self.T_base_camera = camera_mount_T(
            args.camera_x,
            args.camera_y,
            args.camera_z,
            pitch_deg=args.camera_pitch,
            yaw_deg=args.camera_yaw,
            roll_deg=args.camera_roll,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, args.pose_topic, self.pose_cb, 10)

        mode = "PUBLISH map->odom" if args.publish_tf else "dry-run"
        self.get_logger().info(
            f"Listening to {args.pose_topic}; mode={mode}; "
            f"marker_id={args.marker_id}; marker_map={args.marker_map}"
        )
        if args.publish_tf:
            self.get_logger().warn(
                "Publishing map->odom. Do not run another map->odom publisher "
                "such as AMCL at the same time."
            )

    def pose_cb(self, msg):
        now = time.time()
        if (
            self.window
            and self.last_msg_time is not None
            and (now - self.last_msg_time) > self.args.stale_timeout
        ):
            self.get_logger().info(
                f"marker gap {now - self.last_msg_time:.2f}s > "
                f"{self.args.stale_timeout}s, resetting median window"
            )
            self.window.clear()
        self.last_msg_time = now

        try:
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.args.odom_frame,
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.args.tf_timeout),
            )
        except TransformException as exc:
            self._throttled_log(f"waiting for {self.args.odom_frame}->{self.args.base_frame}: {exc}")
            return

        T_camera_marker = self.pose_to_T(msg)
        T_map_base_raw = (
            self.marker.T_map_marker
            @ invert(T_camera_marker)
            @ invert(self.T_base_camera)
        )

        roll, pitch, yaw = matrix_to_rpy(T_map_base_raw[:3, :3])
        z = float(T_map_base_raw[2, 3])
        tilt_limit = math.radians(self.args.outlier_tilt_deg)
        z_bad = abs(z) > self.args.outlier_z_max
        tilt_bad = abs(roll) > tilt_limit or abs(pitch) > tilt_limit
        if z_bad or tilt_bad:
            self.reject_z_count += int(z_bad)
            self.reject_tilt_count += int(tilt_bad)
            self._throttled_log(
                f"reject sample: z={z:+.3f} roll={math.degrees(roll):+.1f}deg "
                f"pitch={math.degrees(pitch):+.1f}deg (limits: "
                f"z<{self.args.outlier_z_max}, tilt<{self.args.outlier_tilt_deg}deg) | "
                f"totals rejected: z={self.reject_z_count} tilt={self.reject_tilt_count} "
                f"accepted={self.accept_count}"
            )
            return
        if self.args.expected_base_yaw_deg is not None:
            expected_yaw = math.radians(self.args.expected_base_yaw_deg)
            yaw_error_deg = abs(math.degrees(angle_diff(yaw, expected_yaw)))
            if yaw_error_deg > self.args.yaw_tolerance_deg:
                self.reject_yaw_count += 1
                self._throttled_log(
                    f"reject sample: yaw={math.degrees(yaw):+.1f}deg "
                    f"expected={self.args.expected_base_yaw_deg:+.1f}deg "
                    f"error={yaw_error_deg:.1f}deg "
                    f"(limit: {self.args.yaw_tolerance_deg}deg) | "
                    f"totals rejected: z={self.reject_z_count} "
                    f"tilt={self.reject_tilt_count} yaw={self.reject_yaw_count} "
                    f"accepted={self.accept_count}"
                )
                return

        # Force a 2D pose: z, roll, pitch are pinned to 0 for a floor-bound robot.
        x = float(T_map_base_raw[0, 3])
        y = float(T_map_base_raw[1, 3])
        T_map_base = make_T(rpy_to_matrix(0.0, 0.0, yaw), [x, y, 0.0])
        T_odom_base = transform_to_T(tf_odom_base)
        T_map_odom_sample = map_odom_correction(T_map_base, T_odom_base)

        # Buffer T_map_odom, not T_map_base: odom already backs out the robot's
        # own motion, so samples taken while driving still cluster tightly here.
        odom_yaw = matrix_to_rpy(T_map_odom_sample[:3, :3])[2]
        self.window.append((float(T_map_odom_sample[0, 3]), float(T_map_odom_sample[1, 3]), odom_yaw))
        self.accept_count += 1

        if len(self.window) < self.args.median_window:
            self._throttled_log(
                f"collecting samples: {len(self.window)}/{self.args.median_window} | "
                f"totals rejected: z={self.reject_z_count} tilt={self.reject_tilt_count} "
                f"accepted={self.accept_count}"
            )
            return

        xs, ys, yaws = zip(*self.window)
        med_x = float(np.median(xs))
        med_y = float(np.median(ys))
        med_yaw = median_angle(np.array(yaws))
        T_map_odom = make_T(rpy_to_matrix(0.0, 0.0, med_yaw), [med_x, med_y, 0.0])

        if self.args.publish_tf:
            self.tf_broadcaster.sendTransform(
                T_to_transform(
                    T_map_odom,
                    self.get_clock().now().to_msg(),
                    self.args.map_frame,
                    self.args.odom_frame,
                )
            )

        self.print_status(T_map_base, T_map_odom)

    def pose_to_T(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        return make_T(
            quaternion_to_matrix(q.x, q.y, q.z, q.w),
            [p.x, p.y, p.z],
        )

    def print_status(self, T_map_base, T_map_odom):
        now = time.time()
        if now - self.last_print_time < self.args.print_period:
            return

        mode = "tf" if self.args.publish_tf else "dry-run"
        _, _, odom_yaw = matrix_to_rpy(T_map_odom[:3, :3])
        print(
            f"{mode} T_map_base(latest): {pose_str(T_map_base)} | "
            f"T_map_odom(median x{self.args.median_window}): x={T_map_odom[0, 3]:+.4f} "
            f"y={T_map_odom[1, 3]:+.4f} "
            f"yaw={math.degrees(odom_yaw):+.2f}deg | "
            f"rejected: z={self.reject_z_count} tilt={self.reject_tilt_count} "
            f"accepted={self.accept_count}"
        )
        self.last_print_time = now

    def _throttled_log(self, text):
        now = time.time()
        if now - self.last_print_time >= self.args.print_period:
            self.get_logger().info(text)
            self.last_print_time = now


def parse_args():
    robot_namespace = default_robot_namespace()
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default=scoped_topic(robot_namespace, "aruco/id24/pose_camera"))
    parser.add_argument("--marker-map", default="logitle_marker_map.yaml")
    parser.add_argument("--marker-id", type=int, default=24)
    parser.add_argument("--publish-tf", action="store_true")

    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default=scoped_frame(robot_namespace, "odom"))
    parser.add_argument("--base-frame", default=scoped_frame(robot_namespace, "base_link"))

    parser.add_argument("--camera-x", type=float, default=0.05)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.10)
    parser.add_argument("--camera-pitch", type=float, default=0.0)
    parser.add_argument("--camera-yaw", type=float, default=0.0)
    parser.add_argument("--camera-roll", type=float, default=0.0)

    parser.add_argument("--tf-timeout", type=float, default=0.05)
    parser.add_argument("--print-period", type=float, default=0.5)

    parser.add_argument(
        "--outlier-z-max", type=float, default=0.05,
        help="Reject samples whose estimated base_link height exceeds this [m].",
    )
    parser.add_argument(
        "--outlier-tilt-deg", type=float, default=5.0,
        help="Reject samples whose estimated roll or pitch exceeds this [deg].",
    )
    parser.add_argument(
        "--expected-base-yaw-deg", type=float, default=None,
        help="Optional expected map->base_link yaw; reject samples outside --yaw-tolerance-deg.",
    )
    parser.add_argument(
        "--yaw-tolerance-deg", type=float, default=5.0,
        help="Yaw tolerance used with --expected-base-yaw-deg [deg].",
    )
    parser.add_argument(
        "--median-window", type=int, default=15,
        help="Number of accepted samples to median-filter before printing/publishing.",
    )
    parser.add_argument(
        "--stale-timeout", type=float, default=1.0,
        help="Reset the median window if the gap since the last marker message exceeds this [s].",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = None
    try:
        node = ArucoTfCorrector(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
