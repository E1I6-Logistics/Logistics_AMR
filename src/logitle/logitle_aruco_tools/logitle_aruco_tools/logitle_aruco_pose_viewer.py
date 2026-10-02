#!/usr/bin/env python3
"""OpenCV ArUco pose viewer for manual marker alignment.

Subscribes to a ROS image topic, reads camera intrinsics from CameraInfo, and
prints the selected marker pose in the camera optical frame.
"""

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image

from logitle_aruco_tools.logitle_marker_map import load_marker_map


ARUCO_DICTS = {
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "4X4_250": cv2.aruco.DICT_4X4_250,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "5X5_100": cv2.aruco.DICT_5X5_100,
    "5X5_250": cv2.aruco.DICT_5X5_250,
    "5X5_1000": cv2.aruco.DICT_5X5_1000,
    "6X6_50": cv2.aruco.DICT_6X6_50,
    "6X6_100": cv2.aruco.DICT_6X6_100,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "6X6_1000": cv2.aruco.DICT_6X6_1000,
}


def make_detector_params():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()

    params.minMarkerPerimeterRate = 0.05
    params.maxMarkerPerimeterRate = 0.8
    params.minCornerDistanceRate = 0.05
    params.minDistanceToBorder = 10
    return params


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


def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to ROS quaternion order (x, y, z, w)."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    trace = float(np.trace(R))

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(q))
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


class ArucoPoseViewer(Node):
    def __init__(self, args):
        super().__init__("aruco_pose_viewer")
        self.args = args
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_print = 0.0
        self.last_save = 0.0
        self.camera_info_shape = None
        self.marker_ids = args.marker_ids or [args.marker_id]
        self.pose_pubs = {}
        self.marker_sizes = {marker_id: args.marker_size for marker_id in self.marker_ids}
        if args.marker_map:
            markers = load_marker_map(args.marker_map)
            for marker_id in self.marker_ids:
                if marker_id in markers:
                    self.marker_sizes[marker_id] = markers[marker_id].size

        dictionary_id = ARUCO_DICTS[args.dictionary]
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.params = make_detector_params()

        self.create_subscription(CameraInfo, args.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, args.image_topic, self.image_cb, 10)
        if len(self.marker_ids) == 1:
            marker_id = self.marker_ids[0]
            pose_topic = args.pose_topic or self.pose_topic_for_marker(marker_id)
            self.pose_pubs[marker_id] = self.create_publisher(PoseStamped, pose_topic, 10)
        else:
            if args.pose_topic:
                raise ValueError("--pose-topic can only be used with a single marker id")
            for marker_id in self.marker_ids:
                pose_topic = self.pose_topic_for_marker(marker_id)
                self.pose_pubs[marker_id] = self.create_publisher(PoseStamped, pose_topic, 10)

        self.get_logger().info(
            f"Waiting for {args.camera_info_topic} and {args.image_topic}; "
            f"dictionary={args.dictionary}, marker_ids={self.marker_ids}, "
            f"marker_sizes={self.marker_sizes}"
        )

    def pose_topic_for_marker(self, marker_id):
        try:
            return self.args.pose_topic_template.format(marker_id=marker_id)
        except (KeyError, IndexError, ValueError):
            return self.args.pose_topic_template

    def camera_info_cb(self, msg):
        if len(msg.k) != 9 or msg.k[0] == 0.0:
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float32)
        shape = (msg.width, msg.height)
        if shape != self.camera_info_shape:
            self.camera_info_shape = shape
            self.get_logger().info(f"camera_info {msg.width}x{msg.height}")

    def image_cb(self, msg):
        if self.camera_matrix is None:
            if self.args.approx_camera_info:
                self._set_approx_camera_info(msg.width, msg.height)
            else:
                self._throttled_print("waiting for valid camera_info")
                return

        if self.camera_matrix is None:
            self._throttled_print("waiting for valid camera_info")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame,
            self.dictionary,
            parameters=self.params,
        )

        if ids is None:
            self._throttled_print("no marker")
            self._show_or_save(frame)
            return

        ids_list = ids.flatten().tolist()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        visible_target_ids = [marker_id for marker_id in self.marker_ids if marker_id in ids_list]
        if not visible_target_ids:
            self._throttled_print(f"other ids ignored: {ids_list}")
            self._show_or_save(frame)
            return

        pose_lines = []
        for marker_id in visible_target_ids:
            idx = ids_list.index(marker_id)
            marker_size = self.marker_sizes[marker_id]
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[idx]],
                marker_size,
                self.camera_matrix,
                self.dist_coeffs,
            )
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]
            x, y, z = tvec
            distance = float(np.linalg.norm(tvec))
            self._publish_pose(marker_id, msg.header.stamp, rvec, tvec)

            pose_lines.append(
                f"id={marker_id} x={x:.3f}m y={y:.3f}m "
                f"z={z:.3f}m distance={distance:.3f}m size={marker_size:.4f}m"
            )
            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                marker_size * 0.5,
            )
        self._throttled_print(" | ".join(pose_lines))
        self._show_or_save(frame)

    def _publish_pose(self, marker_id, stamp, rvec, tvec):
        pose_pub = self.pose_pubs.get(marker_id)
        if pose_pub is None:
            return

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
        pose_pub.publish(pose)

    def _set_approx_camera_info(self, width, height):
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
        self.get_logger().info(
            "using approximate camera_info "
            f"{width}x{height} fx={fx:.1f} fy={fy:.1f} "
            f"cx={cx:.1f} cy={cy:.1f}; calibrate camera for final TF"
        )

    def _throttled_print(self, text):
        now = time.time()
        if now - self.last_print >= self.args.print_period:
            self.get_logger().info(text)
            self.last_print = now

    def _show_or_save(self, frame):
        now = time.time()
        if self.args.save_path and now - self.last_save >= self.args.save_period:
            cv2.imwrite(self.args.save_path, frame)
            self.last_save = now

        if not self.args.no_gui:
            cv2.imshow("aruco_pose_viewer", frame)
            cv2.waitKey(1)


def parse_marker_ids(text):
    marker_ids = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        marker_id = int(part)
        if marker_id < 0:
            raise argparse.ArgumentTypeError("marker ids must be non-negative")
        marker_ids.append(marker_id)
    if not marker_ids:
        raise argparse.ArgumentTypeError("expected at least one marker id")
    return marker_ids


def parse_args():
    robot_namespace = default_robot_namespace()
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--dictionary", choices=sorted(ARUCO_DICTS), default="5X5_1000")
    parser.add_argument("--marker-id", type=int, default=24)
    parser.add_argument(
        "--marker-ids",
        type=parse_marker_ids,
        default=None,
        help="Comma-separated marker ids to track together, e.g. 24,25.",
    )
    parser.add_argument("--marker-size", type=float, default=0.04)
    parser.add_argument(
        "--marker-map",
        default=None,
        help="Optional logitle_marker_map.yaml. When set, per-ID size values override --marker-size.",
    )
    parser.add_argument("--pose-topic", default=None)
    parser.add_argument(
        "--pose-topic-template",
        default=scoped_topic(robot_namespace, "aruco/id{marker_id}/pose_camera"),
    )
    parser.add_argument("--pose-frame-id", default="camera_optical_frame")
    parser.add_argument("--print-period", type=float, default=0.5)
    parser.add_argument("--save-path", default="/tmp/aruco_pose_viewer.jpg")
    parser.add_argument("--save-period", type=float, default=1.0)
    parser.add_argument(
        "--approx-camera-info",
        action="store_true",
        help="Use approximate intrinsics when /camera/camera_info has empty K values.",
    )
    parser.add_argument(
        "--approx-horizontal-fov-deg",
        type=float,
        default=62.2,
        help="Horizontal FOV used with --approx-camera-info.",
    )
    parser.add_argument("--no-gui", action="store_true")
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    rclpy.init()
    node = ArucoPoseViewer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
