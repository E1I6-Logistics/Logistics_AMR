"""Publish saved staging-pose results for display in RViz2."""

import argparse
import csv
import math
from pathlib import Path

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from staging_pose_manager.paths import records_dir
from staging_pose_manager.pose_statistics import calculate_statistics
from visualization_msgs.msg import Marker, MarkerArray


FRAME_ID = "map"


def _read_csv(path):
    with Path(path).open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def select_session(root, station_id, session_id="", latest=True):
    """Resolve a complete session from an explicit ID or the latest result."""
    root = Path(root).expanduser().resolve()
    if not station_id:
        raise ValueError("station_id가 필요합니다.")
    station_dir = root / station_id
    if not station_dir.is_dir():
        raise ValueError(f"Station records가 없습니다: {station_dir}")

    if session_id:
        session_dir = station_dir / session_id
        if not (session_dir / "summary.csv").is_file():
            raise ValueError(f"완료된 session이 아닙니다: {session_dir}")
        return session_dir
    if not latest:
        raise ValueError("session_id를 지정하거나 latest=true를 사용하세요.")

    candidates = [
        path for path in station_dir.iterdir()
        if path.is_dir() and (path / "summary.csv").is_file()
    ]
    if not candidates:
        raise ValueError(f"완료된 session이 없습니다: {station_dir}")
    return max(candidates, key=lambda path: path.name)


class StagingPoseVisualizer(Node):
    """Publish raw samples, trial poses and the final pose as RViz markers."""

    def __init__(self, defaults=None):
        super().__init__("staging_pose_visualizer")
        defaults = defaults or {}
        self.declare_parameter("records_dir", defaults.get("records_dir", str(records_dir())))
        self.declare_parameter("station_id", defaults.get("station_id", ""))
        self.declare_parameter("session_id", defaults.get("session_id", ""))
        self.declare_parameter("latest", defaults.get("latest", True))

        root = self.get_parameter("records_dir").value
        station_id = self.get_parameter("station_id").value
        session_id = self.get_parameter("session_id").value
        latest = self.get_parameter("latest").value
        self.session_dir = select_session(root, station_id, session_id, latest)
        self.samples = _read_csv(self.session_dir / "samples.csv")
        self.trials = _read_csv(self.session_dir / "trials.csv")
        summaries = _read_csv(self.session_dir / "summary.csv")
        if len(summaries) != 1:
            raise ValueError("summary.csv에는 한 개의 결과 행이 있어야 합니다.")
        self.summary = summaries[0]

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/staging_pose_manager/markers", qos
        )
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/staging_pose_manager/final_pose",
            qos,
        )
        self.timer = self.create_timer(1.0, self.publish_result)
        self.publish_result()
        self.get_logger().info(f"Visualizing session: {self.session_dir}")

    def publish_result(self):
        """Publish a complete snapshot; transient-local QoS retains it."""
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self._points_marker("baseline_samples", 0, True, stamp))
        markers.markers.append(self._points_marker("trial_samples", 1, False, stamp))

        baseline_samples = [
            self._numeric_sample(row)
            for row in self.samples
            if row["measurement"] == "baseline"
        ]
        marker_id = 10
        if baseline_samples:
            baseline = calculate_statistics(baseline_samples)["mean"]
            markers.markers.append(
                self._arrow_marker(
                    "baseline", marker_id, baseline, (1.0, 0.75, 0.0, 0.85), stamp
                )
            )
            marker_id += 1

        for row in self.trials:
            pose = {
                "x": float(row["mean_x"]),
                "y": float(row["mean_y"]),
                "yaw": float(row["mean_yaw"]),
            }
            markers.markers.append(
                self._arrow_marker(
                    "trials", marker_id, pose, (0.1, 0.45, 1.0, 0.9), stamp
                )
            )
            marker_id += 1

        final_pose = self._final_pose_values()
        markers.markers.append(
            self._arrow_marker(
                "final", 100, final_pose, (0.0, 1.0, 0.25, 1.0), stamp, large=True
            )
        )
        markers.markers.append(self._ellipse_marker(final_pose, stamp))
        markers.markers.append(self._text_marker(final_pose, stamp))
        self.marker_publisher.publish(markers)
        self.pose_publisher.publish(self._covariance_pose(final_pose, stamp))

    def _points_marker(self, namespace, marker_id, baseline, stamp):
        marker = self._marker(namespace, marker_id, Marker.POINTS, stamp)
        marker.scale.x = 0.018
        marker.scale.y = 0.018
        if baseline:
            self._set_color(marker, 1.0, 0.75, 0.0, 0.55)
        else:
            self._set_color(marker, 0.1, 0.45, 1.0, 0.45)
        wanted = "baseline" if baseline else "trial"
        for row in self.samples:
            if row["measurement"] == wanted:
                marker.points.append(Point(x=float(row["x"]), y=float(row["y"])))
        return marker

    def _arrow_marker(self, namespace, marker_id, pose, color, stamp, large=False):
        marker = self._marker(namespace, marker_id, Marker.ARROW, stamp)
        marker.pose.position.x = pose["x"]
        marker.pose.position.y = pose["y"]
        marker.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
        marker.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
        marker.scale.x = 0.42 if large else 0.28
        marker.scale.y = 0.08 if large else 0.045
        marker.scale.z = 0.08 if large else 0.045
        self._set_color(marker, *color)
        return marker

    def _ellipse_marker(self, pose, stamp):
        marker = self._marker("repeatability_2sigma", 101, Marker.LINE_STRIP, stamp)
        marker.scale.x = 0.012
        self._set_color(marker, 0.0, 1.0, 0.25, 0.8)
        radius_x = max(2.0 * float(self.summary["std_x"]), 0.002)
        radius_y = max(2.0 * float(self.summary["std_y"]), 0.002)
        for index in range(65):
            angle = 2.0 * math.pi * index / 64.0
            marker.points.append(Point(
                x=pose["x"] + radius_x * math.cos(angle),
                y=pose["y"] + radius_y * math.sin(angle),
                z=0.01,
            ))
        return marker

    def _text_marker(self, pose, stamp):
        marker = self._marker("label", 102, Marker.TEXT_VIEW_FACING, stamp)
        marker.pose.position.x = pose["x"]
        marker.pose.position.y = pose["y"]
        marker.pose.position.z = 0.35
        marker.scale.z = 0.12
        self._set_color(marker, 1.0, 1.0, 1.0, 1.0)
        marker.text = (
            f"{self.summary['station_id']}\n"
            f"std: {float(self.summary['std_x']) * 1000:.1f}, "
            f"{float(self.summary['std_y']) * 1000:.1f} mm"
        )
        return marker

    def _covariance_pose(self, pose, stamp):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = FRAME_ID
        message.header.stamp = stamp
        message.pose.pose.position.x = pose["x"]
        message.pose.pose.position.y = pose["y"]
        message.pose.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
        message.pose.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
        message.pose.covariance[0] = float(self.summary["std_x"]) ** 2
        message.pose.covariance[7] = float(self.summary["std_y"]) ** 2
        message.pose.covariance[35] = float(self.summary["std_yaw"]) ** 2
        return message

    def _final_pose_values(self):
        return {
            "x": float(self.summary["mean_x"]),
            "y": float(self.summary["mean_y"]),
            "yaw": float(self.summary["mean_yaw"]),
        }

    @staticmethod
    def _numeric_sample(row):
        return {"x": float(row["x"]), "y": float(row["y"]), "yaw": float(row["yaw"])}

    @staticmethod
    def _set_color(marker, red, green, blue, alpha):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    @staticmethod
    def _marker(namespace, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker


def _parse_arguments(args=None):
    parser = argparse.ArgumentParser(description="Visualize a saved staging pose")
    parser.add_argument("--station", default="", dest="station_id")
    parser.add_argument("--session", default="", dest="session_id")
    parser.add_argument("--records-dir", default=str(records_dir()))
    parser.add_argument("--latest", action="store_true", default=True)
    return parser.parse_known_args(args)


def main(args=None):
    """Run the saved-result RViz publisher."""
    options, ros_args = _parse_arguments(args)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = StagingPoseVisualizer(vars(options))
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as exc:
        if node:
            node.get_logger().info(str(exc))
        else:
            print(f"visualize_pose: {exc}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
