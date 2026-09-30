#!/usr/bin/env python3
"""Monitor whether a TF transform changes continuously over time."""

import argparse
import csv
import math
import sys
from dataclasses import dataclass

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class Sample:
    wall_time: float
    tf_time: float
    x: float
    y: float
    z: float
    yaw: float


class TfContinuityMonitor(Node):
    def __init__(self, args):
        super().__init__("tf_continuity_monitor")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_time = self.get_clock().now()
        self.last_sample = None
        self.last_print_wall = 0.0
        self.sample_count = 0
        self.missing_count = 0
        self.jump_count = 0
        self.max_step_xy = 0.0
        self.max_step_yaw = 0.0
        self.max_gap = 0.0
        self.csv_file = None
        self.csv_writer = None

        if args.csv:
            self.csv_file = open(args.csv, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "wall_time",
                "tf_time",
                "x",
                "y",
                "z",
                "yaw_deg",
                "dt",
                "step_xy",
                "step_yaw_deg",
                "jump",
            ])

        self.create_timer(args.sample_period, self.tick)
        self.get_logger().info(
            f"Monitoring {args.parent_frame}->{args.child_frame} every "
            f"{args.sample_period:.3f}s; jump limits: "
            f"xy>{args.jump_xy_threshold:.3f}m/sample or "
            f"yaw>{args.jump_yaw_threshold_deg:.1f}deg/sample"
        )

    def tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if self.args.duration_sec > 0.0 and elapsed >= self.args.duration_sec:
            self.print_summary()
            rclpy.shutdown()
            return

        try:
            msg = self.tf_buffer.lookup_transform(
                self.args.parent_frame,
                self.args.child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.args.tf_timeout),
            )
        except TransformException as exc:
            self.missing_count += 1
            self._throttled_status(f"missing TF: {exc}")
            return

        now_msg = self.get_clock().now().to_msg()
        wall_time = stamp_to_float(now_msg)
        tf_time = stamp_to_float(msg.header.stamp)
        t = msg.transform.translation
        yaw = yaw_from_quaternion(msg.transform.rotation)
        sample = Sample(wall_time, tf_time, float(t.x), float(t.y), float(t.z), yaw)

        dt = 0.0
        step_xy = 0.0
        step_yaw = 0.0
        jump = False
        if self.last_sample is not None:
            dt = max(sample.wall_time - self.last_sample.wall_time, 0.0)
            dx = sample.x - self.last_sample.x
            dy = sample.y - self.last_sample.y
            step_xy = math.hypot(dx, dy)
            step_yaw = abs(math.degrees(angle_diff(sample.yaw, self.last_sample.yaw)))
            self.max_step_xy = max(self.max_step_xy, step_xy)
            self.max_step_yaw = max(self.max_step_yaw, step_yaw)
            self.max_gap = max(self.max_gap, dt)
            jump = (
                step_xy > self.args.jump_xy_threshold
                or step_yaw > self.args.jump_yaw_threshold_deg
                or dt > self.args.stale_threshold
            )
            if jump:
                self.jump_count += 1
                self.get_logger().warn(
                    f"jump? x={sample.x:+.3f} y={sample.y:+.3f} "
                    f"yaw={math.degrees(sample.yaw):+.1f}deg | "
                    f"dt={dt:.3f}s dxy={step_xy:.3f}m "
                    f"dyaw={step_yaw:.1f}deg"
                )

        self.sample_count += 1
        self.last_sample = sample
        if self.csv_writer is not None:
            self.csv_writer.writerow([
                f"{sample.wall_time:.6f}",
                f"{sample.tf_time:.6f}",
                f"{sample.x:.6f}",
                f"{sample.y:.6f}",
                f"{sample.z:.6f}",
                f"{math.degrees(sample.yaw):.3f}",
                f"{dt:.6f}",
                f"{step_xy:.6f}",
                f"{step_yaw:.3f}",
                int(jump),
            ])
            self.csv_file.flush()

        self._throttled_status(
            f"x={sample.x:+.3f} y={sample.y:+.3f} "
            f"yaw={math.degrees(sample.yaw):+.1f}deg | "
            f"last dxy={step_xy:.3f}m dyaw={step_yaw:.1f}deg "
            f"jumps={self.jump_count}"
        )

    def _throttled_status(self, text):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_print_wall >= self.args.print_period:
            self.get_logger().info(text)
            self.last_print_wall = now

    def print_summary(self):
        status = "OK" if self.jump_count == 0 and self.sample_count > 1 else "CHECK"
        self.get_logger().info(
            f"summary={status} samples={self.sample_count} missing={self.missing_count} "
            f"jumps={self.jump_count} max_dxy={self.max_step_xy:.3f}m "
            f"max_dyaw={self.max_step_yaw:.1f}deg max_gap={self.max_gap:.3f}s"
        )

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--parent-frame", default="map")
    parser.add_argument("--child-frame", default="base_link")
    parser.add_argument("--sample-period", type=float, default=0.1)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--tf-timeout", type=float, default=0.05)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--jump-xy-threshold", type=float, default=0.08)
    parser.add_argument("--jump-yaw-threshold-deg", type=float, default=10.0)
    parser.add_argument("--stale-threshold", type=float, default=0.5)
    parser.add_argument("--csv", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = None
    try:
        node = TfContinuityMonitor(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.print_summary()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
