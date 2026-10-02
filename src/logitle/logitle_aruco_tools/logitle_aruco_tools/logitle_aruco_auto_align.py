#!/usr/bin/env python3
"""Dry-run ArUco auto alignment controller.

Subscribes to /aruco/id24/pose_camera and computes the cmd_vel that would
align the robot with the marker. By default it only prints the command.
Use --enable-motion only after checking the signs with the wheels lifted.
"""

import argparse
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def clamp(value, low, high):
    return max(low, min(high, value))


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


class ArucoAutoAlign(Node):
    def __init__(self, args):
        super().__init__("aruco_auto_align")
        self.args = args
        self.last_pose_time = None
        self.last_print_time = 0.0

        self.create_subscription(PoseStamped, args.pose_topic, self.pose_cb, 10)
        cmd_type = TwistStamped if args.cmd_vel_stamped else Twist
        self.cmd_pub = self.create_publisher(cmd_type, args.cmd_vel_topic, 10)
        self.create_timer(0.05, self.watchdog_cb)

        mode = "MOTION ENABLED" if args.enable_motion else "dry-run"
        self.get_logger().info(
            f"Listening to {args.pose_topic}; mode={mode}; "
            f"cmd_type={cmd_type.__name__}; "
            f"target_z={args.z_target:.3f}m, x_tol={args.x_tol:.3f}m"
        )

    def pose_cb(self, msg):
        self.last_pose_time = time.time()
        x = msg.pose.position.x
        z = msg.pose.position.z
        linear_x, angular_z, state = self.compute_cmd(x, z)

        if self.args.enable_motion:
            self.publish_cmd(linear_x, angular_z)

        self.print_status(x, z, linear_x, angular_z, state)

    def compute_cmd(self, x, z):
        if z < self.args.z_min_stop:
            return 0.0, 0.0, "too_close_stop"

        x_error = 0.0 if abs(x) <= self.args.x_tol else x
        z_error = 0.0 if abs(z - self.args.z_target) <= self.args.z_tol else z - self.args.z_target

        angular_z = clamp(
            -self.args.kx * x_error,
            -self.args.max_angular,
            self.args.max_angular,
        )
        linear_x = clamp(
            self.args.kz * z_error,
            -self.args.max_linear,
            self.args.max_linear,
        )

        if x_error == 0.0 and z_error == 0.0:
            return 0.0, 0.0, "aligned"

        return linear_x, angular_z, "aligning"

    def watchdog_cb(self):
        if self.last_pose_time is None:
            return

        age = time.time() - self.last_pose_time
        if age <= self.args.pose_timeout:
            return

        if self.args.enable_motion:
            self.publish_cmd(0.0, 0.0)

        now = time.time()
        if now - self.last_print_time >= self.args.print_period:
            print(f"pose_timeout age={age:.2f}s -> cmd linear=+0.000 angular=+0.000")
            self.last_print_time = now

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

    def print_status(self, x, z, linear_x, angular_z, state):
        now = time.time()
        if now - self.last_print_time < self.args.print_period:
            return

        prefix = "cmd" if self.args.enable_motion else "dry-run cmd"
        print(
            f"x={x:+.3f}m z={z:.3f}m state={state} -> "
            f"{prefix} linear.x={linear_x:+.3f} angular.z={angular_z:+.3f}"
        )
        self.last_print_time = now

    def stop(self):
        if self.args.enable_motion:
            for _ in range(3):
                self.publish_cmd(0.0, 0.0)


def parse_args():
    robot_namespace = default_robot_namespace()
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default=scoped_topic(robot_namespace, "aruco/id24/pose_camera"))
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--cmd-vel-stamped", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--cmd-frame-id", default="base_footprint")
    parser.add_argument("--enable-motion", action="store_true")

    parser.add_argument("--x-tol", type=float, default=0.02)
    parser.add_argument("--z-target", type=float, default=0.32)
    parser.add_argument("--z-tol", type=float, default=0.03)
    parser.add_argument("--z-min-stop", type=float, default=0.25)

    parser.add_argument("--kx", type=float, default=2.0)
    parser.add_argument("--kz", type=float, default=0.4)
    parser.add_argument("--max-linear", type=float, default=0.04)
    parser.add_argument("--max-angular", type=float, default=0.20)

    parser.add_argument("--pose-timeout", type=float, default=0.3)
    parser.add_argument("--print-period", type=float, default=0.2)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    rclpy.init()
    node = ArucoAutoAlign(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
