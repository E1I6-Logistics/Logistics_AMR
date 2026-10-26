"""Odom-feedback perturbation motion using velocity commands."""

import math
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy


DEFAULT_SETTLE_TIME = 3.0


def quaternion_to_yaw(x, y, z, w):
    """Convert a quaternion to planar yaw."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def angle_diff(a, b):
    """Return the shortest signed angular difference."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MotionError(RuntimeError):
    """Raised when automatic perturbation cannot safely finish."""


class OdomMotionController:
    """Move away and approximately back using relative odometry feedback."""

    def __init__(
        self,
        node,
        cmd_vel_topic="/cmd_vel",
        odom_topic="/odom",
        settle_time=DEFAULT_SETTLE_TIME,
    ):
        self.node = node
        self.odom = None
        self.settle_time = float(settle_time)
        if not math.isfinite(self.settle_time) or self.settle_time < 0.0:
            raise ValueError("settle_time must be a non-negative finite number")
        self.publisher = node.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.subscription = node.create_subscription(
            Odometry, odom_topic, self._odom_callback, 20
        )

    def _odom_callback(self, message):
        pose = message.pose.pose
        self.odom = (
            pose.position.x,
            pose.position.y,
            quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def wait_for_odom(self, timeout=5.0):
        """Wait for a first odometry message."""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.odom is None:
            raise MotionError("/odom 메시지를 받지 못했습니다.")

    def perturb_translation(self, distance, outward_sign, speed=0.12):
        """Translate out and back; final physical alignment remains manual."""
        distance = self._positive(distance, "distance")
        self._translate(distance, math.copysign(speed, outward_sign))
        self.hold_stopped()
        self._translate(distance, -math.copysign(speed, outward_sign))
        self.hold_stopped()

    def perturb_rotation(self, angle_radians, outward_sign, speed=0.35):
        """Rotate out and back; final heading alignment remains manual."""
        angle_radians = self._positive(angle_radians, "angle")
        self._rotate(angle_radians, math.copysign(speed, outward_sign))
        self.hold_stopped()
        self._rotate(angle_radians, -math.copysign(speed, outward_sign))
        self.hold_stopped()

    def hold_stopped(self, duration=None):
        """Continuously publish zero velocity while the base settles."""
        duration = self.settle_time if duration is None else float(duration)
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError("stop duration must be a non-negative finite number")
        if duration == 0.0:
            self.stop()
            return

        self.node.get_logger().info(f"정지 안정화 대기: {duration:.1f}초")
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            message = self._command()
            try:
                self.publisher.publish(message)
            except RuntimeError:
                return
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )
        self.stop()

    def stop(self):
        """Publish several zero commands to ensure the base stops."""
        if not rclpy.ok():
            return
        message = self._command()
        for _ in range(3):
            try:
                message.header.stamp = self.node.get_clock().now().to_msg()
                self.publisher.publish(message)
            except RuntimeError:
                return
            time.sleep(0.03)

    def _translate(self, distance, velocity):
        self.wait_for_odom()
        start_x, start_y, start_yaw = self.odom
        direction = 1.0 if velocity > 0.0 else -1.0
        timeout = max(10.0, distance / abs(velocity) * 3.0 + 3.0)
        deadline = time.monotonic() + timeout
        command = self._command(linear_x=velocity)
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.03)
                if self.odom is not None:
                    dx = self.odom[0] - start_x
                    dy = self.odom[1] - start_y
                    projected = direction * (
                        dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
                    )
                    if projected >= distance:
                        return
                if time.monotonic() >= deadline:
                    raise MotionError("직선 이동이 제한 시간 안에 완료되지 않았습니다.")
                command.header.stamp = self.node.get_clock().now().to_msg()
                self.publisher.publish(command)
        finally:
            self.stop()
        raise MotionError("ROS 종료로 직선 이동이 중단되었습니다.")

    def _rotate(self, target_angle, velocity):
        self.wait_for_odom()
        previous_yaw = self.odom[2]
        accumulated = 0.0
        direction = 1.0 if velocity > 0.0 else -1.0
        timeout = max(10.0, target_angle / abs(velocity) * 3.0 + 3.0)
        deadline = time.monotonic() + timeout
        command = self._command(angular_z=velocity)
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.03)
                if self.odom is not None:
                    yaw = self.odom[2]
                    accumulated += direction * angle_diff(yaw, previous_yaw)
                    previous_yaw = yaw
                    if accumulated >= target_angle:
                        return
                if time.monotonic() >= deadline:
                    raise MotionError("회전이 제한 시간 안에 완료되지 않았습니다.")
                command.header.stamp = self.node.get_clock().now().to_msg()
                self.publisher.publish(command)
        finally:
            self.stop()
        raise MotionError("ROS 종료로 회전이 중단되었습니다.")

    @staticmethod
    def _positive(value, label):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{label} must be a positive finite number")
        return value

    def _command(self, linear_x=0.0, angular_z=0.0):
        """Build a stamped velocity command for ROS 2 Jazzy TurtleBot."""
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        return message
