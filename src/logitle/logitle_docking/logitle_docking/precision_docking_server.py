"""Map-referenced precision docking with fail-closed ICP and recovery."""

from enum import Enum
import math
import threading

import numpy as np
import rclpy as rp
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros
from turtlebot3_my_msg.action import PrecisionDock

from .docking_core import (
    icp_2d, icp_is_safe, make_v_funnel,
    model_transform_from_relative_pose, normalize_angle,
    relative_pose, staging_pose,
)


class DockingState(Enum):
    """States in the map, ICP, final approach, and recovery sequence."""

    MAP_TURN = 0
    MAP_DRIVE = 1
    MAP_ALIGN = 2
    ICP_SETTLE = 3
    ICP_ALIGN = 4
    ICP_REVERSE = 5
    FINAL_REVERSE = 6
    RECOVERY_RETREAT = 7
    RECOVERY_ALIGN = 8
    COMPLETED = 9


def yaw_from_quaternion(q):
    """Extract planar yaw from a quaternion-like ROS message."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PID:
    """Small PID controller with output clamping and anti-windup."""

    def __init__(self, p, i, d, out_min, out_max):
        self.p, self.i, self.d = p, i, d
        self.out_min, self.out_max = out_min, out_max
        self.reset()

    def update(self, error, dt=0.05):
        dt = max(dt, 0.01)
        self.integral += error * dt
        output = (
            self.p * error + self.i * self.integral
            + self.d * (error - self.previous_error) / dt
        )
        if output > self.out_max or output < self.out_min:
            self.integral -= error * dt
        self.previous_error = error
        return float(np.clip(output, self.out_min, self.out_max))

    def reset(self):
        self.previous_error, self.integral = 0.0, 0.0


class PrecisionDockingServer(Node):
    """Execute map-referenced, lidar-validated rear docking goals."""

    def __init__(self):
        super().__init__('precision_docking_server')
        self.callback_group = ReentrantCallbackGroup()
        self.scan_lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.velocity_publisher = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self, PrecisionDock, 'precision_dock',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self._declare_parameters()
        self._load_parameters()
        self.linear_pid = PID(0.1, 0.0, 0.02, 0.005, 0.015)
        self.angular_pid = PID(0.5, 0.0, 0.10, -0.30, 0.30)
        self.map_linear_pid = PID(4.0, 0.01, 0.02, 0.015, 0.15)
        self.map_angular_pid = PID(2.0, 0.05, 0.08, -0.4, 0.4)
        self._reset_action_state()
        self.get_logger().info(
            f'Map-based precision docking ready (dry_run={self.p["dry_run"]})'
        )

    def _declare_parameters(self):
        defaults = {
            'charger_width': 0.145,
            'wing_length': 0.10,
            'wing_angle_deg': 22.5,
            'robot_rear_length': 0.10,
            'roi_x_min': -1.2,
            'roi_x_max': 0.15,
            'roi_y_limit': 0.15,
            'staging_distance': 0.45,
            'steering_lock_dist': 0.10,
            'reverse_lateral_gain': 1.8,
            'reverse_max_target_yaw_deg': 6.0,
            'final_blind_start_dist': 0.20,
            'final_reverse_speed': 0.015,
            'final_reverse_tolerance': 0.005,
            'final_yaw_hold_kp': 1.5,
            'final_yaw_hold_max_w': 0.12,
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_frame': 'base_scan',
            'dry_run': False,
            'max_scan_age_sec': 0.35,
            'max_tf_age_sec': 0.35,
            'sensor_grace_sec': 2.0,
            'overall_timeout_sec': 90.0,
            'map_waypoint_tolerance': 0.02,
            'map_heading_tolerance_deg': 1.5,
            'icp_min_matches': 12,
            'icp_min_fitness': 0.25,
            'icp_max_rmse': 0.035,
            'icp_max_distance_jump': 0.10,
            'icp_max_lateral_jump': 0.05,
            'icp_max_yaw_jump_deg': 10.0,
            'icp_loss_timeout_sec': 1.0,
            'max_recovery_attempts': 2,
            'recovery_distance': 0.10,
            'recovery_speed': 0.03,
            'recovery_min_clearance': 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.parameter_names = tuple(defaults)

    def _load_parameters(self):
        self.p = {
            name: self.get_parameter(name).value
            for name in self.parameter_names
        }
        make_v_funnel(
            self.p['charger_width'], self.p['wing_length'],
            self.p['wing_angle_deg'],
        )

    def _reset_action_state(self):
        with self.scan_lock:
            self.latest_scan_points = None
            self.latest_scan_frame = None
            self.latest_scan_time_ns = None
            self.latest_front_clearance = None
            self.new_scan_available = False
        self.target_points = make_v_funnel(
            self.p['charger_width'], self.p['wing_length'],
            self.p['wing_angle_deg'],
        )
        self.current_transform = np.identity(3, dtype=np.float32)
        self.filtered_yaw = None
        self.filtered_distance = None
        self.filtered_lateral = None
        self.last_valid_icp_ns = None
        self.final_start_pose = None
        self.final_distance = 0.0
        self.recovery_start_pose = None
        self.recovery_attempts = 0
        self.settle_until_ns = None

    def scan_callback(self, message):
        """Store a recent scan and forward recovery clearance."""
        frame = message.header.frame_id.lstrip('/')
        expected_frame = str(self.p['scan_frame']).lstrip('/')
        if not frame or (expected_frame and frame != expected_frame):
            return
        ranges = np.asarray(message.ranges, dtype=np.float32)
        angles = (
            message.angle_min + np.arange(len(ranges), dtype=np.float32)
            * message.angle_increment
        )
        valid = (
            (ranges > message.range_min) & (ranges < message.range_max)
            & np.isfinite(ranges)
        )
        points = np.column_stack((
            ranges[valid] * np.cos(angles[valid]),
            ranges[valid] * np.sin(angles[valid]),
        )).astype(np.float32)
        front = valid & (np.abs(angles) <= math.radians(20.0))
        front_clearance = (
            float(np.min(ranges[front])) if np.any(front) else None
        )
        with self.scan_lock:
            self.latest_scan_points = points
            self.latest_scan_frame = frame
            self.latest_scan_time_ns = self.get_clock().now().nanoseconds
            self.latest_front_clearance = front_clearance
            self.new_scan_available = True

    def _take_scan_snapshot(self):
        with self.scan_lock:
            if not self.new_scan_available:
                return None
            points = self.latest_scan_points
            frame = self.latest_scan_frame
            received_ns = self.latest_scan_time_ns
            self.new_scan_available = False
        if points is None or received_ns is None:
            return None
        age = (self.get_clock().now().nanoseconds - received_ns) / 1e9
        if age > self.p['max_scan_age_sec']:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.p['base_frame'], frame, rp.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        yaw = yaw_from_quaternion(transform.transform.rotation)
        rotation = np.asarray((
            (math.cos(yaw), -math.sin(yaw)),
            (math.sin(yaw), math.cos(yaw)),
        ), dtype=np.float32)
        translation = np.asarray((
            transform.transform.translation.x,
            transform.transform.translation.y,
        ), dtype=np.float32)
        base_points = points @ rotation.T + translation
        roi = (
            (base_points[:, 0] > self.p['roi_x_min'])
            & (base_points[:, 0] < self.p['roi_x_max'])
            & (np.abs(base_points[:, 1]) < self.p['roi_y_limit'])
        )
        selected = base_points[roi]
        return selected if len(selected) >= self.p['icp_min_matches'] else None

    def _recovery_path_is_clear(self):
        """Fail closed if a fresh forward sector is not clear for retreat."""
        with self.scan_lock:
            received_ns = self.latest_scan_time_ns
            clearance = self.latest_front_clearance
        if received_ns is None or clearance is None:
            return False
        age = (self.get_clock().now().nanoseconds - received_ns) / 1e9
        return (
            age <= self.p['max_scan_age_sec']
            and clearance >= self.p['recovery_min_clearance']
        )

    def _scan_is_ready(self):
        """Require a fresh scan and a resolvable scan-to-base transform."""
        with self.scan_lock:
            received_ns = self.latest_scan_time_ns
            frame = self.latest_scan_frame
        if received_ns is None or frame is None:
            return False
        age = (self.get_clock().now().nanoseconds - received_ns) / 1e9
        if age > self.p['max_scan_age_sec']:
            return False
        try:
            self.tf_buffer.lookup_transform(
                self.p['base_frame'], frame, rp.time.Time()
            )
        except tf2_ros.TransformException:
            return False
        return True

    def _map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.p['map_frame'], self.p['base_frame'], rp.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0:
            return None
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age < -0.05 or age > self.p['max_tf_age_sec']:
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw_from_quaternion(transform.transform.rotation),
        )

    def cancel_callback(self, _goal_handle):
        """Accept cancel requests; the execute loop performs the safe stop."""
        return CancelResponse.ACCEPT

    def publish_velocity(self, linear, angular):
        """Publish a stamped velocity, forcing zero commands in dry-run."""
        if self.p['dry_run'] and (linear or angular):
            linear, angular = 0.0, 0.0
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.p['base_frame']
        message.twist.linear.x = float(linear)
        message.twist.angular.z = float(angular)
        self.velocity_publisher.publish(message)

    @staticmethod
    def _goal_pose(goal, map_frame):
        pose = goal.request.target_pose
        if pose.header.frame_id.lstrip('/') != map_frame.lstrip('/'):
            raise ValueError(f'target_pose must be in {map_frame!r}')
        values = (
            pose.pose.position.x, pose.pose.position.y,
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('target_pose contains a non-finite value')
        quaternion_norm = math.sqrt(sum(value * value for value in values[2:]))
        if abs(quaternion_norm - 1.0) > 0.02:
            raise ValueError('target_pose quaternion is not normalized')
        return values[0], values[1], yaw_from_quaternion(pose.pose.orientation)

    @staticmethod
    def _result(success, message):
        result = PrecisionDock.Result()
        result.success, result.message = success, message
        return result

    @staticmethod
    def _feedback(goal_handle, distance, angle):
        feedback = PrecisionDock.Feedback()
        feedback.distance_remaining = float(max(0.0, distance))
        feedback.angle_remaining = float(angle)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _expected_transform(robot_pose, dock_pose):
        return model_transform_from_relative_pose(
            *relative_pose(*robot_pose, *dock_pose)
        )

    def _safe_icp(self, source_points, expected_transform):
        result = icp_2d(
            source_points, self.target_points, expected_transform,
            max_iterations=30, search_radius=0.12, inlier_distance=0.05,
        )
        safe = icp_is_safe(
            result, expected_transform, self.p['icp_min_matches'],
            self.p['icp_min_fitness'], self.p['icp_max_rmse'],
            self.p['icp_max_distance_jump'],
            self.p['icp_max_lateral_jump'],
            self.p['icp_max_yaw_jump_deg'],
        )
        return result, safe

    def _start_recovery(self, robot_pose, reason):
        self.publish_velocity(0.0, 0.0)
        if self.recovery_attempts >= self.p['max_recovery_attempts']:
            return None
        self.recovery_attempts += 1
        self.recovery_start_pose = robot_pose
        self.get_logger().warning(
            f'Recovery {self.recovery_attempts}/'
            f'{self.p["max_recovery_attempts"]}: {reason}'
        )
        return DockingState.RECOVERY_RETREAT

    def execute_callback(self, goal_handle):
        """Run one precision docking action to a map-frame dock pose."""
        self._load_parameters()
        self._reset_action_state()
        for controller in (
            self.linear_pid, self.angular_pid,
            self.map_linear_pid, self.map_angular_pid,
        ):
            controller.reset()
        try:
            dock_pose = self._goal_pose(goal_handle, self.p['map_frame'])
        except ValueError as error:
            goal_handle.abort()
            return self._result(False, str(error))

        waypoint = staging_pose(*dock_pose, self.p['staging_distance'])
        state = DockingState.MAP_TURN
        rate = self.create_rate(20.0)
        started_ns = self.get_clock().now().nanoseconds
        missing_since_ns = None
        scan_missing_since_ns = None
        last_loop_ns, last_feedback_ns = started_ns, 0

        try:
            while rp.ok():
                now_ns = self.get_clock().now().nanoseconds
                dt = max((now_ns - last_loop_ns) / 1e9, 0.01)
                last_loop_ns = now_ns
                if goal_handle.is_cancel_requested:
                    self.publish_velocity(0.0, 0.0)
                    goal_handle.canceled()
                    return self._result(False, 'docking canceled')
                if (now_ns - started_ns) / 1e9 > self.p['overall_timeout_sec']:
                    self.publish_velocity(0.0, 0.0)
                    goal_handle.abort()
                    return self._result(False, 'docking timed out')

                robot_pose = self._map_pose()
                if robot_pose is None:
                    self.publish_velocity(0.0, 0.0)
                    missing_since_ns = missing_since_ns or now_ns
                    if (now_ns - missing_since_ns) / 1e9 > self.p['sensor_grace_sec']:
                        goal_handle.abort()
                        return self._result(False, 'map-to-base TF missing or stale')
                    rate.sleep()
                    continue
                missing_since_ns = None

                if self.p['dry_run']:
                    scan = self._take_scan_snapshot()
                    if scan is None:
                        if (now_ns - started_ns) / 1e9 > self.p['sensor_grace_sec']:
                            goal_handle.abort()
                            return self._result(False, 'dry-run scan/scan-TF validation failed')
                        rate.sleep()
                        continue
                    result, safe = self._safe_icp(
                        scan, self._expected_transform(robot_pose, dock_pose)
                    )
                    self.publish_velocity(0.0, 0.0)
                    if not safe:
                        goal_handle.abort()
                        return self._result(
                            False, 'dry-run ICP rejected '
                            f'(matches={result.matched_count}, '
                            f'fitness={result.fitness:.3f}, rmse={result.rmse:.3f})'
                        )
                    goal_handle.succeed()
                    return self._result(
                        True, 'dry-run safety validation passed; no motion commanded'
                    )

                if not self._scan_is_ready():
                    self.publish_velocity(0.0, 0.0)
                    scan_missing_since_ns = scan_missing_since_ns or now_ns
                    if (
                        now_ns - scan_missing_since_ns
                    ) / 1e9 > self.p['sensor_grace_sec']:
                        goal_handle.abort()
                        return self._result(
                            False, 'scan or scan-to-base TF missing or stale'
                        )
                    rate.sleep()
                    continue
                scan_missing_since_ns = None

                waypoint_distance = math.hypot(
                    waypoint[0] - robot_pose[0], waypoint[1] - robot_pose[1]
                )
                dock_relative = relative_pose(*robot_pose, *dock_pose)
                if now_ns - last_feedback_ns >= 200_000_000:
                    self._feedback(
                        goal_handle,
                        max(0.0, -dock_relative[0] - self.p['robot_rear_length']),
                        dock_relative[2],
                    )
                    last_feedback_ns = now_ns

                if state == DockingState.MAP_TURN:
                    heading = math.atan2(
                        waypoint[1] - robot_pose[1], waypoint[0] - robot_pose[0]
                    )
                    error = normalize_angle(heading - robot_pose[2])
                    if abs(error) <= math.radians(self.p['map_heading_tolerance_deg']):
                        self.publish_velocity(0.0, 0.0)
                        self.map_angular_pid.reset()
                        state = DockingState.MAP_DRIVE
                    else:
                        self.publish_velocity(0.0, self.map_angular_pid.update(error, dt))

                elif state == DockingState.MAP_DRIVE:
                    if waypoint_distance <= self.p['map_waypoint_tolerance']:
                        self.publish_velocity(0.0, 0.0)
                        self.map_linear_pid.reset()
                        state = DockingState.MAP_ALIGN
                    else:
                        heading = math.atan2(
                            waypoint[1] - robot_pose[1], waypoint[0] - robot_pose[0]
                        )
                        error = normalize_angle(heading - robot_pose[2])
                        if abs(error) > math.radians(20.0):
                            self.publish_velocity(0.0, 0.0)
                            state = DockingState.MAP_TURN
                        else:
                            self.publish_velocity(
                                self.map_linear_pid.update(waypoint_distance, dt),
                                float(np.clip(1.2 * error, -0.25, 0.25)),
                            )

                elif state in (DockingState.MAP_ALIGN, DockingState.RECOVERY_ALIGN):
                    error = normalize_angle(dock_pose[2] - robot_pose[2])
                    if abs(error) <= math.radians(1.0):
                        self.publish_velocity(0.0, 0.0)
                        self.current_transform = self._expected_transform(robot_pose, dock_pose)
                        self.settle_until_ns = now_ns + 500_000_000
                        self.last_valid_icp_ns = now_ns
                        self.angular_pid.reset()
                        state = DockingState.ICP_SETTLE
                    else:
                        self.publish_velocity(0.0, self.map_angular_pid.update(error, dt))

                elif state == DockingState.ICP_SETTLE:
                    self.publish_velocity(0.0, 0.0)
                    if now_ns >= self.settle_until_ns:
                        state = DockingState.ICP_ALIGN

                elif state in (DockingState.ICP_ALIGN, DockingState.ICP_REVERSE):
                    scan = self._take_scan_snapshot()
                    if scan is not None:
                        result, safe = self._safe_icp(scan, self.current_transform)
                        if safe:
                            self.current_transform = result.transform
                            self.last_valid_icp_ns = now_ns
                            alpha = 0.6 if result.fitness >= 0.6 else 0.35
                            raw_distance = float(result.transform[0, 2])
                            raw_lateral = float(result.transform[1, 2])
                            raw_yaw = math.atan2(
                                -result.transform[1, 0], result.transform[0, 0]
                            )
                            if self.filtered_distance is None:
                                self.filtered_distance = raw_distance
                                self.filtered_lateral = raw_lateral
                                self.filtered_yaw = raw_yaw
                            else:
                                self.filtered_distance = (
                                    alpha * raw_distance
                                    + (1.0 - alpha) * self.filtered_distance
                                )
                                self.filtered_lateral = (
                                    alpha * raw_lateral
                                    + (1.0 - alpha) * self.filtered_lateral
                                )
                                self.filtered_yaw = normalize_angle(
                                    self.filtered_yaw + alpha * normalize_angle(
                                        raw_yaw - self.filtered_yaw
                                    )
                                )
                        else:
                            self.publish_velocity(0.0, 0.0)
                    if (now_ns - self.last_valid_icp_ns) / 1e9 > self.p['icp_loss_timeout_sec']:
                        state = self._start_recovery(robot_pose, 'ICP lost or rejected')
                        if state is None:
                            goal_handle.abort()
                            return self._result(False, 'ICP recovery exhausted')
                        rate.sleep()
                        continue
                    if self.filtered_distance is None:
                        self.publish_velocity(0.0, 0.0)
                        rate.sleep()
                        continue

                    if state == DockingState.ICP_ALIGN:
                        if abs(self.filtered_yaw) <= math.radians(1.5):
                            self.publish_velocity(0.0, 0.0)
                            distance_error = max(
                                0.0, self.filtered_distance - self.p['robot_rear_length']
                            )
                            self.linear_pid.reset()
                            self.linear_pid.previous_error = distance_error
                            state = DockingState.ICP_REVERSE
                        else:
                            self.publish_velocity(
                                0.0, self.angular_pid.update(self.filtered_yaw, dt)
                            )
                    else:
                        distance_error = max(
                            0.0, self.filtered_distance - self.p['robot_rear_length']
                        )
                        if self.filtered_distance <= self.p['final_blind_start_dist']:
                            self.publish_velocity(0.0, 0.0)
                            self.final_start_pose = robot_pose
                            self.final_distance = distance_error
                            state = (
                                DockingState.COMPLETED
                                if self.final_distance <= self.p['final_reverse_tolerance']
                                else DockingState.FINAL_REVERSE
                            )
                        elif distance_error <= 0.015:
                            self.publish_velocity(0.0, 0.0)
                            state = DockingState.COMPLETED
                        else:
                            target_yaw = float(np.clip(
                                self.p['reverse_lateral_gain'] * self.filtered_lateral,
                                -math.radians(self.p['reverse_max_target_yaw_deg']),
                                math.radians(self.p['reverse_max_target_yaw_deg']),
                            ))
                            steering_scale = float(np.clip(
                                (self.filtered_distance - self.p['robot_rear_length'])
                                / max(
                                    0.01, self.p['steering_lock_dist']
                                    - self.p['robot_rear_length']
                                ), 0.0, 1.0,
                            ))
                            self.publish_velocity(
                                -self.linear_pid.update(distance_error, dt),
                                steering_scale * self.angular_pid.update(
                                    normalize_angle(self.filtered_yaw + target_yaw), dt
                                ),
                            )

                elif state == DockingState.FINAL_REVERSE:
                    moved = math.hypot(
                        robot_pose[0] - self.final_start_pose[0],
                        robot_pose[1] - self.final_start_pose[1],
                    )
                    if moved + self.p['final_reverse_tolerance'] >= self.final_distance:
                        self.publish_velocity(0.0, 0.0)
                        state = DockingState.COMPLETED
                    else:
                        yaw_error = normalize_angle(
                            self.final_start_pose[2] - robot_pose[2]
                        )
                        self.publish_velocity(
                            -self.p['final_reverse_speed'],
                            float(np.clip(
                                self.p['final_yaw_hold_kp'] * yaw_error,
                                -self.p['final_yaw_hold_max_w'],
                                self.p['final_yaw_hold_max_w'],
                            )),
                        )

                elif state == DockingState.RECOVERY_RETREAT:
                    if not self._recovery_path_is_clear():
                        self.publish_velocity(0.0, 0.0)
                        goal_handle.abort()
                        return self._result(
                            False,
                            'recovery stopped: forward scan is stale or obstructed',
                        )
                    moved = math.hypot(
                        robot_pose[0] - self.recovery_start_pose[0],
                        robot_pose[1] - self.recovery_start_pose[1],
                    )
                    if moved >= self.p['recovery_distance']:
                        self.publish_velocity(0.0, 0.0)
                        self.filtered_distance = None
                        self.filtered_lateral = None
                        self.filtered_yaw = None
                        self.map_angular_pid.reset()
                        state = DockingState.RECOVERY_ALIGN
                    else:
                        self.publish_velocity(self.p['recovery_speed'], 0.0)

                if state == DockingState.COMPLETED:
                    self.publish_velocity(0.0, 0.0)
                    goal_handle.succeed()
                    return self._result(True, 'docking completed')
                rate.sleep()
        finally:
            self.publish_velocity(0.0, 0.0)


def main(args=None):
    """Run the docking server with concurrent scan and action callbacks."""
    rp.init(args=args)
    node = PrecisionDockingServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()
