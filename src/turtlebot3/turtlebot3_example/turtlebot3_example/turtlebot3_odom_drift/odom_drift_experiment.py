#!/usr/bin/env python3
#
# Copyright 2026
#
# Licensed under the Apache License, Version 2.0

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import sys

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


ros_distro = os.environ.get('ROS_DISTRO', 'humble').lower()
if ros_distro == 'humble':
    from geometry_msgs.msg import Twist as CmdVelMsg
else:
    from geometry_msgs.msg import TwistStamped as CmdVelMsg


@dataclass
class Pose2D:
    stamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass
class Phase:
    name: str
    duration_sec: float
    linear_x: float = 0.0
    angular_z: float = 0.0


class OdomDriftExperiment(Node):

    def __init__(self, args):
        super().__init__('turtlebot3_odom_drift_experiment')

        self.args = args
        self.odom_frame = args.odom_frame
        self.base_frame = args.base_frame
        self.latest_pose = None
        self.start_pose = None
        self.end_pose = None
        self.samples = []
        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = self.odom_frame

        self.phases = self.make_phases(args)
        self.phase_index = 0
        self.phase_started_at = None
        self.started_at = None
        self.finished = False
        self.result_written = False

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(CmdVelMsg, args.cmd_vel_topic, qos)
        self.path_pub = self.create_publisher(PathMsg, args.path_topic, qos)
        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            args.odom_topic,
            self.odom_callback,
            qos)

        timer_period = 1.0 / args.control_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Experiment "{args.experiment}" is ready. Waiting for {args.odom_topic}...')
        self.get_logger().info(
            f'Path topic: {args.path_topic}, marker topic: {args.marker_topic}')

    def make_phases(self, args):
        move_time = safe_divide(args.distance, args.linear_speed)
        turn_90_time = safe_divide(math.pi / 2.0, args.angular_speed)
        turn_360_time = safe_divide(2.0 * math.pi, args.angular_speed)

        rest = args.rest_sec
        phases = []

        if args.experiment == 'straight':
            phases = [
                Phase('forward', move_time, linear_x=args.linear_speed),
                Phase('stop', rest),
            ]
        elif args.experiment == 'round_trip':
            phases = [
                Phase('forward', move_time, linear_x=args.linear_speed),
                Phase('stop_after_forward', rest),
                Phase('backward', move_time, linear_x=-args.linear_speed),
                Phase('stop_after_backward', rest),
            ]
        elif args.experiment == 'rotate':
            direction = 1.0 if args.turn_direction == 'ccw' else -1.0
            phases = [
                Phase(
                    f'rotate_360_{args.turn_direction}',
                    turn_360_time,
                    angular_z=direction * args.angular_speed),
                Phase('stop_after_rotate', rest),
            ]
        elif args.experiment == 'square':
            direction = 1.0 if args.turn_direction == 'ccw' else -1.0
            for side in range(1, 5):
                phases.extend([
                    Phase(f'side_{side}_forward', move_time, linear_x=args.linear_speed),
                    Phase(f'side_{side}_stop_after_forward', rest),
                    Phase(
                        f'side_{side}_turn_90_{args.turn_direction}',
                        turn_90_time,
                        angular_z=direction * args.angular_speed),
                    Phase(f'side_{side}_stop_after_turn', rest),
                ])
        else:
            raise ValueError(f'Unsupported experiment: {args.experiment}')

        return phases

    def odom_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = Pose2D(stamp, position.x, position.y, yaw)

        if self.started_at is None:
            return

        elapsed = self.get_elapsed_time()
        phase_name = self.current_phase_name()
        self.samples.append([
            f'{elapsed:.4f}',
            phase_name,
            f'{position.x:.6f}',
            f'{position.y:.6f}',
            f'{math.degrees(yaw):.6f}',
        ])

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.odom_frame
        pose.pose = msg.pose.pose
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > self.args.max_path_points:
            self.path_msg.poses = self.path_msg.poses[-self.args.max_path_points:]

        self.path_pub.publish(self.path_msg)
        self.publish_markers()

    def timer_callback(self):
        if self.finished:
            self.publish_stop()
            if not self.result_written:
                self.end_pose = self.latest_pose
                self.write_results()
                self.result_written = True
                self.get_logger().info('Experiment finished. Result files were written.')
                if self.args.shutdown_on_finish:
                    rclpy.shutdown()
            return

        if self.latest_pose is None:
            self.publish_stop()
            return

        now = self.get_clock().now()
        if self.started_at is None:
            self.started_at = now
            self.phase_started_at = now
            self.start_pose = self.latest_pose
            self.get_logger().info(
                f'Starting experiment "{self.args.experiment}" with {len(self.phases)} phases.')

        phase = self.phases[self.phase_index]
        phase_elapsed = seconds_between(self.phase_started_at, now)

        if phase_elapsed >= phase.duration_sec:
            self.get_logger().info(f'Completed phase: {phase.name}')
            self.phase_index += 1
            self.phase_started_at = now

            if self.phase_index >= len(self.phases):
                self.finished = True
                self.publish_stop()
                return

            phase = self.phases[self.phase_index]
            self.get_logger().info(f'Starting phase: {phase.name}')

        self.publish_velocity(phase.linear_x, phase.angular_z)

    def publish_velocity(self, linear_x, angular_z):
        if ros_distro == 'humble':
            msg = CmdVelMsg()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.cmd_vel_pub.publish(msg)
            return

        msg = CmdVelMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0)

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if self.start_pose is not None:
            markers.markers.append(
                self.make_sphere_marker(
                    0,
                    'start',
                    stamp,
                    self.start_pose.x,
                    self.start_pose.y,
                    0.0,
                    0.0,
                    0.8,
                    0.1))

        current_pose = self.end_pose if self.end_pose is not None else self.latest_pose
        if current_pose is not None:
            markers.markers.append(
                self.make_sphere_marker(
                    1,
                    'end',
                    stamp,
                    current_pose.x,
                    current_pose.y,
                    0.9,
                    0.1,
                    0.1,
                    0.0))

        if self.start_pose is not None and current_pose is not None:
            markers.markers.append(
                self.make_error_line_marker(2, stamp, self.start_pose, current_pose))

        self.marker_pub.publish(markers)

    def make_sphere_marker(self, marker_id, namespace, stamp, x, y, r, g, b, z):
        marker = Marker()
        marker.header.frame_id = self.odom_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.args.marker_scale
        marker.scale.y = self.args.marker_scale
        marker.scale.z = self.args.marker_scale
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        return marker

    def make_error_line_marker(self, marker_id, stamp, start, end):
        marker = Marker()
        marker.header.frame_id = self.odom_frame
        marker.header.stamp = stamp
        marker.ns = 'position_error'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.args.error_line_width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.0

        p0 = Point()
        p0.x = start.x
        p0.y = start.y
        p0.z = 0.05
        p1 = Point()
        p1.x = end.x
        p1.y = end.y
        p1.z = 0.05
        marker.points = [p0, p1]
        return marker

    def write_results(self):
        if self.start_pose is None or self.end_pose is None:
            self.get_logger().error('Cannot write results because start or end pose is missing.')
            return

        output_dir = Path(os.path.expanduser(self.args.output_dir))
        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        stem = f'{timestamp}_{self.args.experiment}'
        summary_path = output_dir / f'{stem}_summary.csv'
        trace_path = output_dir / f'{stem}_odom_trace.csv'

        position_error = math.hypot(
            self.end_pose.x - self.start_pose.x,
            self.end_pose.y - self.start_pose.y)
        yaw_error = normalize_angle(self.end_pose.yaw - self.start_pose.yaw)
        total_distance = self.estimated_travel_distance()
        drift_rate = position_error / total_distance if total_distance > 0.0 else None

        with summary_path.open('w', newline='') as summary_file:
            writer = csv.writer(summary_file)
            writer.writerow([
                'experiment',
                'distance_m',
                'linear_speed_mps',
                'angular_speed_radps',
                'turn_direction',
                'start_x_m',
                'start_y_m',
                'start_yaw_deg',
                'end_x_m',
                'end_y_m',
                'end_yaw_deg',
                'position_error_m',
                'yaw_error_deg',
                'estimated_travel_distance_m',
                'drift_rate',
                'sample_count',
            ])
            writer.writerow([
                self.args.experiment,
                f'{self.args.distance:.6f}',
                f'{self.args.linear_speed:.6f}',
                f'{self.args.angular_speed:.6f}',
                self.args.turn_direction,
                f'{self.start_pose.x:.6f}',
                f'{self.start_pose.y:.6f}',
                f'{math.degrees(self.start_pose.yaw):.6f}',
                f'{self.end_pose.x:.6f}',
                f'{self.end_pose.y:.6f}',
                f'{math.degrees(self.end_pose.yaw):.6f}',
                f'{position_error:.6f}',
                f'{math.degrees(yaw_error):.6f}',
                f'{total_distance:.6f}',
                '' if drift_rate is None else f'{drift_rate:.6f}',
                len(self.samples),
            ])

        with trace_path.open('w', newline='') as trace_file:
            writer = csv.writer(trace_file)
            writer.writerow(['elapsed_sec', 'phase', 'x_m', 'y_m', 'yaw_deg'])
            writer.writerows(self.samples)

        self.get_logger().info(f'Summary CSV: {summary_path}')
        self.get_logger().info(f'Odom trace CSV: {trace_path}')
        self.get_logger().info(
            f'position_error={position_error:.3f} m, '
            f'yaw_error={math.degrees(yaw_error):.2f} deg')

    def estimated_travel_distance(self):
        if self.args.experiment == 'straight':
            return self.args.distance
        if self.args.experiment == 'round_trip':
            return self.args.distance * 2.0
        if self.args.experiment == 'square':
            return self.args.distance * 4.0
        return 0.0

    def current_phase_name(self):
        if self.phase_index >= len(self.phases):
            return 'finished'
        return self.phases[self.phase_index].name

    def get_elapsed_time(self):
        if self.started_at is None:
            return 0.0
        return seconds_between(self.started_at, self.get_clock().now())


def safe_divide(numerator, denominator):
    if denominator <= 0.0:
        raise ValueError('Velocity values must be greater than 0.')
    return numerator / denominator


def seconds_between(start_time, end_time):
    return (end_time.nanoseconds - start_time.nanoseconds) * 1e-9


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Automate TurtleBot3 odometry drift experiments.')
    parser.add_argument(
        '--experiment',
        choices=['straight', 'round_trip', 'rotate', 'square'],
        default='round_trip',
        help='Experiment pattern to execute.')
    parser.add_argument(
        '--distance',
        type=float,
        default=1.0,
        help='Forward distance per leg in meters.')
    parser.add_argument(
        '--linear-speed',
        type=float,
        default=0.1,
        help='Linear speed in m/s.')
    parser.add_argument(
        '--angular-speed',
        type=float,
        default=0.3,
        help='Angular speed in rad/s.')
    parser.add_argument(
        '--turn-direction',
        choices=['ccw', 'cw'],
        default='ccw',
        help='Turn direction for rotate and square experiments.')
    parser.add_argument(
        '--rest-sec',
        type=float,
        default=1.0,
        help='Stop duration between motion phases.')
    parser.add_argument(
        '--control-rate',
        type=float,
        default=20.0,
        help='Command publishing rate in Hz.')
    parser.add_argument('--odom-topic', default='odom')
    parser.add_argument('--cmd-vel-topic', default='cmd_vel')
    parser.add_argument('--path-topic', default='odom_drift_path')
    parser.add_argument('--marker-topic', default='odom_drift_markers')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument(
        '--output-dir',
        default='~/turtlebot3_odom_drift_results',
        help='Directory where CSV result files are written.')
    parser.add_argument(
        '--max-path-points',
        type=int,
        default=20000,
        help='Maximum path poses kept for RViz visualization.')
    parser.add_argument('--marker-scale', type=float, default=0.12)
    parser.add_argument('--error-line-width', type=float, default=0.025)
    parser.add_argument(
        '--no-shutdown-on-finish',
        action='store_false',
        dest='shutdown_on_finish',
        help='Keep the node alive after writing results.')
    parser.set_defaults(shutdown_on_finish=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = OdomDriftExperiment(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
