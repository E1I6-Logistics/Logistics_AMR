#!/usr/bin/env python3

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
        super().__init__(
            f'turtlebot3_odom_drift_experiment_2_{args.condition}'
    )

        self.args = args

        # 실제 /odom frame과 분리된 실험 비교용 frame
        self.comparison_frame = args.comparison_frame

        self.latest_pose = None
        self.start_pose = None
        self.end_pose = None

        self.samples = []

        # 시작점을 (0,0,0)으로 정규화한 Path
        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = self.comparison_frame

        self.phases = self.make_phases(args)

        self.phase_index = 0
        self.phase_started_at = None
        self.started_at = None

        self.finished = False
        self.result_written = False

        self.last_hold_publish_time = None

        qos = QoSProfile(depth=10)

        # --------------------------------------------------
        # cmd_vel publisher
        # --------------------------------------------------

        self.cmd_vel_pub = self.create_publisher(
            CmdVelMsg,
            args.cmd_vel_topic,
            qos,
        )

        # --------------------------------------------------
        # condition별 Path topic
        #
        # baseline:
        # /odom_drift/baseline_path
        #
        # ekf:
        # /odom_drift/ekf_path
        # --------------------------------------------------

        self.path_topic = (
            f'/odom_drift/{args.condition}_path'
        )

        self.marker_topic = (
            f'/odom_drift/{args.condition}_markers'
        )

        self.path_pub = self.create_publisher(
            PathMsg,
            self.path_topic,
            qos,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            qos,
        )

        # --------------------------------------------------
        # 핵심:
        # 두 workspace 모두 /odom 하나만 구독
        # --------------------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            args.odom_topic,
            self.odom_callback,
            qos,
        )

        timer_period = 1.0 / args.control_rate

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
        )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            f'Condition : {args.condition}'
        )

        self.get_logger().info(
            f'Trial     : {args.trial}'
        )

        self.get_logger().info(
            f'Experiment: {args.experiment}'
        )

        self.get_logger().info(
            f'Odom topic: {args.odom_topic}'
        )

        self.get_logger().info(
            f'Path topic: {self.path_topic}'
        )

        self.get_logger().info(
            'Waiting for odometry...'
        )

        self.get_logger().info(
            '======================================'
        )

    # ======================================================
    # Motion phases
    # ======================================================

    def make_phases(self, args):

        rest = args.rest_sec

        turn_90_time = safe_divide(
            math.pi / 2.0,
            args.angular_speed,
        )

        turn_360_time = safe_divide(
            2.0 * math.pi,
            args.angular_speed,
        )

        direction = (
            1.0
            if args.turn_direction == 'ccw'
            else -1.0
        )

        phases = []

        # --------------------------------------------------
        # Straight
        # --------------------------------------------------

        if args.experiment == 'straight':

            move_time = safe_divide(
                args.distance,
                args.linear_speed,
            )

            phases = [
                Phase(
                    'forward',
                    move_time,
                    linear_x=args.linear_speed,
                ),

                Phase(
                    'stop',
                    rest,
                ),
            ]

        # --------------------------------------------------
        # Round trip
        # --------------------------------------------------

        elif args.experiment == 'round_trip':

            move_time = safe_divide(
                args.distance,
                args.linear_speed,
            )

            phases = [
                Phase(
                    'forward',
                    move_time,
                    linear_x=args.linear_speed,
                ),

                Phase(
                    'stop_after_forward',
                    rest,
                ),

                Phase(
                    'backward',
                    move_time,
                    linear_x=-args.linear_speed,
                ),

                Phase(
                    'stop_after_backward',
                    rest,
                ),
            ]

        # --------------------------------------------------
        # Rotate 360
        # --------------------------------------------------

        elif args.experiment == 'rotate':

            phases = [
                Phase(
                    f'rotate_360_{args.turn_direction}',
                    turn_360_time,
                    angular_z=(
                        direction
                        * args.angular_speed
                    ),
                ),

                Phase(
                    'stop_after_rotate',
                    rest,
                ),
            ]

        # --------------------------------------------------
        # Square
        # --------------------------------------------------

        elif args.experiment == 'square':

            move_time = safe_divide(
                args.distance,
                args.linear_speed,
            )

            for side in range(1, 5):

                phases.extend([
                    Phase(
                        f'side_{side}_forward',
                        move_time,
                        linear_x=args.linear_speed,
                    ),

                    Phase(
                        f'side_{side}_stop',
                        rest,
                    ),

                    Phase(
                        f'side_{side}_turn_90',
                        turn_90_time,
                        angular_z=(
                            direction
                            * args.angular_speed
                        ),
                    ),

                    Phase(
                        f'side_{side}_turn_stop',
                        rest,
                    ),
                ])

        # --------------------------------------------------
        # Rectangle
        #
        # long
        # short
        # long
        # short
        # --------------------------------------------------

        elif args.experiment == 'rectangle':

            long_time = safe_divide(
                args.long_distance,
                args.linear_speed,
            )

            short_time = safe_divide(
                args.short_distance,
                args.linear_speed,
            )

            sides = [
                ('long_1', long_time),
                ('short_1', short_time),
                ('long_2', long_time),
                ('short_2', short_time),
            ]

            for name, move_time in sides:

                phases.extend([
                    Phase(
                        f'{name}_forward',
                        move_time,
                        linear_x=args.linear_speed,
                    ),

                    Phase(
                        f'{name}_stop',
                        rest,
                    ),

                    Phase(
                        f'{name}_turn_90',
                        turn_90_time,
                        angular_z=(
                            direction
                            * args.angular_speed
                        ),
                    ),

                    Phase(
                        f'{name}_turn_stop',
                        rest,
                    ),
                ])

        else:

            raise ValueError(
                f'Unsupported experiment: '
                f'{args.experiment}'
            )

        return phases

    # ======================================================
    # /odom callback
    # ======================================================

    def odom_callback(self, msg):

        pose = self.pose_from_odom(msg)

        self.latest_pose = pose

        # 아직 실험 시작 전
        if self.started_at is None:
            return

        # 실험 종료 후에는 새로운 데이터를 기록하지 않음
        if self.finished:
            return

        relative_pose = self.to_relative_pose(
            pose
        )

        elapsed = self.get_elapsed_time()

        phase_name = self.current_phase_name()

        # --------------------------------------------------
        # CSV
        # --------------------------------------------------

        self.samples.append([
            f'{elapsed:.4f}',
            phase_name,

            # 실제 /odom 값
            f'{pose.x:.6f}',
            f'{pose.y:.6f}',
            f'{math.degrees(pose.yaw):.6f}',

            # 시작점 기준 정규화 값
            f'{relative_pose.x:.6f}',
            f'{relative_pose.y:.6f}',
            f'{math.degrees(relative_pose.yaw):.6f}',
        ])

        # --------------------------------------------------
        # RViz Path
        # --------------------------------------------------

        path_pose = PoseStamped()

        path_pose.header.stamp = msg.header.stamp
        path_pose.header.frame_id = (
            self.comparison_frame
        )

        path_pose.pose.position.x = (
            relative_pose.x
        )

        path_pose.pose.position.y = (
            relative_pose.y
        )

        path_pose.pose.position.z = 0.0

        path_pose.pose.orientation.z = math.sin(
            relative_pose.yaw / 2.0
        )

        path_pose.pose.orientation.w = math.cos(
            relative_pose.yaw / 2.0
        )

        self.path_msg.header.stamp = (
            msg.header.stamp
        )

        self.path_msg.poses.append(
            path_pose
        )

        if (
            len(self.path_msg.poses)
            > self.args.max_path_points
        ):

            self.path_msg.poses = (
                self.path_msg.poses[
                    -self.args.max_path_points:
                ]
            )

        self.path_pub.publish(
            self.path_msg
        )

        self.publish_markers(
            relative_pose
        )

    # ======================================================
    # Absolute /odom → Pose2D
    # ======================================================

    def pose_from_odom(self, msg):

        stamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec
            * 1e-9
        )

        p = msg.pose.pose.position

        yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        return Pose2D(
            stamp,
            p.x,
            p.y,
            yaw,
        )

    # ======================================================
    # 시작점 기준 좌표로 변환
    #
    # start = (0,0,0)
    # ======================================================

    def to_relative_pose(self, pose):

        if self.start_pose is None:

            return Pose2D(
                pose.stamp_sec,
                0.0,
                0.0,
                0.0,
            )

        dx = (
            pose.x
            - self.start_pose.x
        )

        dy = (
            pose.y
            - self.start_pose.y
        )

        yaw0 = self.start_pose.yaw

        cos_yaw = math.cos(yaw0)
        sin_yaw = math.sin(yaw0)

        relative_x = (
            cos_yaw * dx
            + sin_yaw * dy
        )

        relative_y = (
            -sin_yaw * dx
            + cos_yaw * dy
        )

        relative_yaw = normalize_angle(
            pose.yaw
            - self.start_pose.yaw
        )

        return Pose2D(
            pose.stamp_sec,
            relative_x,
            relative_y,
            relative_yaw,
        )

    # ======================================================
    # Timer
    # ======================================================

    def timer_callback(self):

        # --------------------------------------------------
        # 실험 종료
        # --------------------------------------------------

        if self.finished:

            if not self.result_written:

                self.end_pose = (
                    self.latest_pose
                )

                self.publish_stop()

                self.write_results()

                self.result_written = True

                self.get_logger().info(
                    'Experiment finished.'
                )

                # 바로 종료
                if self.args.shutdown_on_finish:

                    rclpy.shutdown()

                    return

            # ----------------------------------------------
            # --no-shutdown-on-finish 인 경우
            #
            # cmd_vel은 더 이상 publish하지 않고
            # RViz Path만 유지
            # ----------------------------------------------

            self.hold_visualization()

            return

        # --------------------------------------------------
        # /odom이 아직 없으면 정지
        # --------------------------------------------------

        if self.latest_pose is None:

            self.publish_stop()

            return

        now = self.get_clock().now()

        # --------------------------------------------------
        # 실험 시작
        # --------------------------------------------------

        if self.started_at is None:

            self.started_at = now

            self.phase_started_at = now

            self.start_pose = (
                self.latest_pose
            )

            self.get_logger().info(
                '--------------------------------'
            )

            self.get_logger().info(
                f'Start condition = '
                f'{self.args.condition}'
            )

            self.get_logger().info(
                f'Start pose = '
                f'({self.start_pose.x:.3f}, '
                f'{self.start_pose.y:.3f}, '
                f'{math.degrees(self.start_pose.yaw):.2f} deg)'
            )

            self.get_logger().info(
                '--------------------------------'
            )

        phase = self.phases[
            self.phase_index
        ]

        phase_elapsed = seconds_between(
            self.phase_started_at,
            now,
        )

        # --------------------------------------------------
        # Phase 완료
        # --------------------------------------------------

        if phase_elapsed >= phase.duration_sec:

            self.get_logger().info(
                f'Completed phase: '
                f'{phase.name}'
            )

            self.phase_index += 1

            self.phase_started_at = now

            if (
                self.phase_index
                >= len(self.phases)
            ):

                self.finished = True

                self.publish_stop()

                return

            phase = self.phases[
                self.phase_index
            ]

            self.get_logger().info(
                f'Starting phase: '
                f'{phase.name}'
            )

        self.publish_velocity(
            phase.linear_x,
            phase.angular_z,
        )

    # ======================================================
    # cmd_vel
    # ======================================================

    def publish_velocity(
        self,
        linear_x,
        angular_z,
    ):

        if ros_distro == 'humble':

            msg = CmdVelMsg()

            msg.linear.x = linear_x
            msg.angular.z = angular_z

            self.cmd_vel_pub.publish(msg)

            return

        msg = CmdVelMsg()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z

        self.cmd_vel_pub.publish(msg)

    def publish_stop(self):

        self.publish_velocity(
            0.0,
            0.0,
        )

    # ======================================================
    # RViz Marker
    # ======================================================

    def publish_markers(
        self,
        relative_pose,
    ):

        markers = MarkerArray()

        stamp = (
            self.get_clock().now().to_msg()
        )

        # 시작점
        start_marker = self.make_sphere_marker(
            marker_id=0,
            namespace=(
                f'{self.args.condition}_start'
            ),
            stamp=stamp,
            x=0.0,
            y=0.0,
            z=0.05,
        )

        markers.markers.append(
            start_marker
        )

        # 현재 위치
        current_marker = self.make_sphere_marker(
            marker_id=1,
            namespace=(
                f'{self.args.condition}_current'
            ),
            stamp=stamp,
            x=relative_pose.x,
            y=relative_pose.y,
            z=0.08,
        )

        markers.markers.append(
            current_marker
        )

        # 시작점 → 현재점 오차선
        error_marker = (
            self.make_error_line_marker(
                marker_id=2,
                namespace=(
                    f'{self.args.condition}_error'
                ),
                stamp=stamp,
                x=relative_pose.x,
                y=relative_pose.y,
            )
        )

        markers.markers.append(
            error_marker
        )

        self.marker_pub.publish(
            markers
        )

    def make_sphere_marker(
        self,
        marker_id,
        namespace,
        stamp,
        x,
        y,
        z,
    ):

        marker = Marker()

        marker.header.frame_id = (
            self.comparison_frame
        )

        marker.header.stamp = stamp

        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.w = 1.0

        marker.scale.x = (
            self.args.marker_scale
        )

        marker.scale.y = (
            self.args.marker_scale
        )

        marker.scale.z = (
            self.args.marker_scale
        )

        marker.color.a = 1.0
        marker.color.r = 0.9
        marker.color.g = 0.5
        marker.color.b = 0.1

        return marker

    def make_error_line_marker(
        self,
        marker_id,
        namespace,
        stamp,
        x,
        y,
    ):

        marker = Marker()

        marker.header.frame_id = (
            self.comparison_frame
        )

        marker.header.stamp = stamp

        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = (
            self.args.error_line_width
        )

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.0

        p0 = Point()

        p0.x = 0.0
        p0.y = 0.0
        p0.z = 0.04

        p1 = Point()

        p1.x = x
        p1.y = y
        p1.z = 0.04

        marker.points = [
            p0,
            p1,
        ]

        return marker

    # ======================================================
    # RViz Path 유지
    # ======================================================

    def hold_visualization(self):

        if len(self.path_msg.poses) == 0:
            return

        now = self.get_clock().now()

        if self.last_hold_publish_time is None:

            self.last_hold_publish_time = now

            self.path_pub.publish(
                self.path_msg
            )

            return

        elapsed = seconds_between(
            self.last_hold_publish_time,
            now,
        )

        # 1초마다 재발행
        if elapsed >= 1.0:

            self.path_pub.publish(
                self.path_msg
            )

            if self.end_pose is not None:

                end_relative = (
                    self.to_relative_pose(
                        self.end_pose
                    )
                )

                self.publish_markers(
                    end_relative
                )

            self.last_hold_publish_time = now

    # ======================================================
    # 결과 저장
    # ======================================================

    def write_results(self):

        if (
            self.start_pose is None
            or self.end_pose is None
        ):

            self.get_logger().error(
                'Start/end pose missing.'
            )

            return

        output_dir = Path(
            os.path.expanduser(
                self.args.output_dir
            )
        )

        output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )

        stem = (
            f'{timestamp}_'
            f'{self.args.condition}_'
            f'trial{self.args.trial}_'
            f'{self.args.experiment}'
        )

        summary_path = (
            output_dir
            / f'{stem}_summary.csv'
        )

        trace_path = (
            output_dir
            / f'{stem}_trace.csv'
        )

        end_relative = (
            self.to_relative_pose(
                self.end_pose
            )
        )

        position_error = math.hypot(
            end_relative.x,
            end_relative.y,
        )

        yaw_error_deg = math.degrees(
            end_relative.yaw
        )

        yaw_error_abs_deg = abs(
            yaw_error_deg
        )

        total_distance = (
            self.estimated_travel_distance()
        )

        if total_distance > 0.0:

            drift_rate = (
                position_error
                / total_distance
            )

            drift_percent = (
                drift_rate
                * 100.0
            )

        else:

            drift_rate = None
            drift_percent = None

        # --------------------------------------------------
        # Summary
        # --------------------------------------------------

        with summary_path.open(
            'w',
            newline='',
        ) as summary_file:

            writer = csv.writer(
                summary_file
            )

            writer.writerow([
                'condition',
                'trial',
                'experiment',
                'odom_topic',

                'distance_m',
                'long_distance_m',
                'short_distance_m',

                'linear_speed_mps',
                'angular_speed_radps',
                'turn_direction',

                'position_error_m',
                'yaw_error_deg',
                'yaw_error_abs_deg',

                'estimated_travel_distance_m',

                'drift_rate',
                'drift_percent',

                'sample_count',
            ])

            writer.writerow([
                self.args.condition,
                self.args.trial,
                self.args.experiment,
                self.args.odom_topic,

                f'{self.args.distance:.6f}',
                f'{self.args.long_distance:.6f}',
                f'{self.args.short_distance:.6f}',

                f'{self.args.linear_speed:.6f}',
                f'{self.args.angular_speed:.6f}',
                self.args.turn_direction,

                f'{position_error:.6f}',
                f'{yaw_error_deg:.6f}',
                f'{yaw_error_abs_deg:.6f}',

                f'{total_distance:.6f}',

                (
                    ''
                    if drift_rate is None
                    else f'{drift_rate:.6f}'
                ),

                (
                    ''
                    if drift_percent is None
                    else f'{drift_percent:.6f}'
                ),

                len(self.samples),
            ])

        # --------------------------------------------------
        # Trace
        # --------------------------------------------------

        with trace_path.open(
            'w',
            newline='',
        ) as trace_file:

            writer = csv.writer(
                trace_file
            )

            writer.writerow([
                'elapsed_sec',
                'phase',

                'odom_x_m',
                'odom_y_m',
                'odom_yaw_deg',

                'relative_x_m',
                'relative_y_m',
                'relative_yaw_deg',
            ])

            writer.writerows(
                self.samples
            )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            f'Condition = {self.args.condition}'
        )

        self.get_logger().info(
            f'Position error = '
            f'{position_error:.3f} m'
        )

        self.get_logger().info(
            f'Yaw error = '
            f'{yaw_error_abs_deg:.2f} deg'
        )

        if drift_percent is not None:

            self.get_logger().info(
                f'Drift = '
                f'{drift_percent:.2f} %'
            )

        self.get_logger().info(
            f'Summary: {summary_path}'
        )

        self.get_logger().info(
            f'Trace: {trace_path}'
        )

        self.get_logger().info(
            '======================================'
        )

    # ======================================================
    # 이동거리
    # ======================================================

    def estimated_travel_distance(self):

        if self.args.experiment == 'straight':

            return self.args.distance

        if self.args.experiment == 'round_trip':

            return (
                self.args.distance
                * 2.0
            )

        if self.args.experiment == 'square':

            return (
                self.args.distance
                * 4.0
            )

        if self.args.experiment == 'rectangle':

            return (
                2.0
                * (
                    self.args.long_distance
                    + self.args.short_distance
                )
            )

        return 0.0

    def current_phase_name(self):

        if (
            self.phase_index
            >= len(self.phases)
        ):

            return 'finished'

        return self.phases[
            self.phase_index
        ].name

    def get_elapsed_time(self):

        if self.started_at is None:
            return 0.0

        return seconds_between(
            self.started_at,
            self.get_clock().now(),
        )


# ==========================================================
# Utility
# ==========================================================

def safe_divide(
    numerator,
    denominator,
):

    if denominator <= 0.0:

        raise ValueError(
            'Velocity must be > 0.'
        )

    return numerator / denominator


def seconds_between(
    start_time,
    end_time,
):

    return (
        end_time.nanoseconds
        - start_time.nanoseconds
    ) * 1e-9


def yaw_from_quaternion(q):

    siny_cosp = (
        2.0
        * (
            q.w * q.z
            + q.x * q.y
        )
    )

    cosy_cosp = (
        1.0
        - 2.0
        * (
            q.y * q.y
            + q.z * q.z
        )
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


def normalize_angle(angle):

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


# ==========================================================
# Arguments
# ==========================================================

def parse_args(argv):

    parser = argparse.ArgumentParser(
        description=(
            'TurtleBot3 odometry drift experiment '
            'for baseline vs EKF comparison.'
        )
    )

    parser.add_argument(
        '--condition',
        choices=[
            'baseline',
            'ekf',
        ],
        required=True,
        help=(
            'baseline=turtlebot3_ws, '
            'ekf=logitle_ws'
        ),
    )

    parser.add_argument(
        '--trial',
        type=int,
        default=1,
    )

    parser.add_argument(
        '--experiment',
        choices=[
            'straight',
            'round_trip',
            'rotate',
            'square',
            'rectangle',
        ],
        default='rectangle',
    )

    parser.add_argument(
        '--distance',
        type=float,
        default=1.0,
    )

    parser.add_argument(
        '--long-distance',
        type=float,
        default=2.0,
    )

    parser.add_argument(
        '--short-distance',
        type=float,
        default=1.0,
    )

    parser.add_argument(
        '--linear-speed',
        type=float,
        default=0.1,
    )

    parser.add_argument(
        '--angular-speed',
        type=float,
        default=0.3,
    )

    parser.add_argument(
        '--turn-direction',
        choices=[
            'ccw',
            'cw',
        ],
        default='ccw',
    )

    parser.add_argument(
        '--rest-sec',
        type=float,
        default=1.0,
    )

    parser.add_argument(
        '--control-rate',
        type=float,
        default=20.0,
    )

    # 두 workspace 모두 /odom
    parser.add_argument(
        '--odom-topic',
        default='/odom',
    )

    parser.add_argument(
        '--cmd-vel-topic',
        default='/cmd_vel',
    )

    # RViz 비교 전용 frame
    parser.add_argument(
        '--comparison-frame',
        default='odom_experiment',
    )

    parser.add_argument(
        '--output-dir',
        default='~/turtlebot3_odom_drift_results',
    )

    parser.add_argument(
        '--max-path-points',
        type=int,
        default=20000,
    )

    parser.add_argument(
        '--marker-scale',
        type=float,
        default=0.12,
    )

    parser.add_argument(
        '--error-line-width',
        type=float,
        default=0.025,
    )

    parser.add_argument(
        '--no-shutdown-on-finish',
        action='store_false',
        dest='shutdown_on_finish',
        help=(
            'Keep node alive after experiment '
            'so RViz can continue displaying Path.'
        ),
    )

    parser.set_defaults(
        shutdown_on_finish=True
    )

    return parser.parse_args(argv)


# ==========================================================
# Main
# ==========================================================

def main(argv=None):

    args = parse_args(
        sys.argv[1:]
        if argv is None
        else argv
    )

    rclpy.init()

    node = OdomDriftExperiment(
        args
    )

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Keyboard Interrupt'
        )

    finally:

        if rclpy.ok():

            # 아직 주행 중 종료한 경우에만 stop
            if not node.finished:
                node.publish_stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()