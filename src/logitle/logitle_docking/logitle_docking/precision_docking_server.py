import math
from dataclasses import dataclass
from enum import Enum

import numpy as np
from scipy.spatial import cKDTree

import rclpy as rp
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
from turtlebot3_my_msg.action import PrecisionDock


class DockingState(Enum):
    INIT = 0
    STAGING_TURN = 1
    STAGING_DRIVE = 2
    STAGING_ALIGN = 3
    ICP_SETTLE = 4
    ALIGN_HEADING = 5
    STRAIGHT_REVERSE = 6
    ALIGN_YAW = 7
    COMPLETED = 8


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def extract_se2_transform(T: np.ndarray):
    """base -> dock 행렬 T에서 dock의 위치(x, y)와 방향각(yaw)을 base 기준 좌표로 변환"""
    R, t = T[:2, :2], T[:2, 2]
    pos_base = -R.T @ t
    # R_base_dock = R.T 기준 각도 추출
    yaw_base = normalize_angle(math.atan2(-R[0, 1], R[0, 0]))
    return float(pos_base[0]), float(pos_base[1]), yaw_base


def icp_2d(source_pts, target_tree, target_pts, initial_transform=None, max_iter=20, search_radius=0.5):
    T = np.identity(3, dtype=np.float32) if initial_transform is None else np.copy(initial_transform)
    prev_error = float('inf')

    for _ in range(max_iter):
        transformed = (source_pts @ T[:2, :2].T) + T[:2, 2]
        distances, indices = target_tree.query(transformed, workers=2)

        valid = distances < search_radius
        if np.sum(valid) < 10:
            break

        pts_src, pts_tgt = source_pts[valid], target_pts[indices[valid]]
        c_src, c_tgt = np.mean(pts_src, axis=0), np.mean(pts_tgt, axis=0)

        H = (pts_src - c_src).T @ (pts_tgt - c_tgt)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        t = c_tgt - (R @ c_src)
        current_error = np.sum(np.abs(T[:2, :2] - R)) + np.sum(np.abs(T[:2, 2] - t))
        T[:2, :2], T[:2, 2] = R, t

        if abs(prev_error - current_error) < 1e-4:
            break
        prev_error = current_error

    fitness = float(np.sum(distances < 0.08) / max(len(target_pts), 1)) if len(distances) > 0 else 0.0
    return T, fitness, int(np.sum(valid))


class PID:
    def __init__(self, p=0.0, i=0.0, d=0.0, out_min=0.0, out_max=0.0):
        self.p, self.i, self.d = p, i, d
        self.out_min, self.out_max = out_min, out_max
        self.prev_err = self.integral = 0.0

    def update(self, error: float, dt: float = 0.05) -> float:
        dt = max(dt, 0.01)
        self.integral += error * dt
        output = (self.p * error) + (self.i * self.integral) + (self.d * (error - self.prev_err) / dt)

        # Anti-windup clamping
        if not (self.out_min <= output <= self.out_max):
            self.integral -= error * dt
        self.prev_err = error
        return float(np.clip(output, self.out_min, self.out_max))

    def reset(self, init_err: float = 0.0):
        self.prev_err = init_err
        self.integral = 0.0


class PrecisionDockingServer(Node):
    def __init__(self):
        super().__init__('precision_docking_server')
        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._load_parameters()
        self._init_controllers()

        self._action_server = ActionServer(
            self, PrecisionDock, 'precision_dock',
            execute_callback=self.execute_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb_group
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = None
        self._reset_state_variables()

        self.get_logger().info('Two-Phase 정밀 도킹 서버 준비 완료 (리팩토링 버전)')

    def _load_parameters(self):
        defaults = {
            'use_sim_time': False,
            'charger_width': 0.04,
            'wing_length': 0.30,
            'wing_angle_deg': 45.0,
            'robot_rear_length': 0.20,
            'roi_x_min': -0.8,
            'roi_x_max': 0.15,
            'roi_y_limit': 0.25,
            'staging_distance': 0.45,
            'final_yaw_tolerance_deg': 1.0,
            'steering_lock_dist': 0.35,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }
        self.declare_parameters('', [(k, v) for k, v in defaults.items()])
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        self.wing_rad = math.radians(self.cfg['wing_angle_deg'])
        self.final_yaw_tol = math.radians(self.cfg['final_yaw_tolerance_deg'])

    def _init_controllers(self):
        # 파라미터화가 용이하도록 일괄 구성
        self.linear_pid = PID(p=0.5, i=0.0, d=0.0, out_min=0.01, out_max=0.03)
        self.angular_pid = PID(p=1.2, i=0.1, d=0.05, out_min=-0.25, out_max=0.25)
        self.staging_linear_pid = PID(p=0.8, i=0.01, d=0.02, out_min=0.015, out_max=0.15)
        self.staging_angular_pid = PID(p=2.0, i=0.05, d=0.08, out_min=-0.4, out_max=0.4)

    def _reset_state_variables(self):
        self.latest_source_pts = self.target_tree = None
        self.new_scan_available = False
        self.current_transform = np.identity(3, dtype=np.float32)
        self.filt_rel_yaw = self.filt_dock_dist = self.prev_yaw = self.prev_dist = None
        self.last_fitness = self.icp_fail_count = self.settle_timer = 0.0
        self.target_dist = self.cfg['robot_rear_length']
        self.target_dock_yaw = 0.0

    def generate_v_funnel(self) -> np.ndarray:
        half_w = self.cfg['charger_width'] / 2.0
        pts = [[0.0, y] for y in np.arange(-half_w, half_w + 0.001, 0.005)]

        for l in np.arange(0.01, self.cfg['wing_length'] + 0.005, 0.025):
            px = l * math.cos(self.wing_rad)
            py = half_w + l * math.sin(self.wing_rad)
            pts.extend([[px, py], [px, -py]])

        return np.array(pts, dtype=np.float32)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment

        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])

        roi = (xs < self.cfg['roi_x_max']) & (xs > self.cfg['roi_x_min']) & (np.abs(ys) < self.cfg['roi_y_limit'])
        if np.sum(roi) >= 10:
            self.latest_source_pts = np.column_stack((xs[roi], ys[roi])).astype(np.float32)
            self.new_scan_available = True
        else:
            self.latest_source_pts = None

    def get_odom_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.cfg['odom_frame'], self.cfg['base_frame'], rp.time.Time())
            return t.transform.translation.x, t.transform.translation.y, get_yaw_from_quaternion(t.transform.rotation)
        except tf2_ros.TransformException:
            return None, None, None

    def publish_vel(self, v: float, w: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd_vel_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('도킹 액션 가동')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data, callback_group=self.cb_group)
        self._reset_state_variables()
        for pid in [self.linear_pid, self.angular_pid, self.staging_linear_pid, self.staging_angular_pid]:
            pid.reset()

        state = DockingState.INIT
        active_target_pts = self.generate_v_funnel()
        self.target_tree = cKDTree(active_target_pts)
        target_odom = {}
        last_valid_time = last_time = self.get_clock().now()
        rate = self.create_rate(20.0)

        try:
            while rp.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.publish_vel(0, 0)
                    return PrecisionDock.Result(success=False)

                curr_time = self.get_clock().now()
                dt = max((curr_time - last_time).nanoseconds / 1e9, 0.01)
                last_time = curr_time
                odom_x, odom_y, odom_yaw = self.get_odom_pose()

                if state == DockingState.INIT:
                    if not self.new_scan_available or odom_x is None:
                        rate.sleep()
                        continue

                    req_x = goal_handle.request.target_pose.pose.position.x
                    self.target_dist = req_x if req_x > 0 else self.cfg['robot_rear_length']
                    self.new_scan_available = False

                    # 무게중심 기반 초기 이동량 보정
                    init_T = np.identity(3, dtype=np.float32)
                    init_T[:2, 2] = np.mean(active_target_pts, axis=0) - np.mean(self.latest_source_pts, axis=0)

                    T, _, match_cnt = icp_2d(self.latest_source_pts, self.target_tree, active_target_pts, init_T, max_iter=40, search_radius=0.5)
                    if match_cnt >= 10:
                        rel_x, rel_y, dock_yaw_in_base = extract_se2_transform(T)
                        dock_odom_x = odom_x + rel_x * math.cos(odom_yaw) - rel_y * math.sin(odom_yaw)
                        dock_odom_y = odom_y + rel_x * math.sin(odom_yaw) + rel_y * math.cos(odom_yaw)
                        dock_odom_yaw = normalize_angle(odom_yaw + dock_yaw_in_base)
                        self.target_dock_yaw = dock_odom_yaw

                        stg_dist = self.cfg['staging_distance']
                        target_odom['x'] = dock_odom_x + stg_dist * math.cos(dock_odom_yaw)
                        target_odom['y'] = dock_odom_y + stg_dist * math.sin(dock_odom_yaw)
                        target_odom['stg_yaw'] = math.atan2(target_odom['y'] - odom_y, target_odom['x'] - odom_x)
                        target_odom['dock_yaw'] = dock_odom_yaw

                        self.get_logger().info(f'도크 인식 성공: Odom({dock_odom_x:.2f}, {dock_odom_y:.2f}), Yaw={math.degrees(dock_odom_yaw):.1f}°')
                        state = DockingState.STAGING_TURN

                elif 1 <= state.value <= 3:
                    if odom_x is not None:
                        state = self._handle_macro_drive(state, dt, odom_x, odom_y, odom_yaw, target_odom)

                elif state.value >= 4:
                    state, last_valid_time = self._handle_micro_docking(state, dt, curr_time, last_valid_time, active_target_pts, odom_yaw)
                    if state == DockingState.COMPLETED:
                        goal_handle.succeed()
                        return PrecisionDock.Result(success=True)

                rate.sleep()

        finally:
            self.publish_vel(0, 0)
            if self.scan_sub:
                self.destroy_subscription(self.scan_sub)

    def _handle_macro_drive(self, state, dt, x, y, yaw, tgt):
        if state == DockingState.STAGING_TURN:
            err = normalize_angle(tgt['stg_yaw'] - yaw)
            if abs(err) < math.radians(1.0):
                self.publish_vel(0.0, 0.0)
                return DockingState.STAGING_DRIVE
            self.publish_vel(0.0, self.staging_angular_pid.update(err, dt))

        elif state == DockingState.STAGING_DRIVE:
            dist = math.hypot(tgt['x'] - x, tgt['y'] - y)
            if dist < 0.02:
                self.publish_vel(0.0, 0.0)
                return DockingState.STAGING_ALIGN
            self.publish_vel(self.staging_linear_pid.update(dist, dt), 0.0)

        elif state == DockingState.STAGING_ALIGN:
            err = normalize_angle(tgt['dock_yaw'] - yaw)
            if abs(err) < math.radians(0.8):
                self.publish_vel(0.0, 0.0)
                self.new_scan_available = False
                self.current_transform = np.identity(3, dtype=np.float32)
                self.filt_dock_dist = self.filt_rel_yaw = self.prev_dist = self.prev_yaw = None
                self.settle_timer = 0.5
                return DockingState.ICP_SETTLE
            self.publish_vel(0.0, self.staging_angular_pid.update(err, dt))

        return state

    def _handle_micro_docking(self, state, dt, curr_time, valid_time, active_pts, odom_yaw):
        if state == DockingState.ICP_SETTLE:
            self.publish_vel(0, 0)
            self.settle_timer -= dt
            if self.settle_timer <= 0:
                return DockingState.ALIGN_HEADING, valid_time

        elif state == DockingState.ALIGN_YAW:
            if odom_yaw is None:
                return state, valid_time
            yaw_err = normalize_angle(self.target_dock_yaw - odom_yaw)
            if abs(yaw_err) < self.final_yaw_tol:
                self.publish_vel(0, 0)
                return DockingState.COMPLETED, valid_time
            self.publish_vel(0, self.staging_angular_pid.update(yaw_err, dt))
            return state, valid_time

        # 센서 기반 후진 제어
        if self.new_scan_available and self.latest_source_pts is not None:
            self.new_scan_available = False
            s_rad = 1.0 if self.filt_dock_dist is None else 0.4
            T, fitness, match_cnt = icp_2d(self.latest_source_pts, self.target_tree, active_pts, self.current_transform, max_iter=20, search_radius=s_rad)

            if match_cnt >= 10:
                valid_time = curr_time
                self.icp_fail_count = 0
                self.current_transform = T
                _, _, raw_yaw = extract_se2_transform(T)
                raw_dist = float(T[0, 2])

                if self.prev_dist is None or (abs(raw_dist - self.prev_dist) <= 0.1 and abs(normalize_angle(raw_yaw - self.prev_yaw)) <= math.radians(10.0)):
                    alpha = 0.6 if fitness > 0.6 else 0.3
                    self.filt_rel_yaw = raw_yaw if self.filt_rel_yaw is None else (alpha * raw_yaw) + ((1 - alpha) * self.filt_rel_yaw)
                    self.filt_dock_dist = raw_dist if self.filt_dock_dist is None else (alpha * raw_dist) + ((1 - alpha) * self.filt_dock_dist)
                    self.prev_dist, self.prev_yaw, self.last_fitness = self.filt_dock_dist, self.filt_rel_yaw, fitness
            else:
                self.icp_fail_count += 1
                if self.icp_fail_count > 3:
                    self.current_transform = np.identity(3, dtype=np.float32)

        if (curr_time - valid_time).nanoseconds / 1e9 > 1.0:
            self.publish_vel(0, 0)
            return state, valid_time

        if self.filt_rel_yaw is not None and self.filt_dock_dist is not None:
            if state == DockingState.ALIGN_HEADING:
                if abs(self.filt_rel_yaw) < math.radians(1.0):
                    self.publish_vel(0, 0)
                    self.linear_pid.reset(init_err=max(0.0, self.filt_dock_dist - self.target_dist))
                    return DockingState.STRAIGHT_REVERSE, valid_time
                self.publish_vel(0, self.angular_pid.update(self.filt_rel_yaw, dt))

            elif state == DockingState.STRAIGHT_REVERSE:
                dist_error = max(0.0, self.filt_dock_dist - self.target_dist)
                if dist_error <= 0.015:
                    self.publish_vel(0, 0)
                    return DockingState.ALIGN_YAW, valid_time

                v = -self.linear_pid.update(dist_error, dt)
                w = 0.0 if self.filt_dock_dist < self.cfg['steering_lock_dist'] else self.angular_pid.update(self.filt_rel_yaw, dt)
                self.publish_vel(v, w)

        return state, valid_time


def main(args=None):
    rp.init(args=args)
    node = PrecisionDockingServer()
    executor = MultiThreadedExecutor(num_threads=4)
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