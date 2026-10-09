"""TurtleBot3 MAP-ONLY 정밀 도킹 서버.

완전 도킹된 로봇 Map Pose -> 전방 경유지 -> Map 사전 위치로 제한한 ICP -> S자 후진.
기준: E1I6-Logistics/Logistics_AMR dev. 실제 로봇 주행은 미검증.
"""
import math
import threading
from enum import Enum

import numpy as np
from scipy.spatial import cKDTree
import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import tf2_ros
from turtlebot3_my_msg.action import PrecisionDock

from .icp_guard import ( StableDockTracker, select_scan_near_expected_dock, validate_micro_fit, )
from .docking_recovery import (retreat_distance, axis_progress, corridor_clear, unsafe_turn_near_dock, MotionStallWatchdog)
from .map_docking import ( staging_pose, dock_model_pose, base_to_model_transform, observed_model_map_pose, accept_map_prior, )
from .docking_control import ( wrap_angle, clamp, yaw_target_from_lateral, reverse_angular_error,
    final_entry_ok, reverse_axis_progress, forward_heading_command, map_target_yaw_to_odom, )


class DockingState(Enum):
    INIT = 0
    STAGING_TURN = 1
    STAGING_DRIVE = 2
    STAGING_ALIGN = 3
    ICP_SETTLE = 4
    ALIGN_HEADING = 5
    STRAIGHT_REVERSE = 6
    FINAL_REVERSE = 7
    RECOVERY_STOP = 8
    RECOVERY_EXIT = 9
    COMPLETED = 10
    FAILED = 11


def normalize_angle(angle: float) -> float:
    return wrap_angle(angle)


def get_yaw_from_quaternion(q) -> float:
    return math.atan2( 2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z), )


def icp_2d(source_pts, target_tree, target_pts, initial_transform=None, max_iter=20, search_radius=0.5):
    """Original point-to-point 2D ICP retained for compatible V-model fits."""
    T = (np.identity(3, dtype=np.float32) if initial_transform is None else np.copy(initial_transform))
    prev_error = float('inf')
    matched_count = 0
    distances = np.array([], dtype=float)
    for _ in range(max_iter):
        R_current, t_current = T[:2, :2], T[:2, 2]
        transformed = source_pts @ R_current.T + t_current
        distances, indices = target_tree.query(transformed, workers=1)
        valid = distances < search_radius
        matched_count = int(np.count_nonzero(valid))
        if matched_count < 10:
            break
        src, dst = source_pts[valid], target_pts[indices[valid]]
        c_src, c_dst = src.mean(axis=0), dst.mean(axis=0)
        H = (src - c_src).T @ (dst - c_dst)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        t = c_dst - R @ c_src
        delta = (np.abs(T[:2, :2] - R).sum() + np.abs(T[:2, 2] - t).sum())
        T[:2, :2], T[:2, 2] = R, t
        if abs(prev_error - delta) < 1e-4:
            break
        prev_error = delta
    # Recompute quality for FINAL T; the last loop's distances were computed
    # before T was updated and may otherwise give a misleading success count.
    if len(source_pts):
        final_points = source_pts @ T[:2, :2].T + T[:2, 2]
        final_distances, _ = target_tree.query(final_points, workers=1)
        matched_count = int(np.count_nonzero(final_distances < search_radius))
        fitness = float(np.mean(final_distances < 0.08))
    else:
        matched_count, fitness = 0, 0.0
    return T, fitness, matched_count


class PID:
    def __init__(self, p, i, d, out_min, out_max):
        self.p, self.i, self.d = p, i, d
        self.out_min, self.out_max = out_min, out_max
        self.reset()

    def update(self, error, dt=0.05):
        dt = max(dt, 0.01)
        self.integral += error*dt
        output = (self.p*error + self.i*self.integral + self.d*(error-self.prev_err)/dt)
        if output > self.out_max or output < self.out_min:
            self.integral -= error*dt
        self.prev_err = error
        return float(np.clip(output, self.out_min, self.out_max))

    def reset(self):
        self.prev_err, self.integral = 0.0, 0.0


class PrecisionDockingServer(Node):
    def __init__(self):
        super().__init__('precision_docking_server')
        self.cb_group = ReentrantCallbackGroup()
        self.scan_lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._action_server = ActionServer( self, PrecisionDock, 'precision_dock', execute_callback=self.execute_callback,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback, callback_group=self.cb_group, )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = None
        self._declare_and_update_params()
        self._reset_state_variables()
        self.contact_sub = (self.create_subscription(Bool, self.p['contact_topic'], self.contact_callback, 10, callback_group=self.cb_group)
                            if self.p['contact_topic'] else None)
        self._configure_controllers()
        self.get_logger().info( 'MAP-ONLY 정밀 도킹: registered final pose + map-prior ICP + guarded reverse' )

    def _declare_and_update_params(self):
        params = {
            # 도크 V 모델 및 LiDAR ROI (실제 사용값은 launch에서 덮어씀)
            'charger_width': 0.145, 'wing_length': 0.10, 'wing_angle_deg': 22.5, 'robot_rear_length': 0.10,
            'roi_x_min': -1.2, 'roi_x_max': 0.15, 'roi_y_limit': 0.20,
            'odom_frame': 'odom', 'base_frame': 'base_footprint', 'map_frame': 'map',
            # 로봇이 완전히 도킹된 상태의 map Pose: initial_points와 동일
            'dock_pose_configured': False, 'dock_final_map_x': 0.0, 'dock_final_map_y': 0.0, 'dock_final_map_yaw': 0.0,
            # 도킹 완료 로봇 중심 기준 거리. 모델 원점은 로봇 뒤, 경유지는 로봇 앞.
            'staging_offset_from_final': 0.35, 'dock_model_rear_offset': 0.10,
            'map_prior_max_xy_m': 0.040, 'map_prior_max_yaw_deg': 7.0, 'map_tf_max_age_sec': 1.0,
            'staging_start_max_distance_m': 0.75,
            # 기본값은 이동 금지: 위치 계산만 확인
            'dry_run': True,
            # 연속 ICP 확인 + TF/센서 타임아웃
            'micro_required_scans': 3, 'init_consistency_xy_m': 0.025, 'init_consistency_yaw_deg': 3.0,
            'init_timeout_sec': 10.0, 'micro_timeout_sec': 25.0,
            'scan_stale_stop_sec': 0.25, 'scan_abort_sec': 3.0,
            # Map 기반 경유지까지 전진하면서 Heading 오차 보정
            'staging_max_v': 0.15, 'staging_max_w': 0.4, 'staging_heading_kp': 1.6,
            'staging_yaw_slowdown_deg': 15.0, 'staging_timeout_sec': 30.0,
            'staging_front_lookahead_m': 0.12, 'staging_no_spin_radius_m': 0.30, 'staging_near_dock_max_turn_deg': 3.0,
            # 도크 예상 형상 주변 점만 ICP 정합: 인접 모서리 영향 감소
            'micro_first_gate_radius_m': 0.045, 'micro_gate_radius_m': 0.035, 'micro_match_radius_m': 0.027,
            # 후진 횡오차 -> 목표 Yaw 변환, 도크에 가까워지면 횡보정 요구각 감쇠
            'reverse_lateral_gain': 1.5, 'reverse_max_target_yaw_deg': 6.0,
            'reverse_yaw_kp': 0.5, 'reverse_yaw_kd': 0.10, 'reverse_max_w': 0.30,
            'reverse_taper_start_dist': 0.34, 'reverse_taper_end_dist': 0.22,
            # 최종 20cm 전부터 조향 허용오차 검사, 이후 Odom 저속 후진
            'final_blind_start_dist': 0.20, 'final_entry_lateral_tol': 0.015, 'final_entry_yaw_tol_deg': 2.0,
            'final_reverse_speed': 0.015, 'final_reverse_tolerance': 0.005,
            'final_yaw_hold_kp': 1.5, 'final_yaw_hold_max_w': 0.12,
            'final_max_lateral_drift_m': 0.015, 'final_timeout_sec': 12.0,
            # 완전 도킹 시 로봇의 전방은 등록된 Map Yaw와 같아야 한다.
            # 최종 Yaw는 성공 조건이 아니라 충돌 위험 방지를 위한 진입/후진 제한.
            'final_map_yaw_tol_deg': 2.0, 'final_map_yaw_abort_deg': 4.0,
            # 도킹 복구: 안전한 직진 이탈 -> Map 경유지 -> ICP 후진. 횟수 초과 시 실패.
            'recovery_max_retries': 2, 'recovery_stop_sec': 0.6, 'recovery_exit_extra_m': 0.05,
            'recovery_exit_min_m': 0.12, 'recovery_exit_max_m': 0.55,
            'recovery_exit_speed': 0.020, 'recovery_exit_timeout_sec': 35.0,
            'recovery_front_half_width_m': 0.16, 'recovery_front_offset_m': 0.13,
            'recovery_front_margin_m': 0.07, 'recovery_scan_max_age_sec': 0.35,
            'recovery_lateral_drift_max_m': 0.035, 'recovery_max_heading_error_deg': 5.0, 'recovery_heading_drift_max_deg': 3.0,
            'recovery_contact_release_m': 0.08,
            'stall_timeout_sec': 2.5, 'stall_progress_m': 0.010, 'stall_min_command_v': 0.004,
            # 선택: 물리적 접촉 센서 Bool 토픽. 빈 문자열이면 contact 감지 없음.
            'contact_topic': '', 'contact_timeout_sec': 1.0,
        }
        for name, default in params.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in params}
        self.wing_rad = math.radians(self.p['wing_angle_deg'])
        self.final_map_pose = ( float(self.p['dock_final_map_x']), float(self.p['dock_final_map_y']), float(self.p['dock_final_map_yaw']), )
        self.model_map_pose = dock_model_pose( self.final_map_pose, self.p['dock_model_rear_offset'])
        self.staging_map_pose = staging_pose( self.final_map_pose, self.p['staging_offset_from_final'])
        if not (self.p['staging_offset_from_final'] > 0.0 and self.p['staging_offset_from_final'] + self.p['dock_model_rear_offset'] < 1.2):
            raise ValueError('Invalid staging/model offset')
        if not (self.p['reverse_taper_start_dist'] > self.p['reverse_taper_end_dist'] >
                self.p['final_blind_start_dist'] > self.p['robot_rear_length']):
            raise ValueError('Require taper_start > taper_end > blind_start > rear_length')
        if not 0 < self.p['final_map_yaw_tol_deg'] < self.p['final_map_yaw_abort_deg'] <= 15.:
            raise ValueError('Require 0 < final map yaw tolerance < abort angle <= 15 deg')
        if not (0 <= self.p['recovery_max_retries'] <= 3 and 0 < self.p['recovery_exit_min_m'] < self.p['recovery_exit_max_m'] <= 1.):
            raise ValueError('Invalid recovery attempts or exit limits')

    def _configure_controllers(self):
        self.linear_pid = PID(.1, 0., .02, .005, .015)
        self.angular_pid = PID( self.p['reverse_yaw_kp'], 0., self.p['reverse_yaw_kd'], -self.p['reverse_max_w'], self.p['reverse_max_w'], )
        self.staging_linear_pid = PID( 4., .01, .02, .015, self.p['staging_max_v'], )
        self.staging_angular_pid = PID( 2., .05, .08, -self.p['staging_max_w'], self.p['staging_max_w'], )

    def _reset_state_variables(self):
        with self.scan_lock:
            self.latest_source_pts = None
            self.latest_scan_stamp = None
            self.new_scan_available = False
            self.latest_front_x = self.latest_front_stamp = None
        # Action 재시작 시 최신 접촉 상태를 지우지 않는다(센서 True 은닉 방지).
        if not hasattr(self, 'contact_last_time'):
            self.contact_active, self.contact_last_time = False, None
        self.last_linear_command = 0.0
        self.stall_watchdog = MotionStallWatchdog(self.p['stall_timeout_sec'], self.p['stall_progress_m'], self.p['stall_min_command_v'])
        self.recovery_attempts, self.recovery_exit_distance = 0, None
        self.recovery_start_odom = None
        self.recovery_reason = ''
        self.target_tree = None
        self.current_transform = np.eye(3, dtype=np.float32)
        self.filt_rel_yaw = self.filt_dock_dist = self.filt_lateral_error = None
        self.prev_yaw = self.prev_dist = self.prev_lateral = None
        self.last_fitness = 0.0
        self.icp_fail_count = 0
        self.settle_timer = 0.0
        self.target_dist = float(self.p['robot_rear_length'])
        self.final_reverse_start_x = None
        self.final_reverse_start_y = None
        self.final_reverse_start_yaw = None
        self.final_reverse_target_odom_yaw = None
        self.final_reverse_distance = 0.0
        self.failure_reason = ''
        self.dock_tracker = StableDockTracker( self.p['micro_required_scans'], self.p['init_consistency_xy_m'], self.p['init_consistency_yaw_deg'],
        )

    def generate_v_funnel(self):
        pts = []
        half_w = self.p['charger_width']/2.0
        for y in np.arange(-half_w, half_w + .001, .005):
            pts.append([0., y])
        for length in np.arange(.01, self.p['wing_length'] + .005, .025):
            px = length*math.cos(self.wing_rad)
            py = half_w + length*math.sin(self.wing_rad)
            pts.extend([[px, py], [px, -py]])
        return np.asarray(pts, dtype=np.float32)

    def scan_callback(self, msg: LaserScan):
        """Project in scan frame, TF to base_footprint, THEN crop ROI."""
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = (msg.angle_min + np.arange(len(ranges), dtype=np.float32)* msg.angle_increment)
        valid = ((ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges))
        xs = ranges[valid]*np.cos(angles[valid])
        ys = ranges[valid]*np.sin(angles[valid])
        if msg.header.frame_id != self.p['base_frame']:
            try:
                tr = self.tf_buffer.lookup_transform( self.p['base_frame'], msg.header.frame_id, rp.time.Time.from_msg(msg.header.stamp),
                ).transform
            except tf2_ros.TransformException as exc:
                # On startup or time discontinuity: drop scan, never treat sensor
                # frame as robot-base frame or reuse mismatched points.
                self.get_logger().warn(f'LiDAR -> base TF unavailable: {exc}')
                with self.scan_lock:
                    self.new_scan_available = False
                    self.latest_source_pts = None
                    self.latest_scan_stamp = None
                    self.latest_front_x = self.latest_front_stamp = None
                return
            sensor_yaw = get_yaw_from_quaternion(tr.rotation)
            c, s = math.cos(sensor_yaw), math.sin(sensor_yaw)
            xs, ys = (tr.translation.x + c*xs - s*ys, tr.translation.y + s*xs + c*ys)
        # 전방 복구 경로를 검사할 때는 후방 도킹 ROI가 아니라 전체 스캔을 사용한다.
        lane = (xs > 0.) & (np.abs(ys) < self.p['recovery_front_half_width_m'])
        front = float(np.min(xs[lane])) if np.any(lane) else None
        roi = ((xs < self.p['roi_x_max']) & (xs > self.p['roi_x_min']) & (np.abs(ys) < self.p['roi_y_limit']))
        src = (np.column_stack((xs[roi], ys[roi])).astype(np.float32) if np.count_nonzero(roi) >= 10 else None)
        with self.scan_lock:
            self.latest_source_pts = src
            self.latest_scan_stamp = msg.header.stamp if src is not None else None
            self.new_scan_available = src is not None
            self.latest_front_x, self.latest_front_stamp = front, msg.header.stamp

    def contact_callback(self, msg: Bool):
        # True = 외부 범퍼/접촉 검출. 실제 토픽은 현장 하드웨어에 맞게 설정.
        self.contact_active, self.contact_last_time = bool(msg.data), self.get_clock().now()

    def _contact_sensor_healthy(self, now):
        # 범퍼 토픽을 설정했다면 센서가 실제로 발행 중이어야 주행 허용.
        if not self.p['contact_topic']: return True
        if self.contact_last_time is None: return False
        age = (now-self.contact_last_time).nanoseconds/1e9
        return 0 <= age <= self.p['contact_timeout_sec']

    def _forward_clear(self, travel=0.0):
        with self.scan_lock: front, stamp = self.latest_front_x, self.latest_front_stamp
        if stamp is None: return False
        age = (self.get_clock().now()-rp.time.Time.from_msg(stamp)).nanoseconds/1e9
        return 0 <= age <= self.p['recovery_scan_max_age_sec'] and corridor_clear(
            front, self.p['recovery_front_offset_m'], self.p['recovery_front_margin_m'], travel)

    def _take_scan_snapshot(self):
        with self.scan_lock:
            if not self.new_scan_available or self.latest_source_pts is None:
                return None
            src = self.latest_source_pts.copy()
            stamp = self.latest_scan_stamp
            self.new_scan_available = False
        # 오래된 /scan으로 ICP를 수행하면 Map 기반 후보 검증 자체가 틀어질 수 있다.
        if stamp is None: return None
        age = (self.get_clock().now()-rp.time.Time.from_msg(stamp)).nanoseconds/1e9
        if age < 0. or age > self.p['scan_stale_stop_sec']: return None
        return src, stamp

    def _discard_pending_scan(self):
        with self.scan_lock:
            self.new_scan_available = False

    def get_odom_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform( self.p['odom_frame'], self.p['base_frame'], rp.time.Time())
            tf = tr.transform
            return (tf.translation.x, tf.translation.y, get_yaw_from_quaternion(tf.rotation))
        except tf2_ros.TransformException:
            return None, None, None

    def get_map_pose(self, stamp=None):
        """Return (x,y,yaw) of base in MAP, only with fresh localized TF."""
        try:
            query_time = (rp.time.Time.from_msg(stamp) if stamp is not None else rp.time.Time())
            stamped = self.tf_buffer.lookup_transform( self.p['map_frame'], self.p['base_frame'], query_time)
            t = stamped.transform
            age_sec = (self.get_clock().now() - rp.time.Time.from_msg(stamped.header.stamp)).nanoseconds / 1e9
            if age_sec > self.p['map_tf_max_age_sec'] or age_sec < -0.25:
                return None
            pose = (t.translation.x, t.translation.y, get_yaw_from_quaternion(t.rotation))
            if not all(math.isfinite(value) for value in pose):
                return None
            return pose
        except (tf2_ros.TransformException, ValueError):
            return None

    def goal_callback(self, goal_request):
        with self.goal_lock:
            if self.goal_active:
                self.get_logger().warn('Already docking: reject simultaneous goal')
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def publish_vel(self, v, w):
        # Dry-run에서는 상태 전이와 무관하게 속도 명령을 강제로 0으로 만든다.
        if self.p['dry_run']: v, w = 0., 0.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)
        self.last_linear_command = float(v)

    def _fail(self, message):
        self.failure_reason = str(message)
        self.publish_vel(0., 0.)
        self.get_logger().error(self.failure_reason)
        return DockingState.FAILED

    def _start_recovery(self, reason):
        # contact 또는 후진 정체 / 진입 형상 불량: 즉시 정지, 경유지 복귀.
        self.publish_vel(0., 0.)
        self.stall_watchdog.reset()
        if self.recovery_attempts >= self.p['recovery_max_retries']:
            return self._fail(f'Recovery exhausted ({self.recovery_attempts}): {reason}')
        self.recovery_attempts += 1
        self.recovery_reason, self.recovery_start_odom = str(reason), None
        self.get_logger().warn('RECOVERY %d/%d: %s' % (self.recovery_attempts, self.p['recovery_max_retries'], reason))
        return DockingState.RECOVERY_STOP

    def _handle_recovery(self, state, elapsed, odom_pose, target):
        x, y, yaw = odom_pose
        localized = self.get_map_pose()
        if localized is None or x is None: return self._fail('RECOVERY: MAP/ODOM TF unavailable')
        if state == DockingState.RECOVERY_STOP:
            self.publish_vel(0., 0.)
            if elapsed < self.p['recovery_stop_sec']: return state
            yaw_err = abs(wrap_angle(localized[2]-self.final_map_pose[2]))
            if yaw_err > math.radians(self.p['recovery_max_heading_error_deg']):
                return self._fail('RECOVERY: large yaw; cannot safely exit straight')
            dist = retreat_distance(localized, self.final_map_pose, self.p['staging_offset_from_final'],
                self.p['recovery_exit_extra_m'], self.p['recovery_exit_min_m'], self.p['recovery_exit_max_m'])
            if dist is None: return self._fail('RECOVERY: exit distance out of safety bounds')
            # 먼저 전체 이탈 거리만큼 전방에 공간이 있는지 확인한다.
            if not self._forward_clear(dist): return self._fail('RECOVERY: forward exit corridor blocked/unknown')
            self.recovery_exit_distance, self.recovery_start_odom = dist, (x, y, yaw)
            self.get_logger().warn('RECOVERY_EXIT: %.3fm forward, no in-dock turning' % dist)
            return DockingState.RECOVERY_EXIT
        if elapsed > self.p['recovery_exit_timeout_sec']: return self._fail('RECOVERY: exit timeout')
        start = self.recovery_start_odom
        moved = axis_progress(*start, x, y)
        # 최초 확인만으로는 부족하다. 남은 이탈 거리 전부를 매 주기 다시 확인.
        remaining_exit = max(0., self.recovery_exit_distance-moved)
        if not self._forward_clear(remaining_exit): return self._fail('RECOVERY: remaining forward path blocked/scan stale')
        lateral = abs(-(x-start[0])*math.sin(start[2]) + (y-start[1])*math.cos(start[2]))
        heading_drift = abs(wrap_angle(yaw-start[2]))
        if lateral > self.p['recovery_lateral_drift_max_m'] or moved < -.015 or heading_drift > math.radians(self.p['recovery_heading_drift_max_deg']):
            return self._fail('RECOVERY: lateral/heading drift or wrong direction')
        if self.contact_active and moved >= self.p['recovery_contact_release_m']:
            return self._fail('RECOVERY: contact sensor still active after release travel')
        if self.stall_watchdog.observe(self.get_clock().now().nanoseconds/1e9, (x,y,yaw), self.last_linear_command):
            return self._fail('RECOVERY: forward movement stalled')
        if moved >= self.recovery_exit_distance-.010:
            self.publish_vel(0., 0.)
            mx, my, _ = localized
            sx, sy, syaw = self.staging_map_pose
            target['x'], target['y'] = sx, sy
            target['stg_yaw'] = math.atan2(sy-my, sx-mx) if math.hypot(sx-mx, sy-my) > .02 else syaw
            target['dock_yaw'] = syaw
            # 목표 경유지보다 약간 앞까지 빠졌다면 다시 180° 돌지 않는다.
            # 후진 직전 정렬을 바로 재시작하며, 과도한 위치 오차는 수동/상위 재배치.
            staging_error = math.hypot(sx-mx, sy-my)
            if staging_error > .08:
                return self._fail('RECOVERY: staging displacement >8cm; require Nav2 reposition')
            for pid in (self.linear_pid, self.angular_pid, self.staging_linear_pid, self.staging_angular_pid): pid.reset()
            self.dock_tracker.reset()
            self.filt_rel_yaw = self.filt_dock_dist = self.filt_lateral_error = None
            self.prev_yaw = self.prev_dist = self.prev_lateral = None
            self.current_transform = np.eye(3, dtype=np.float32)
            self.final_reverse_start_x = self.final_reverse_start_y = self.final_reverse_target_odom_yaw = None
            self._discard_pending_scan()
            self.get_logger().info('RECOVERY -> STAGING_ALIGN, map target confirmed')
            return DockingState.STAGING_ALIGN
        # 도크 내부에서는 정면으로 빠지기만 하며 제자리 회전하지 않는다.
        self.publish_vel(self.p['recovery_exit_speed'], 0.)
        return state

    def _send_feedback(self, goal_handle):
        if self.filt_dock_dist is None:
            return
        fb = PrecisionDock.Feedback()
        fb.distance_remaining = float(max(0., self.filt_dock_dist-self.target_dist))
        fb.angle_remaining = float(abs(self.filt_rel_yaw or 0.))
        goal_handle.publish_feedback(fb)

    def execute_callback(self, goal_handle):
        # Map 기준 경유지를 먼저 계산하므로 INIT에서 광역 ICP를 수행하지 않는다.
        self._reset_state_variables()
        self.scan_sub = self.create_subscription( LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data, callback_group=self.cb_group, )
        for pid in (self.linear_pid, self.angular_pid, self.staging_linear_pid, self.staging_angular_pid):
            pid.reset()
        # MAP-ONLY goal contract: target_pose is only an Action trigger.
        # NEVER interpret position.x as a distance or a map x coordinate.
        self.target_dist = float(self.p['dock_model_rear_offset'])
        if not bool(self.p['dock_pose_configured']):
            goal_handle.abort()
            self.publish_vel(0., 0.)
            with self.goal_lock:
                self.goal_active = False
            if self.scan_sub is not None:
                self.destroy_subscription(self.scan_sub)
                self.scan_sub = None
            return PrecisionDock.Result( success=False, message='MAP-ONLY: robot dock pose not configured')
        if not (0.03 <= self.target_dist < self.p['final_blind_start_dist']):
            goal_handle.abort()
            self.publish_vel(0., 0.)
            with self.goal_lock:
                self.goal_active = False
            if self.scan_sub is not None:
                self.destroy_subscription(self.scan_sub)
                self.scan_sub = None
            return PrecisionDock.Result( success=False, message='MAP-ONLY: invalid dock model rear offset')

        self.model_points = self.generate_v_funnel()
        self.target_tree = cKDTree(self.model_points)
        target = {'x': 0., 'y': 0., 'stg_yaw': 0., 'dock_yaw': 0.}
        state = DockingState.INIT
        rate = self.create_rate(20.0)
        entered = last = self.get_clock().now()
        valid_time = entered
        last_feedback = entered
        try:
            while rp.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return PrecisionDock.Result(success=False, message='canceled')
                now = self.get_clock().now()
                dt = max(.01, min(.2, (now-last).nanoseconds/1e9))
                last = now
                x, y, yaw = self.get_odom_pose()
                elapsed = (now-entered).nanoseconds/1e9
                previous_state = state

                # 선택한 물리적 접촉 센서가 끊어졌다면 False라고 가정하지 않는다.
                if not self._contact_sensor_healthy(now):
                    if state == DockingState.INIT and elapsed <= self.p['init_timeout_sec']:
                        self.publish_vel(0., 0.)   # 첫 센서 heartbeat 수신 전엔 대기
                        rate.sleep()
                        continue
                    state = self._fail('Contact sensor missing/stale')
                # 정밀 후진과 복구 전진에는 연속적인 odom이 반드시 필요.
                elif state not in (DockingState.INIT, DockingState.FAILED) and x is None:
                    state = self._fail('ODOM unavailable during docking/recovery')

                if state == DockingState.INIT:
                    self.publish_vel(0., 0.)
                    if elapsed > self.p['init_timeout_sec']:
                        state = self._fail('MAP-ONLY INIT: map->base TF unavailable')
                    else:
                        localized = self.get_map_pose()
                        if localized is not None and x is not None:
                            mx, my, myaw = localized
                            sx, sy, syaw = self.staging_map_pose
                            start_dist = math.hypot(sx-mx, sy-my)
                            if start_dist > self.p['staging_start_max_distance_m']:
                                state = self._fail( 'MAP-ONLY: robot too far from staging; ' 'navigate close using Nav2 first')
                            else:
                                target['x'] = sx
                                target['y'] = sy
                                target['stg_yaw'] = math.atan2(sy-my, sx-mx) if start_dist > .02 else syaw
                                target['dock_yaw'] = syaw
                                self.get_logger().info( 'MAP-ONLY target: final=(%.4f,%.4f,%.4f), ' 'model=(%.4f,%.4f,%.4f), '
                                    'staging=(%.4f,%.4f,%.4f), start_dist=%.3f' % (self.final_map_pose + self.model_map_pose +
                                     self.staging_map_pose + (start_dist,)))
                                if self.p['dry_run']:
                                    state = self._fail( 'DRY_RUN: staging/map target computed; '
                                        'motion disabled (set dry_run:=false after validation)')
                                else:
                                    state = DockingState.STAGING_TURN

                elif state in (DockingState.STAGING_TURN, DockingState.STAGING_DRIVE, DockingState.STAGING_ALIGN):
                    if self.contact_active:
                        state = self._fail('STAGING: physical contact reported')
                    elif (state == DockingState.STAGING_DRIVE and self.stall_watchdog.observe(
                            now.nanoseconds/1e9, (x,y,yaw) if x is not None else None, self.last_linear_command)):
                        state = self._fail('STAGING: forward drive stalled')
                    elif elapsed > self.p['staging_timeout_sec']:
                        state = self._fail(f'{state.name}: staging timeout')
                    else:
                        localized = self.get_map_pose()
                        if localized is None:
                            state = self._fail('MAP-ONLY STAGING: localization TF lost/stale')
                        elif x is None:
                            state = self._fail('MAP-ONLY STAGING: odom TF lost')
                        else:
                            state = self._handle_macro_drive( state, dt, *localized, target)

                elif state in (DockingState.ICP_SETTLE, DockingState.ALIGN_HEADING, DockingState.STRAIGHT_REVERSE, DockingState.FINAL_REVERSE):
                    timeout = self.p['final_timeout_sec'] if state == DockingState.FINAL_REVERSE else self.p['micro_timeout_sec']
                    if self.contact_active:
                        state = self._start_recovery('external contact sensor active')
                    elif (state in (DockingState.STRAIGHT_REVERSE, DockingState.FINAL_REVERSE) and
                          self.stall_watchdog.observe(now.nanoseconds/1e9, (x,y,yaw) if x is not None else None, self.last_linear_command)):
                        state = self._start_recovery('reverse movement stalled')
                    elif elapsed > timeout:
                        state = (self._start_recovery(f'{state.name}: docking timeout') if state in
                            (DockingState.STRAIGHT_REVERSE, DockingState.FINAL_REVERSE) else self._fail(f'{state.name}: timeout'))
                    else:
                        state, valid_time = self._handle_micro_docking(state, dt, now, valid_time, x, y, yaw)
                elif state in (DockingState.RECOVERY_STOP, DockingState.RECOVERY_EXIT):
                    state = self._handle_recovery(state, elapsed, (x,y,yaw), target)

                if state == DockingState.COMPLETED:
                    self.publish_vel(0, 0)
                    goal_handle.succeed()
                    return PrecisionDock.Result(success=True, message='target depth reached (odom estimate; physical entry not verified)')
                if state == DockingState.FAILED:
                    goal_handle.abort()
                    return PrecisionDock.Result(success=False, message=self.failure_reason)
                if state != previous_state:
                    self.get_logger().info(f'{previous_state.name} -> {state.name}')
                    entered = now
                    self.stall_watchdog.reset()
                    # 복구 전진이 오래 걸려도 이전 ICP의 검증 타임아웃이 이월되지 않도록 초기화.
                    if state == DockingState.ICP_SETTLE: valid_time = now
                if (now-last_feedback).nanoseconds/1e9 >= .2:
                    self._send_feedback(goal_handle)
                    last_feedback = now
                rate.sleep()
            goal_handle.abort()
            return PrecisionDock.Result(success=False, message='ROS stopped')
        finally:
            self.publish_vel(0, 0)
            if self.scan_sub is not None:
                self.destroy_subscription(self.scan_sub)
                self.scan_sub = None
            with self.goal_lock:
                self.goal_active = False
            self.get_logger().info('precision docking goal resources released')

    def _handle_macro_drive(self, state, dt, x, y, yaw, tgt):
        if state == DockingState.STAGING_TURN:
            error = wrap_angle(tgt['stg_yaw']-yaw)
            # 로봇이 도크 내부/입구에 있으면 큰 제자리 회전으로 날개와 충돌할 수 있다.
            if unsafe_turn_near_dock((x,y,yaw), self.final_map_pose, error,
                    self.p['staging_no_spin_radius_m'], self.p['staging_near_dock_max_turn_deg']):
                return self._fail('STAGING_TURN: unsafe in-dock rotation; reposition manually')
            if abs(error) < math.radians(1.5):
                self.publish_vel(0., 0.)
                self.staging_angular_pid.reset()
                return DockingState.STAGING_DRIVE
            self.publish_vel(0., self.staging_angular_pid.update(error, dt))

        elif state == DockingState.STAGING_DRIVE:
            dx, dy = tgt['x']-x, tgt['y']-y
            distance = math.hypot(dx, dy)
            if distance < .02:
                self.publish_vel(0., 0.)
                self.staging_angular_pid.reset()
                self.staging_linear_pid.reset()
                return DockingState.STAGING_ALIGN
            yaw_error = wrap_angle(math.atan2(dy, dx)-yaw)
            # Nav2 경로 계획을 쓰지 않으므로 전진 경로도 별도 lookahead 검사.
            if not self._forward_clear(self.p['staging_front_lookahead_m']):
                return self._fail('STAGING: front lookahead blocked or unobserved')
            # Rotate at low or zero linear velocity if pose drifts too much.
            w = forward_heading_command(
                x, y, yaw, tgt['x'], tgt['y'],
                self.p['staging_heading_kp'], self.p['staging_max_w'])
            if abs(yaw_error) > math.radians(45.):
                self.publish_vel(0., w)
                return state
            v = self.staging_linear_pid.update(distance, dt)
            slow = clamp(1.0 - abs(yaw_error)/math.radians( self.p['staging_yaw_slowdown_deg']), .20, 1.0)
            self.publish_vel(v*slow, w)

        elif state == DockingState.STAGING_ALIGN:
            error = wrap_angle(tgt['dock_yaw']-yaw)
            if abs(error) < math.radians(1.):
                self.publish_vel(0., 0.)
                self._discard_pending_scan()
                # x/y/yaw are MAP coordinates in STAGING mode.
                self.current_transform = base_to_model_transform(
                    (x, y, yaw), self.model_map_pose)
                self.dock_tracker.reset()
                self.filt_dock_dist = None
                self.filt_rel_yaw = None
                self.filt_lateral_error = None
                self.prev_dist = self.prev_yaw = self.prev_lateral = None
                self.settle_timer = .5
                self.staging_angular_pid.reset()
                return DockingState.ICP_SETTLE
            self.publish_vel(0., self.staging_angular_pid.update(error, dt))
        return state

    def _accept_and_filter_icp(self, T, fitness, now):
        R, t = T[:2, :2], T[:2, 2]
        raw_dist, raw_lat = float(t[0]), float(t[1])
        raw_yaw = wrap_angle(math.atan2(R[0, 1], R[0, 0]))
        if self.prev_dist is None:
            ref = self.current_transform
            ref_dist, ref_lat = float(ref[0, 2]), float(ref[1, 2])
            ref_yaw = wrap_angle(math.atan2(ref[0, 1], ref[0, 0]))
        else:
            ref_dist, ref_lat, ref_yaw = ( self.prev_dist, self.prev_lateral, self.prev_yaw)
        jumps = (abs(raw_dist-ref_dist), abs(raw_lat-ref_lat), abs(wrap_angle(raw_yaw-ref_yaw)))
        if (jumps[0] > .10 or jumps[1] > .05 or jumps[2] > math.radians(10.)):
            self.get_logger().warn( 'Reject ICP jump: dist=%.3f lat=%.3f yaw=%.1fdeg ' 'delta=(%.3f,%.3f,%.1fdeg)' %
                (raw_dist, raw_lat, math.degrees(raw_yaw), jumps[0], jumps[1], math.degrees(jumps[2])))
            return False
        self.current_transform = T
        alpha = (.6 if fitness > .6 else .4 if fitness > .3 else .2)
        if self.filt_rel_yaw is None:
            self.filt_rel_yaw = raw_yaw
            self.filt_dock_dist = raw_dist
            self.filt_lateral_error = raw_lat
        else:
            self.filt_rel_yaw = wrap_angle( self.filt_rel_yaw + alpha*wrap_angle(raw_yaw-self.filt_rel_yaw))
            self.filt_dock_dist = (alpha*raw_dist + (1.-alpha)*self.filt_dock_dist)
            self.filt_lateral_error = (alpha*raw_lat + (1.-alpha)*self.filt_lateral_error)
        self.prev_dist, self.prev_yaw, self.prev_lateral = ( self.filt_dock_dist, self.filt_rel_yaw, self.filt_lateral_error)
        self.last_fitness = fitness
        self.icp_fail_count = 0
        return True

    def _check_final_map_heading(self):
        """Verify robot front direction at the registered final Map yaw."""
        map_pose = self.get_map_pose()
        error = None if map_pose is None else wrap_angle(self.final_map_pose[2]-map_pose[2])
        return (error is not None and abs(error) <= math.radians(self.p['final_map_yaw_tol_deg'])), error, map_pose

    def _handle_micro_docking(self, state, dt, now, last_valid_time, odom_x, odom_y, odom_yaw):
        # 후진 ICP: Map 예측 도크 주변만 정합하고 매번 품질·위치·시간 일관성을 확인.
        if state == DockingState.ICP_SETTLE:
            self.publish_vel(0., 0.)
            self.settle_timer -= dt
            if self.settle_timer <= 0.:
                self.icp_fail_count = 0
                return DockingState.ALIGN_HEADING, now
            return state, last_valid_time

        if state == DockingState.FINAL_REVERSE:
            if (odom_x is None or odom_y is None or odom_yaw is None or self.final_reverse_start_x is None):
                return self._fail('FINAL_REVERSE: odom unavailable'), last_valid_time
            moved, lateral_drift = reverse_axis_progress( self.final_reverse_start_x, self.final_reverse_start_y,
                self.final_reverse_start_yaw, odom_x, odom_y)
            if lateral_drift > self.p['final_max_lateral_drift_m']:
                return self._start_recovery('FINAL_REVERSE: excess lateral drift'), last_valid_time
            remaining = max(0., self.final_reverse_distance-moved)
            if self.final_reverse_target_odom_yaw is None:
                return self._fail('FINAL_REVERSE: target Map heading not initialized'), last_valid_time
            yaw_err = wrap_angle(self.final_reverse_target_odom_yaw-odom_yaw)
            if abs(yaw_err) > math.radians(self.p['final_map_yaw_abort_deg']):
                return self._start_recovery('FINAL_REVERSE: yaw drift exceeded safety limit'), last_valid_time
            if remaining <= self.p['final_reverse_tolerance']:
                self.publish_vel(0., 0.)
                # 완료 방향 ±2°는 필수 성공 조건이 아님. 진입 중 안전 Yaw 제한은 유지.
                if self.contact_active or self.get_map_pose() is None:
                    return self._fail('FINAL_REVERSE: contact or lost Map TF at completion'), last_valid_time
                self.get_logger().info('Estimated docking depth reached; physical collision-free entry is not verified')
                return DockingState.COMPLETED, now
            # 최종 도크 안에서는 제자리 회전 금지; 목표 Map Yaw를 Odom에서 유지하며 저속 후진.
            w = clamp(self.p['final_yaw_hold_kp']*yaw_err, -self.p['final_yaw_hold_max_w'], self.p['final_yaw_hold_max_w'])
            slow = clamp(1.0-abs(yaw_err)/math.radians(self.p['final_map_yaw_abort_deg']), .3, 1.)
            self.publish_vel(-self.p['final_reverse_speed']*slow, w)
            return state, now

        age = (now-last_valid_time).nanoseconds/1e9
        if age > self.p['scan_abort_sec']:
            return self._fail('Micro ICP: scan/pose verification timeout'), last_valid_time
        snapshot = self._take_scan_snapshot()
        fresh_verified_scan = False
        if snapshot is not None:
            source_pts, scan_stamp = snapshot
            localized = self.get_map_pose(scan_stamp)
            if localized is None:
                self.publish_vel(0., 0.)
                self.dock_tracker.reset()
                return state, last_valid_time
            map_prior_T = base_to_model_transform(localized, self.model_map_pose)
            first = self.filt_dock_dist is None
            # Spatial gate anchored to MAP, never an unverified ICP result.
            scan = select_scan_near_expected_dock(
                source_pts, self.model_points, map_prior_T,
                gate_radius_m=(self.p['micro_first_gate_radius_m'] if first
                               else self.p['micro_gate_radius_m']))
            if len(scan) >= 12:
                T, _, count = icp_2d( scan, self.target_tree, self.model_points, initial_transform=map_prior_T if first else self.current_transform,
                    max_iter=25 if first else 12, search_radius=.08 if first else .06)
                if (count >= 12 and validate_micro_fit( scan, self.model_points, self.target_tree, T,
                        match_radius_m=self.p['micro_match_radius_m'])):
                    # Geometry fitness is evaluated on filtered points.
                    transformed = scan @ T[:2, :2].T + T[:2, 2]
                    distances, _ = self.target_tree.query(transformed)
                    fitness = float(np.mean( distances < self.p['micro_match_radius_m']))
                    observed_map_pose = observed_model_map_pose(localized, T)
                    accepted_map, residual_m, residual_yaw_deg = accept_map_prior( observed_map_pose, self.model_map_pose,
                        max_xy_m=self.p['map_prior_max_xy_m'], max_yaw_deg=self.p['map_prior_max_yaw_deg'])
                    if not accepted_map:
                        self.dock_tracker.reset()
                        self.icp_fail_count += 1
                        self.publish_vel(0., 0.)
                        self.get_logger().warn( 'Reject ICP vs MAP: XY=%.3fm yaw=%.1fdeg' % (residual_m, residual_yaw_deg))
                        return state, last_valid_time
                    if self._accept_and_filter_icp(T, fitness, now):
                        self.dock_tracker.observe(*observed_map_pose)
                        last_valid_time = now
                        fresh_verified_scan = True
                    else:
                        self.dock_tracker.reset()
                        self.icp_fail_count += 1
                        self.publish_vel(0., 0.)
                        return state, last_valid_time
                else:
                    self.dock_tracker.reset()
                    self.icp_fail_count += 1
                    self.publish_vel(0., 0.)
                    return state, last_valid_time
            else:
                self.dock_tracker.reset()
                self.icp_fail_count += 1
                self.publish_vel(0., 0.)
                return state, last_valid_time
        elif age > self.p['scan_stale_stop_sec']:
            self.publish_vel(0., 0.)
            return state, last_valid_time

        if (self.filt_rel_yaw is None or self.filt_dock_dist is None or self.filt_lateral_error is None):
            self.publish_vel(0., 0.)
            return state, last_valid_time
        # Never change docking state based solely on a cached scan. Hold the
        # previous command for a very short interval, then stop on staleness.
        if not fresh_verified_scan:
            return state, last_valid_time

        # Require 3 consecutive, geometrically valid and MAP-consistent scans
        # before heading alignment/forward reverse proceeds.
        if self.dock_tracker.count < self.p['micro_required_scans']:
            self.publish_vel(0., 0.)
            return state, last_valid_time

        if state == DockingState.ALIGN_HEADING:
            if abs(self.filt_rel_yaw) < math.radians(1.5):
                self.publish_vel(0., 0.)
                self.linear_pid.reset()
                self.linear_pid.prev_err = max( 0., self.filt_dock_dist-self.target_dist)
                self.angular_pid.reset()
                return DockingState.STRAIGHT_REVERSE, last_valid_time
            self.publish_vel(0., self.angular_pid.update(self.filt_rel_yaw, dt))
            return state, last_valid_time

        if state != DockingState.STRAIGHT_REVERSE:
            return self._fail('Unknown micro state'), last_valid_time

        dist_error = max(0., self.filt_dock_dist-self.target_dist)
        lateral, yaw = self.filt_lateral_error, self.filt_rel_yaw
        entry_ok = final_entry_ok( lateral, yaw, self.p['final_entry_lateral_tol'], self.p['final_entry_yaw_tol_deg'])

        if dist_error <= .015:
            if not entry_ok:
                return self._start_recovery('Dock depth reached but entry geometry unsafe'), last_valid_time
            if self.contact_active or self.get_map_pose() is None:
                return self._fail('Dock depth: contact or lost Map TF'), last_valid_time
            self.publish_vel(0., 0.)
            return DockingState.COMPLETED, now

        if self.filt_dock_dist <= self.p['final_blind_start_dist']:
            if not entry_ok:
                return self._start_recovery('Refuse blind reverse: lateral=%.3fm yaw=%.2fdeg' % (lateral, math.degrees(yaw))), last_valid_time
            if odom_x is None or odom_y is None or odom_yaw is None:
                return self._fail('FINAL_REVERSE entry: odom unavailable'), last_valid_time
            heading_ok, map_err, localized = self._check_final_map_heading()
            if not heading_ok:
                return self._start_recovery('FINAL_REVERSE entry: Map heading unsuitable'), last_valid_time
            self.publish_vel(0., 0.)
            self.final_reverse_start_x, self.final_reverse_start_y = odom_x, odom_y
            self.final_reverse_start_yaw = odom_yaw  # 후진 거리 계산용: 실제 진입축
            self.final_reverse_target_odom_yaw = map_target_yaw_to_odom(localized[2], odom_yaw, self.final_map_pose[2])
            self.final_reverse_distance = dist_error
            self.get_logger().info('ICP -> FINAL_REVERSE: remaining=%.3f lat=%.3f yaw=%.1fdeg map_yaw_err=%.1fdeg' %
                                   (dist_error, lateral, math.degrees(yaw), math.degrees(map_err)))
            return DockingState.FINAL_REVERSE, now

        # Differential drive: lateral -> small yaw request, followed by PD yaw.
        # Only the lateral *request* tapers out; heading correction remains.
        target_yaw, scale = yaw_target_from_lateral(
            lateral, self.filt_dock_dist,
            self.p['reverse_lateral_gain'],
            self.p['reverse_max_target_yaw_deg'],
            self.p['reverse_taper_start_dist'],
            self.p['reverse_taper_end_dist'])
        yaw_error = reverse_angular_error(yaw, target_yaw)
        w = self.angular_pid.update(yaw_error, dt)
        v = -self.linear_pid.update(dist_error, dt)
        self.publish_vel(v, w)
        return state, last_valid_time


def main(args=None):
    rp.init(args=args)
    node = PrecisionDockingServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_vel(0., 0.)
        node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()
