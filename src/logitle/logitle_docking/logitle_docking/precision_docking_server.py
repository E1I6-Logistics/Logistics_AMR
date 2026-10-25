import math
import threading

from geometry_msgs.msg import TwistStamped
from logitle_docking.core.docking_state import DockingState
from logitle_docking.core.geometry import (
    generate_v_funnel,
    get_yaw_from_quaternion,
    icp_2d,
    normalize_angle,
)
from logitle_docking.core.pid import PID
import numpy as np
import rclpy as rp
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
import tf2_ros
from turtlebot3_my_msg.action import PrecisionDock

# Re-export core classes & functions for backwards compatibility
__all__ = [
    'DockingState',
    'PID',
    'PrecisionDockingServer',
    'generate_v_funnel',
    'get_yaw_from_quaternion',
    'icp_2d',
    'main',
    'normalize_angle',
]


class PrecisionDockingServer(Node):
    """V자 충전 도크 정밀 진입을 수행하는 Two-Phase 도킹 액션 서버."""

    def __init__(self):
        super().__init__('precision_docking_server')
        self.cb_group = ReentrantCallbackGroup()
        self.scan_lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(
            self,
            PrecisionDock,
            'precision_dock',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = None

        self._declare_and_update_params()
        self._reset_state_variables()

        # [PID 제어기 설정]
        self.linear_pid = PID(p=0.1, i=0.0, d=0.02, out_min=0.005, out_max=0.015)
        self.angular_pid = PID(p=0.5, i=0.0, d=0.10, out_min=-0.30, out_max=0.30)
        self.staging_linear_pid = PID(p=4.0, i=0.01, d=0.02, out_min=0.015, out_max=0.15)
        self.staging_angular_pid = PID(p=2.0, i=0.05, d=0.08, out_min=-0.4, out_max=0.4)

        self.get_logger().info('Two-Phase 정밀 도킹 서버 준비 완료 (Idle 상태)')
        self.get_logger().info('후진 제어: lateral->target yaw + yaw PD (S-curve feedback)')

    def _declare_and_update_params(self):
        """서버 파라미터 선언 및 최신값 갱신."""
        params = {
            'charger_width': 0.145,
            'wing_length': 0.10,
            'wing_angle_deg': 22.5,
            'robot_rear_length': 0.06,
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
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }
        for name, val in params.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, val)

        self.p = {name: self.get_parameter(name).value for name in params.keys()}
        self.wing_rad = math.radians(self.p['wing_angle_deg'])

    def _reset_state_variables(self):
        """도킹 수행 관련 내부 상태 변수 초기화."""
        with self.scan_lock:
            self.latest_source_pts = None
            self.new_scan_available = False

        self.active_target_pts = None
        self.target_tree = None
        self.current_transform = np.identity(3, dtype=np.float32)

        self.filt_rel_yaw = None
        self.filt_dock_dist = None
        self.filt_lateral_error = None

        self.icp_fail_count = 0
        self.settle_timer = 0.0
        self.target_dist = self.p['robot_rear_length']

        # FINAL_REVERSE (ICP OFF)용 odom 기준값
        self.final_reverse_start_x = None
        self.final_reverse_start_y = None
        self.final_reverse_start_yaw = None
        self.final_reverse_distance = 0.0

    def generate_v_funnel(self) -> np.ndarray:
        """현재 파라미터 기반의 V자 충전 도크 CAD 점군 생성."""
        return generate_v_funnel(
            self.p['charger_width'],
            self.p['wing_length'],
            self.wing_rad,
        )

    def scan_callback(self, msg: LaserScan):
        """레이저 스캔 수신 콜백: ROI 영역의 2D 점군 추출."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment

        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])

        roi = (
            (xs < self.p['roi_x_max'])
            & (xs > self.p['roi_x_min'])
            & (np.abs(ys) < self.p['roi_y_limit'])
        )

        if np.sum(roi) >= 10:
            source_pts = np.column_stack((xs[roi], ys[roi])).astype(np.float32)
        else:
            source_pts = None

        with self.scan_lock:
            self.latest_source_pts = source_pts
            self.new_scan_available = source_pts is not None

    def _take_scan_snapshot(self):
        """최신 유효 scan을 스레드 안전하게 복사하여 반환."""
        with self.scan_lock:
            if not self.new_scan_available or self.latest_source_pts is None:
                return None
            source_pts = self.latest_source_pts.copy()
            self.new_scan_available = False
        return source_pts

    def _discard_pending_scan(self):
        """현재 대기 중인 scan 소비 처리."""
        with self.scan_lock:
            self.new_scan_available = False

    def get_odom_pose(self):
        """TF로부터 odom 기준 로봇 2D 위치(x, y, yaw) 조회."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.p['odom_frame'],
                self.p['base_frame'],
                rp.time.Time(),
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                get_yaw_from_quaternion(t.transform.rotation),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None, None, None

    def cancel_callback(self, goal_handle):
        """액션 취소 요청 수락."""
        return CancelResponse.ACCEPT

    def publish_vel(self, v: float, w: float):
        """로봇 선속도 및 각속도 명령 발행."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def execute_callback(self, goal_handle):
        """정밀 도킹 액션 메인 실행 루프."""
        self.get_logger().info('도킹 액션 수신: Two-Phase 제어기 가동')
        self._reset_state_variables()
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
            callback_group=self.cb_group,
        )

        for pid in [
            self.linear_pid,
            self.angular_pid,
            self.staging_linear_pid,
            self.staging_angular_pid,
        ]:
            pid.reset()

        state = DockingState.INIT
        target_odom = {'x': 0.0, 'y': 0.0, 'stg_yaw': 0.0, 'dock_yaw': 0.0}
        rate = self.create_rate(20.0)
        last_time = self.get_clock().now()
        last_valid_time = self.get_clock().now()

        try:
            while rp.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.publish_vel(0.0, 0.0)
                    return PrecisionDock.Result(success=False)

                curr_time = self.get_clock().now()
                dt = max((curr_time - last_time).nanoseconds / 1e9, 0.01)
                last_time = curr_time
                odom_x, odom_y, odom_yaw = self.get_odom_pose()

                # Phase 1: ICP 스냅샷 후 목표 좌표/각도 확정
                if state == DockingState.INIT:
                    state = self._handle_init_phase(
                        goal_handle, odom_x, odom_y, odom_yaw, target_odom
                    )

                # Phase 2: Odom 기반 경유지 전진 주행 및 후진 자세 회전
                elif 1 <= state.value <= 3:
                    if odom_x is None:
                        rate.sleep()
                        continue
                    state = self._handle_macro_drive(
                        state, dt, odom_x, odom_y, odom_yaw, target_odom
                    )

                # Phase 3 & 4: ICP 후진 도킹 + 근거리 odom 최종 후진
                elif state.value >= 4:
                    state, last_valid_time = self._handle_micro_docking(
                        state, dt, curr_time, last_valid_time, odom_x, odom_y, odom_yaw
                    )
                    if state == DockingState.COMPLETED:
                        goal_handle.succeed()
                        return PrecisionDock.Result(success=True)

                rate.sleep()

        finally:
            self.publish_vel(0.0, 0.0)
            if self.scan_sub:
                self.destroy_subscription(self.scan_sub)
            self.get_logger().info('도킹 액션 종료: 리소스 반환 완료')

    def _handle_init_phase(self, goal_handle, odom_x, odom_y, odom_yaw, target_odom):
        """Phase 1: 초기 스캔으로부터 도크를 인식하고 경유지(Staging) Odom 좌표 계산."""
        if odom_x is None:
            return DockingState.INIT

        source_pts = self._take_scan_snapshot()
        if source_pts is None:
            return DockingState.INIT

        req_x = goal_handle.request.target_pose.pose.position.x
        self.target_dist = req_x if req_x > 0 else self.p['robot_rear_length']
        self.active_target_pts = self.generate_v_funnel()
        self.target_tree = cKDTree(self.active_target_pts)

        # 초기 평행이동 추정
        init_t = np.mean(source_pts, axis=0)
        init_T = np.identity(3, dtype=np.float32)
        init_T[:2, 2] = -init_t

        T, fitness, match_cnt = icp_2d(
            source_pts,
            self.target_tree,
            self.active_target_pts,
            initial_transform=init_T,
            max_iter=40,
            search_radius=0.8,
        )

        if match_cnt < 10:
            self.get_logger().warn(
                f'초기 ICP 도크 인식 실패: match={match_cnt}, fitness={fitness:.3f}'
            )
            return DockingState.INIT

        self.get_logger().info(
            f'초기 ICP 도크 인식 성공: match={match_cnt}, fitness={fitness:.3f}'
        )

        R, t = T[:2, :2], T[:2, 2]
        rel_x = float(-R[0, 0] * t[0] - R[1, 0] * t[1])
        rel_y = float(-R[0, 1] * t[0] - R[1, 1] * t[1])
        dock_yaw_in_base = math.atan2(R[0, 1], R[0, 0])

        dock_odom_x = odom_x + rel_x * math.cos(odom_yaw) - rel_y * math.sin(odom_yaw)
        dock_odom_y = odom_y + rel_x * math.sin(odom_yaw) + rel_y * math.cos(odom_yaw)
        dock_odom_yaw = normalize_angle(odom_yaw + dock_yaw_in_base)

        stg_dist = self.p['staging_distance']
        target_odom['x'] = dock_odom_x + stg_dist * math.cos(dock_odom_yaw)
        target_odom['y'] = dock_odom_y + stg_dist * math.sin(dock_odom_yaw)
        target_odom['stg_yaw'] = math.atan2(target_odom['y'] - odom_y, target_odom['x'] - odom_x)
        target_odom['dock_yaw'] = dock_odom_yaw

        self.get_logger().info(
            f'도크 Odom 추정: x={dock_odom_x:.2f}, y={dock_odom_y:.2f}, '
            f'도크방향={math.degrees(dock_odom_yaw):.1f}°'
        )
        self.get_logger().info(
            f'경유지 설정: x={target_odom["x"]:.2f}, y={target_odom["y"]:.2f}'
        )
        return DockingState.STAGING_TURN

    def _handle_macro_drive(self, state, dt, x, y, yaw, tgt):
        """Phase 2: Odom 기반 경유지 전진 주행 및 후진 자세 정렬."""
        if state == DockingState.STAGING_TURN:
            err = normalize_angle(tgt['stg_yaw'] - yaw)
            if abs(err) < math.radians(1.5):
                self.publish_vel(0.0, 0.0)
                self.staging_angular_pid.reset()
                return DockingState.STAGING_DRIVE

            w = self.staging_angular_pid.update(err, dt)
            self.publish_vel(0.0, w)

        elif state == DockingState.STAGING_DRIVE:
            dist = math.hypot(tgt['x'] - x, tgt['y'] - y)
            if dist < 0.02:
                self.publish_vel(0.0, 0.0)
                self.staging_linear_pid.reset()
                self.staging_angular_pid.reset()
                self.get_logger().info(f'경유지 도착: dist={dist:.3f}m')
                return DockingState.STAGING_ALIGN

            v = self.staging_linear_pid.update(dist, dt)
            self.publish_vel(v, 0.0)

        elif state == DockingState.STAGING_ALIGN:
            err = normalize_angle(tgt['dock_yaw'] - yaw)
            if abs(err) < math.radians(1.0):
                self.publish_vel(0.0, 0.0)
                self._discard_pending_scan()

                self.current_transform = np.identity(3, dtype=np.float32)
                self.current_transform[0, 2] = self.p['staging_distance']

                self.filt_dock_dist = None
                self.filt_rel_yaw = None
                self.filt_lateral_error = None
                self.settle_timer = 0.5
                self.staging_angular_pid.reset()
                return DockingState.ICP_SETTLE

            w = self.staging_angular_pid.update(err, dt)
            self.publish_vel(0.0, w)

        return state

    def _handle_micro_docking(self, state, dt, curr_time, valid_time,
                              odom_x, odom_y, odom_yaw):
        """Phase 3 & 4: 모션 정지 대기, 근거리 최종 후진, ICP 정밀 후진 제어 총괄."""
        if state == DockingState.ICP_SETTLE:
            self.publish_vel(0.0, 0.0)
            self.settle_timer -= dt
            if self.settle_timer <= 0:
                self.get_logger().info('경유지 도착: 후진 도킹 시작')
                self.icp_fail_count = 0
                return DockingState.ALIGN_HEADING, curr_time
            return state, valid_time

        if state == DockingState.FINAL_REVERSE:
            return self._step_final_reverse(odom_x, odom_y, odom_yaw, curr_time, valid_time)

        # 도크 접점 도달 완료 여부 선검사
        if state == DockingState.STRAIGHT_REVERSE and self.filt_dock_dist is not None:
            dist_error = max(0.0, self.filt_dock_dist - self.target_dist)
            if dist_error <= 0.015:
                self.publish_vel(0.0, 0.0)
                self.get_logger().info(
                    f'도크 접점 도달 완료 (dock_dist={self.filt_dock_dist:.3f}m, '
                    f'target={self.target_dist:.3f}m, 남은오차={dist_error:.3f}m). 도킹 완료.'
                )
                return DockingState.COMPLETED, curr_time

        # 최신 스캔 기반 ICP 포즈 추정 및 이상값 필터링
        icp_ok, valid_time = self._track_icp_pose(curr_time, valid_time)
        if not icp_ok:
            return state, valid_time

        # ICP 타임아웃 검사 (1.0초 이상 정상 정합 부재 시 정지)
        if (curr_time - valid_time).nanoseconds / 1e9 > 1.0:
            self.publish_vel(0.0, 0.0)
            return state, valid_time

        # 필터링된 포즈를 바탕으로 이동 제어 실행
        if (self.filt_rel_yaw is not None and self.filt_dock_dist is not None and
                self.filt_lateral_error is not None):
            return self._execute_micro_motion(
                state, dt, odom_x, odom_y, odom_yaw, curr_time, valid_time
            )

        return state, valid_time

    def _step_final_reverse(self, odom_x, odom_y, odom_yaw, curr_time, valid_time):
        """Phase 4: 근거리 Odom 기반 직선 저속 후진 및 각도 유지."""
        if odom_x is None or odom_y is None or odom_yaw is None:
            self.publish_vel(0.0, 0.0)
            return DockingState.FINAL_REVERSE, valid_time

        if (self.final_reverse_start_x is None or self.final_reverse_start_y is None or
                self.final_reverse_start_yaw is None):
            self.publish_vel(0.0, 0.0)
            self.get_logger().warn('FINAL_REVERSE 시작 odom/yaw가 없어 정지합니다.')
            return DockingState.FINAL_REVERSE, valid_time

        moved = math.hypot(
            odom_x - self.final_reverse_start_x,
            odom_y - self.final_reverse_start_y,
        )
        remaining = max(0.0, self.final_reverse_distance - moved)

        if remaining <= self.p['final_reverse_tolerance']:
            self.publish_vel(0.0, 0.0)
            self.get_logger().info(
                f'최종 직선 후진 완료 -> 도킹 완료: moved={moved:.3f}m, '
                f'target_move={self.final_reverse_distance:.3f}m'
            )
            return DockingState.COMPLETED, curr_time

        yaw_error = normalize_angle(self.final_reverse_start_yaw - odom_yaw)
        w_cmd = self.p['final_yaw_hold_kp'] * yaw_error
        w_cmd = float(np.clip(
            w_cmd,
            -self.p['final_yaw_hold_max_w'],
            self.p['final_yaw_hold_max_w'],
        ))

        self.publish_vel(-self.p['final_reverse_speed'], w_cmd)
        return DockingState.FINAL_REVERSE, curr_time

    def _track_icp_pose(self, curr_time, valid_time):
        """스캔 스냅샷을 가져와 ICP를 수행하고, 이상값 검사 및 지수평활 필터 갱신."""
        source_pts = self._take_scan_snapshot()
        if source_pts is None:
            return True, valid_time

        s_rad, max_iter = (0.15, 25) if self.filt_dock_dist is None else (0.10, 12)
        T, fitness, match_cnt = icp_2d(
            source_pts,
            self.target_tree,
            self.active_target_pts,
            self.current_transform,
            max_iter,
            s_rad,
        )

        if match_cnt < 10:
            self.icp_fail_count += 1
            self.get_logger().warn(
                f'ICP match 부족: match={match_cnt}, fitness={fitness:.3f}, '
                f'fail={self.icp_fail_count}'
            )
            self.publish_vel(0.0, 0.0)
            if self.icp_fail_count > 3:
                self.get_logger().warn('ICP match 연속 실패 - 현재 위치에서 재획득 대기')
                self.icp_fail_count = 0
            return False, valid_time

        R, t = T[:2, :2], T[:2, 2]
        raw_dist = float(t[0])
        raw_lateral = float(t[1])
        raw_yaw = normalize_angle(math.atan2(R[0, 1], R[0, 0]))

        # 이상값 점프 검사
        if self.filt_dock_dist is None:
            exp_R = self.current_transform[:2, :2]
            exp_dist = float(self.current_transform[0, 2])
            exp_lateral = float(self.current_transform[1, 2])
            exp_yaw = normalize_angle(math.atan2(exp_R[0, 1], exp_R[0, 0]))

            dist_jump = abs(raw_dist - exp_dist)
            lateral_jump = abs(raw_lateral - exp_lateral)
            yaw_jump = abs(normalize_angle(raw_yaw - exp_yaw))
        else:
            dist_jump = abs(raw_dist - self.filt_dock_dist)
            lateral_jump = abs(raw_lateral - self.filt_lateral_error)
            yaw_jump = abs(normalize_angle(raw_yaw - self.filt_rel_yaw))

        icp_valid = (
            dist_jump <= 0.10
            and lateral_jump <= 0.05
            and yaw_jump <= math.radians(10.0)
        )

        if not icp_valid:
            self.icp_fail_count += 1
            self.get_logger().warn(
                f'ICP 이상값 거부: dist={raw_dist:.3f}, lateral={raw_lateral:.3f}, '
                f'yaw={math.degrees(raw_yaw):.1f}°, dDist={dist_jump:.3f}, '
                f'dLat={lateral_jump:.3f}, dYaw={math.degrees(yaw_jump):.1f}°, '
                f'fail={self.icp_fail_count}'
            )
            self.publish_vel(0.0, 0.0)
            if self.icp_fail_count > 3:
                self.get_logger().warn('ICP 연속 실패 - 현재 위치에서 재획득 대기')
                self.icp_fail_count = 0
            return False, valid_time

        # 정상 정합 처리 및 지수 이동평균 필터 갱신
        self.icp_fail_count = 0
        self.current_transform = T
        alpha = 0.6 if fitness > 0.6 else (0.4 if fitness > 0.3 else 0.2)

        if self.filt_rel_yaw is None:
            self.filt_rel_yaw = raw_yaw
        else:
            yaw_delta = normalize_angle(raw_yaw - self.filt_rel_yaw)
            self.filt_rel_yaw = normalize_angle(self.filt_rel_yaw + alpha * yaw_delta)

        if self.filt_dock_dist is None:
            self.filt_dock_dist = raw_dist
        else:
            self.filt_dock_dist = alpha * raw_dist + (1.0 - alpha) * self.filt_dock_dist

        if self.filt_lateral_error is None:
            self.filt_lateral_error = raw_lateral
        else:
            self.filt_lateral_error = alpha * raw_lateral + (1.0 - alpha) * self.filt_lateral_error

        return True, curr_time

    def _execute_micro_motion(self, state, dt, odom_x, odom_y, odom_yaw,
                              curr_time, valid_time):
        """Phase 3: ALIGN_HEADING 및 STRAIGHT_REVERSE 모션 제어 연산."""
        if state == DockingState.ALIGN_HEADING:
            if abs(self.filt_rel_yaw) < math.radians(1.5):
                self.publish_vel(0.0, 0.0)
                self.linear_pid.reset()
                init_err = max(0.0, self.filt_dock_dist - self.target_dist)
                self.linear_pid.prev_err = init_err
                return DockingState.STRAIGHT_REVERSE, valid_time

            self.publish_vel(0.0, self.angular_pid.update(self.filt_rel_yaw, dt))
            return state, valid_time

        if state == DockingState.STRAIGHT_REVERSE:
            # 근거리 진입 시 Odom 기반 FINAL_REVERSE로 전환
            if self.filt_dock_dist <= self.p['final_blind_start_dist']:
                if odom_x is None or odom_y is None or odom_yaw is None:
                    self.publish_vel(0.0, 0.0)
                    return state, valid_time

                self.publish_vel(0.0, 0.0)
                self.final_reverse_start_x = odom_x
                self.final_reverse_start_y = odom_y
                self.final_reverse_start_yaw = odom_yaw
                self.final_reverse_distance = max(0.0, self.filt_dock_dist - self.target_dist)

                self.get_logger().info(
                    f'ICP 종료 -> 최종 직선 후진 시작: dock_dist={self.filt_dock_dist:.3f}m, '
                    f'target={self.target_dist:.3f}m, 남은후진={self.final_reverse_distance:.3f}m, '
                    f'speed={self.p["final_reverse_speed"]:.3f}m/s'
                )

                if self.final_reverse_distance <= self.p['final_reverse_tolerance']:
                    self.get_logger().info('추가 후진 거리 없음 -> 도킹 완료')
                    return DockingState.COMPLETED, curr_time

                return DockingState.FINAL_REVERSE, curr_time

            dist_error = max(0.0, self.filt_dock_dist - self.target_dist)
            if dist_error <= 0.015:
                self.publish_vel(0.0, 0.0)
                self.get_logger().info(f'도크 접점 도달 완료 (남은오차: {dist_error:.3f}m). 도킹 완료.')
                return DockingState.COMPLETED, curr_time

            v = -self.linear_pid.update(dist_error, dt)

            # Lateral 오차 -> 목표 Yaw (S-curve 조향)
            lateral_gain = self.p['reverse_lateral_gain']
            max_target_yaw = math.radians(self.p['reverse_max_target_yaw_deg'])
            target_yaw = float(np.clip(
                lateral_gain * self.filt_lateral_error,
                -max_target_yaw,
                max_target_yaw,
            ))
            yaw_control_error = normalize_angle(self.filt_rel_yaw + target_yaw)

            # 도크가 가까워질수록 steering scale 서서히 감소 (0.0 ~ 1.0)
            steering_start_dist = self.p['steering_lock_dist']
            steering_stop_dist = self.target_dist + 0.015

            if self.filt_dock_dist >= steering_start_dist:
                steering_scale = 1.0
            elif self.filt_dock_dist <= steering_stop_dist:
                steering_scale = 0.0
            else:
                steering_scale = (
                    (self.filt_dock_dist - steering_stop_dist) /
                    (steering_start_dist - steering_stop_dist)
                )

            w = steering_scale * self.angular_pid.update(yaw_control_error, dt)
            self.publish_vel(v, w)
            return state, valid_time

        self.get_logger().info('도킹 상태 오류: 알 수 없는 상태')
        return state, valid_time


def main(args=None):
    """정밀 도킹 액션 서버 실행 진입점."""
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
