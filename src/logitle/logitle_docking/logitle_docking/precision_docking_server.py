import math
import threading
import numpy as np
from enum import Enum
from scipy.spatial import cKDTree

import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

import tf2_ros
from turtlebot3_my_msg.action import PrecisionDock


class DockingState(Enum):
    INIT = 0
    STAGING_TURN = 1       # [Odom] 경유지를 향해 제자리 회전 (전진 조준)
    STAGING_DRIVE = 2      # [Odom] 경유지로 직진 주행 (전진)
    STAGING_ALIGN = 3      # [Odom] 도크 진입축 정렬 (도크를 등지는 후진 자세 회전)
    ICP_SETTLE = 4         # 모션 스큐 방지 정지 대기
    ALIGN_HEADING = 5      # [Pure ICP] 근거리 각도 정밀 정렬
    STRAIGHT_REVERSE = 6   # [Pure ICP] 도크 안으로 정밀 감속 후진
    FINAL_REVERSE = 7      # [Odom] 근거리에서 ICP를 끄고 저속 직선 후진 + yaw 유지
    COMPLETED = 8


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def icp_2d(source_pts, target_tree, target_pts, initial_transform=None, max_iter=20, search_radius=0.5):
    T = np.identity(3, dtype=np.float32) if initial_transform is None else np.copy(initial_transform)
    prev_error = float('inf')

    for _ in range(max_iter):
        R_current, t_current = T[:2, :2], T[:2, 2]
        transformed = (source_pts @ R_current.T) + t_current
        distances, indices = target_tree.query(transformed, workers=1)

        valid = distances < search_radius
        matched_count = int(np.sum(valid))
        if matched_count < 10:
            break

        pts_src, pts_tgt = source_pts[valid], target_pts[indices[valid]]
        centroid_src, centroid_tgt = np.mean(pts_src, axis=0), np.mean(pts_tgt, axis=0)

        H = (pts_src - centroid_src).T @ (pts_tgt - centroid_tgt)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_tgt - (R @ centroid_src)
        current_error = np.sum(np.abs(T[:2, :2] - R)) + np.sum(np.abs(T[:2, 2] - t))

        T[:2, :2], T[:2, 2] = R, t
        if abs(prev_error - current_error) < 1e-4:
            break
        prev_error = current_error

    fitness = float(np.mean(distances < 0.08)) if len(distances) > 0 else 0.0
    return T, fitness, matched_count


class PID:
    def __init__(self, p, i, d, out_min, out_max):
        self.p, self.i, self.d = p, i, d
        self.out_min, self.out_max = out_min, out_max
        self.prev_err, self.integral = 0.0, 0.0

    def update(self, error, dt=0.05):
        dt = max(dt, 0.01)
        self.integral += error * dt
        output = (self.p * error) + (self.i * self.integral) + (self.d * (error - self.prev_err) / dt)

        if output > self.out_max or output < self.out_min:
            self.integral -= error * dt
        self.prev_err = error
        return float(np.clip(output, self.out_min, self.out_max))

    def reset(self):
        self.prev_err, self.integral = 0.0, 0.0


class PrecisionDockingServer(Node):
    def __init__(self):
        super().__init__('precision_docking_server')
        self.cb_group = ReentrantCallbackGroup()
        self.scan_lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(
            self, PrecisionDock, 'precision_dock',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
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
        params = {
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
            'odom_frame': 'odom', 
            'base_frame': 'base_footprint'
        }
        for name, val in params.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, val)
                
        self.p = {name: self.get_parameter(name).value for name in params.keys()}
        self.wing_rad = math.radians(self.p['wing_angle_deg'])

    def _reset_state_variables(self):
        # /scan callback과 action callback이 공유하는 데이터는 lock으로 보호
        with self.scan_lock:
            self.latest_source_pts = None
            self.new_scan_available = False

        self.target_tree = None
        self.current_transform = np.identity(3, dtype=np.float32)
        self.filt_rel_yaw = None
        self.filt_dock_dist = None
        self.filt_lateral_error = None

        self.prev_yaw = None
        self.prev_dist = None
        self.prev_lateral = None
        self.last_fitness, self.icp_fail_count, self.settle_timer = 0.0, 0, 0.0
        self.target_dist = self.p['robot_rear_length']

        # FINAL_REVERSE(ICP OFF)용 odom 기준값
        self.final_reverse_start_x = None
        self.final_reverse_start_y = None
        self.final_reverse_start_yaw = None
        self.final_reverse_distance = 0.0

    def generate_v_funnel(self):
        pts = []
        half_w = self.p['charger_width'] / 2.0
        
        for y in np.arange(-half_w, half_w + 0.001, 0.005):
            pts.append([0.0, y])
            
        for l in np.arange(0.01, self.p['wing_length'] + 0.005, 0.025):
            px = l * math.cos(self.wing_rad)
            py = half_w + l * math.sin(self.wing_rad)
            pts.extend([[px, py], [px, -py]])
            
        return np.array(pts, dtype=np.float32)

    def scan_callback(self, msg: LaserScan):
        # 무거운 LaserScan -> XY/ROI 계산은 lock 밖에서 수행한다.
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

        # 공유 변수 교체만 아주 짧게 lock으로 보호한다.
        with self.scan_lock:
            self.latest_source_pts = source_pts
            self.new_scan_available = source_pts is not None

    def _take_scan_snapshot(self):
        """
        최신 유효 scan을 독립적인 numpy 배열로 복사해서 가져온다.

        lock 안에서는 공유 변수 확인/복사/소비 플래그 변경만 하고,
        ICP 계산은 반드시 lock 밖에서 수행한다.
        """
        with self.scan_lock:
            if not self.new_scan_available or self.latest_source_pts is None:
                return None

            source_pts = self.latest_source_pts.copy()
            self.new_scan_available = False

        return source_pts

    def _discard_pending_scan(self):
        """현재 대기 중인 scan만 소비 처리한다. 배열 자체는 callback이 관리한다."""
        with self.scan_lock:
            self.new_scan_available = False

    def get_odom_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.p['odom_frame'], self.p['base_frame'], rp.time.Time())
            return t.transform.translation.x, t.transform.translation.y, get_yaw_from_quaternion(t.transform.rotation)
        except:
            return None, None, None

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def publish_vel(self, v, w):
        msg = TwistStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd_vel_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('도킹 액션 수신: Two-Phase 제어기 가동')
        self._reset_state_variables()
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
            callback_group=self.cb_group
        )
        for pid in [self.linear_pid, self.angular_pid, self.staging_linear_pid, self.staging_angular_pid]:
            pid.reset()

        state = DockingState.INIT
        active_target_pts, target_odom = None, {'x': 0, 'y': 0, 'stg_yaw': 0, 'dock_yaw': 0}
        last_valid_time, rate, last_time = self.get_clock().now(), self.create_rate(20.0), self.get_clock().now()

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

                # Phase 1: ICP 스냅샷 후 목표 좌표/각도 확정
                if state == DockingState.INIT:
                    if odom_x is None:
                        rate.sleep()
                        continue

                    # Lock 안에서 scan 한 장을 복사하고, 이후 INIT ICP는
                    # 이 snapshot 하나만 사용한다.
                    source_pts = self._take_scan_snapshot()
                    if source_pts is None:
                        rate.sleep()
                        continue

                    req_x = goal_handle.request.target_pose.pose.position.x
                    self.target_dist = req_x if req_x > 0 else self.p['robot_rear_length']
                    active_target_pts = self.generate_v_funnel()
                    self.target_tree = cKDTree(active_target_pts)

                    # 초기 평행이동 추정도 같은 snapshot을 사용
                    init_t = np.mean(source_pts, axis=0)
                    init_T = np.identity(3, dtype=np.float32)
                    init_T[:2, 2] = -init_t

                    T, fitness, match_cnt = icp_2d(
                        source_pts,
                        self.target_tree,
                        active_target_pts,
                        initial_transform=init_T,
                        max_iter=40,
                        search_radius=0.8
                    )

                    # 실제 V 모델(wing_angle_deg=22.5°)에 대한 ICP 매칭만 사용한다.
                    # 별도의 L/C/R 형상 강제 검사는 사용하지 않는다.
                    if match_cnt >= 10:
                        self.get_logger().info(
                            f'초기 ICP 도크 인식 성공: '
                            f'match={match_cnt}, fitness={fitness:.3f}'
                        )

                        # 기존 도크 위치 계산
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

                        self.get_logger().info(f'도크 Odom 추정: x={dock_odom_x:.2f}, y={dock_odom_y:.2f}, 도크방향={math.degrees(dock_odom_yaw):.1f}°')
                        self.get_logger().info(f'경유지 설정: x={target_odom["x"]:.2f}, y={target_odom["y"]:.2f}')
                        state = DockingState.STAGING_TURN
                    else:
                        self.get_logger().warn(
                            f'초기 ICP 도크 인식 실패: '
                            f'match={match_cnt}, fitness={fitness:.3f}'
                        )

                # Phase 2: Odom 기반 경유지 전진 주행 및 후진 자세 회전
                elif 1 <= state.value <= 3:
                    if odom_x is None:
                        rate.sleep()
                        continue
                    state = self._handle_macro_drive(state, dt, odom_x, odom_y, odom_yaw, target_odom)

                # Phase 3 & 4: ICP 후진 도킹 + 근거리 odom 최종 후진
                elif state.value >= 4:
                    state, last_valid_time = self._handle_micro_docking(
                        state, dt, curr_time, last_valid_time,
                        active_target_pts, odom_x, odom_y, odom_yaw
                    )
                    if state == DockingState.COMPLETED:
                        goal_handle.succeed()
                        return PrecisionDock.Result(success=True)

                rate.sleep()

        finally:
            self.publish_vel(0, 0)
            if self.scan_sub:
                self.destroy_subscription(self.scan_sub)
            self.get_logger().info('도킹 액션 종료: 리소스 반환 완료')

    def _handle_macro_drive(self, state, dt, x, y, yaw, tgt):
        # ---------------------------------------------------------
        # 경유지 제어는 안정적이었던 기존 방식으로 원상복구
        # 1) 경유지 방향으로 제자리 회전
        # 2) 경유지까지 직선 전진 (주행 중 추가 heading 보정 없음)
        # 3) 경유지 도착 후 도크 진입축으로 제자리 정렬
        # ---------------------------------------------------------
        if state == DockingState.STAGING_TURN:
            err = normalize_angle(tgt['stg_yaw'] - yaw)

            # 기존 허용오차 1.5도로 복구
            if abs(err) < math.radians(1.5):
                self.publish_vel(0.0, 0.0)
                self.staging_angular_pid.reset()
                return DockingState.STAGING_DRIVE

            w = self.staging_angular_pid.update(err, dt)
            self.publish_vel(0.0, w)

        elif state == DockingState.STAGING_DRIVE:
            dist = math.hypot(tgt['x'] - x, tgt['y'] - y)

            # 기존 경유지 도착 조건 2 cm로 복구
            if dist < 0.02:
                self.publish_vel(0.0, 0.0)
                self.staging_linear_pid.reset()
                self.staging_angular_pid.reset()
                self.get_logger().info(
                    f'경유지 도착: dist={dist:.3f}m'
                )
                return DockingState.STAGING_ALIGN

            # 기존 방식: 최초 STAGING_TURN에서 맞춘 방향으로 직선 전진
            v = self.staging_linear_pid.update(dist, dt)
            self.publish_vel(v, 0.0)

        elif state == DockingState.STAGING_ALIGN:
            err = normalize_angle(tgt['dock_yaw'] - yaw)

            # 기존 1.0도 허용오차로 복구
            if abs(err) < math.radians(1.0):
                self.publish_vel(0.0, 0.0)
                self._discard_pending_scan()

                # 이 부분은 후진 ICP 안정화를 위해 유지:
                # 경유지에서는 도크가 staging_distance만큼 떨어져 있다고 보고
                # micro ICP 초기 transform을 예상 거리에서 시작한다.
                self.current_transform = np.identity(3, dtype=np.float32)
                self.current_transform[0, 2] = self.p['staging_distance']

                self.filt_dock_dist = None
                self.filt_rel_yaw = None
                self.filt_lateral_error = None
                self.prev_dist = None
                self.prev_yaw = None
                self.prev_lateral = None
                self.settle_timer = 0.5
                self.staging_angular_pid.reset()
                return DockingState.ICP_SETTLE

            w = self.staging_angular_pid.update(err, dt)
            self.publish_vel(0.0, w)

        return state

    def _handle_micro_docking(
        self, state, dt, curr_time, valid_time,
        active_pts, odom_x, odom_y, odom_yaw
    ):
        if state == DockingState.ICP_SETTLE:
            self.publish_vel(0, 0)
            self.settle_timer -= dt
            if self.settle_timer <= 0:
                self.get_logger().info('경유지 도착: 후진 도킹 시작')
                # [FIX 2] micro docking 시작 시점부터 ICP timeout을 다시 센다.
                self.icp_fail_count = 0
                return DockingState.ALIGN_HEADING, curr_time

        elif state == DockingState.FINAL_REVERSE:
            # ---------------------------------------------------------
            # 근거리 최종 후진:
            # ICP는 완전히 사용하지 않고, FINAL_REVERSE 진입 순간의 yaw를
            # odom 기준으로 유지하면서 남은 거리만 저속 후진한다.
            # ---------------------------------------------------------
            if odom_x is None or odom_y is None or odom_yaw is None:
                self.publish_vel(0.0, 0.0)
                return state, valid_time

            if (
                self.final_reverse_start_x is None
                or self.final_reverse_start_y is None
                or self.final_reverse_start_yaw is None
            ):
                self.publish_vel(0.0, 0.0)
                self.get_logger().warn('FINAL_REVERSE 시작 odom/yaw가 없어 정지합니다.')
                return state, valid_time

            moved = math.hypot(
                odom_x - self.final_reverse_start_x,
                odom_y - self.final_reverse_start_y
            )
            remaining = max(0.0, self.final_reverse_distance - moved)

            if remaining <= self.p['final_reverse_tolerance']:
                self.publish_vel(0.0, 0.0)
                self.get_logger().info(
                    f'최종 직선 후진 완료 -> 도킹 완료: '
                    f'moved={moved:.3f}m, '
                    f'target_move={self.final_reverse_distance:.3f}m'
                )
                return DockingState.COMPLETED, curr_time

            # FINAL_REVERSE 시작 순간의 yaw만 유지한다.
            # 초기 도크 인식 yaw로 마지막 제자리 회전은 하지 않는다.
            yaw_error = normalize_angle(
                self.final_reverse_start_yaw - odom_yaw
            )
            w_cmd = self.p['final_yaw_hold_kp'] * yaw_error
            w_cmd = float(np.clip(
                w_cmd,
                -self.p['final_yaw_hold_max_w'],
                self.p['final_yaw_hold_max_w']
            ))

            self.publish_vel(-self.p['final_reverse_speed'], w_cmd)
            return state, curr_time

        else:
            # [FIX 7] 완료조건 선검사
            # 후진 중에는 새 ICP보다 먼저 마지막 정상 ICP 거리로 완료 여부를 확인한다.
            # 도크에 아주 가까워진 뒤 새 scan/ICP가 튀어도, 아래의 ICP reject return 때문에
            # 완료 조건이 실행되지 못하는 상황을 방지한다.
            if (
                state == DockingState.STRAIGHT_REVERSE
                and self.filt_dock_dist is not None
            ):
                dist_error = max(
                    0.0,
                    self.filt_dock_dist - self.target_dist
                )

                if dist_error <= 0.015:
                    self.publish_vel(0.0, 0.0)
                    self.get_logger().info(
                        f'도크 접점 도달 완료 ' 
                        f'(dock_dist={self.filt_dock_dist:.3f}m, ' 
                        f'target={self.target_dist:.3f}m, ' 
                        f'남은오차={dist_error:.3f}m). ' 
                        f'도킹 완료.'
                    )
                    return DockingState.COMPLETED, curr_time

            # 매 제어 주기마다 최신 scan 한 장만 snapshot으로 가져온다.
            # 이후 ICP가 도는 동안 scan_callback이 새 데이터를 써도
            # 현재 ICP 입력은 변하지 않는다.
            source_pts = self._take_scan_snapshot()

            if source_pts is not None:
                # [FIX 3] 이전 1.5m/0.5m는 주변 구조물까지 correspondence 후보가 되어
                # 잘못된 각도(예: 28~50도)로 수렴할 수 있으므로 탐색 반경을 제한한다.
                s_rad, max_iter = (0.15, 25) if self.filt_dock_dist is None else (0.10, 12)
                T, fitness, match_cnt = icp_2d(
                    source_pts,
                    self.target_tree,
                    active_pts,
                    self.current_transform,
                    max_iter,
                    s_rad
                )

                if match_cnt >= 10:
                    R, t = T[:2, :2], T[:2, 2]

                    raw_dist = float(t[0])
                    raw_lateral = float(t[1])

                    raw_yaw = normalize_angle(
                        math.atan2(R[0, 1], R[0, 0])
                    )

                    # -----------------------------
                    # ICP 이상값 검사
                    # -----------------------------
                    if self.prev_dist is None:
                        # [FIX 4] 첫 ICP/재획득 ICP도 무조건 신뢰하지 않는다.
                        # current_transform은 STAGING_ALIGN의 예상 pose 또는 마지막 정상 ICP pose다.
                        expected_R = self.current_transform[:2, :2]
                        expected_dist = float(self.current_transform[0, 2])
                        expected_lateral = float(self.current_transform[1, 2])
                        expected_yaw = normalize_angle(
                            math.atan2(expected_R[0, 1], expected_R[0, 0])
                        )

                        dist_jump = abs(raw_dist - expected_dist)
                        lateral_jump = abs(raw_lateral - expected_lateral)
                        yaw_jump = abs(normalize_angle(raw_yaw - expected_yaw))

                        icp_valid = (
                            dist_jump <= 0.10
                            and lateral_jump <= 0.05
                            and yaw_jump <= math.radians(10.0)
                        )

                    else:
                        dist_jump = abs(
                            raw_dist - self.prev_dist
                        )

                        lateral_jump = abs(
                            raw_lateral - self.prev_lateral
                        )

                        yaw_jump = abs(
                            normalize_angle(
                                raw_yaw - self.prev_yaw
                            )
                        )

                        icp_valid = (
                            dist_jump <= 0.10
                            and lateral_jump <= 0.05
                            and yaw_jump <= math.radians(10.0)
                        )

                    # -----------------------------
                    # 정상 ICP
                    # -----------------------------
                    if icp_valid:
                        valid_time = curr_time
                        self.icp_fail_count = 0
                        self.current_transform = T

                        alpha = (
                            0.6 if fitness > 0.6
                            else 0.4 if fitness > 0.3
                            else 0.2
                        )

                        # Yaw 필터
                        if self.filt_rel_yaw is None:
                            self.filt_rel_yaw = raw_yaw
                        else:
                            yaw_delta = normalize_angle(
                                raw_yaw - self.filt_rel_yaw
                            )

                            self.filt_rel_yaw = normalize_angle(
                                self.filt_rel_yaw
                                + alpha * yaw_delta
                            )

                        # 거리 필터
                        if self.filt_dock_dist is None:
                            self.filt_dock_dist = raw_dist
                        else:
                            self.filt_dock_dist = (
                                alpha * raw_dist
                                + (1.0 - alpha)
                                * self.filt_dock_dist
                            )

                        # 좌우 오차 필터
                        if self.filt_lateral_error is None:
                            self.filt_lateral_error = raw_lateral
                        else:
                            self.filt_lateral_error = (
                                alpha * raw_lateral
                                + (1.0 - alpha)
                                * self.filt_lateral_error
                            )

                        self.prev_dist = self.filt_dock_dist
                        self.prev_yaw = self.filt_rel_yaw
                        self.prev_lateral = self.filt_lateral_error

                        self.last_fitness = fitness

                    # -----------------------------
                    # match는 됐지만 ICP가 갑자기 튐
                    # -----------------------------
                    else:
                        self.icp_fail_count += 1

                        self.get_logger().warn(
                            f'ICP 이상값 거부: '
                            f'dist={raw_dist:.3f}, '
                            f'lateral={raw_lateral:.3f}, '
                            f'yaw={math.degrees(raw_yaw):.1f}°, '
                            f'dDist={dist_jump:.3f}, '
                            f'dLat={lateral_jump:.3f}, '
                            f'dYaw={math.degrees(yaw_jump):.1f}°, '
                            f'fail={self.icp_fail_count}'
                        )

                        # [FIX 6] 이상 ICP가 나온 순간에는 이전 필터값으로 계속 후진하지 않는다.
                        # 즉시 정지하고, 마지막 정상 transform/filter를 유지한 채 다음 scan에서 재획득한다.
                        self.publish_vel(0.0, 0.0)

                        if self.icp_fail_count > 3:
                            self.get_logger().warn(
                                'ICP 연속 실패 - 현재 위치에서 재획득 대기'
                            )
                            self.icp_fail_count = 0

                        return state, valid_time

                # ---------------------------------
                # matching 자체가 제대로 안 됨
                # ---------------------------------
                else:
                    self.icp_fail_count += 1

                    self.get_logger().warn(
                        f'ICP match 부족: '
                        f'match={match_cnt}, '
                        f'fitness={fitness:.3f}, '
                        f'fail={self.icp_fail_count}'
                    )

                    # [FIX 6] match 부족 시에도 stale filter 값으로 계속 후진하지 않는다.
                    # 즉시 정지하고 마지막 정상 transform/filter를 유지한 채 다음 scan을 기다린다.
                    self.publish_vel(0.0, 0.0)

                    if self.icp_fail_count > 3:
                        self.get_logger().warn(
                            'ICP match 연속 실패 - 현재 위치에서 재획득 대기'
                        )
                        self.icp_fail_count = 0

                    return state, valid_time
            if (curr_time - valid_time).nanoseconds / 1e9 > 1.0:
                self.publish_vel(0, 0)
                return state, valid_time

            if (self.filt_rel_yaw is not None and self.filt_dock_dist is not None and self.filt_lateral_error is not None):
                if state == DockingState.ALIGN_HEADING:
                    # 회전 헌팅 방지를 위해 1.5도로 여유를 부여하여 후진 진입 유도
                    if abs(self.filt_rel_yaw) < math.radians(1.5):
                        self.publish_vel(0, 0)
                        self.linear_pid.reset()
                        init_err = max(0.0, self.filt_dock_dist - self.target_dist)
                        self.linear_pid.prev_err = init_err
                        return DockingState.STRAIGHT_REVERSE, valid_time
                    self.publish_vel(0, self.angular_pid.update(self.filt_rel_yaw, dt))

                elif state == DockingState.STRAIGHT_REVERSE:
                    dist_error = max(
                        0.0,
                        self.filt_dock_dist
                        - self.target_dist
                    )

                    # --------------------------------
                    # 근거리 진입: 여기부터 ICP를 끄고 odom 직선 후진
                    # --------------------------------
                    if self.filt_dock_dist <= self.p['final_blind_start_dist']:
                        if odom_x is None or odom_y is None or odom_yaw is None:
                            self.publish_vel(0.0, 0.0)
                            return state, valid_time

                        self.publish_vel(0.0, 0.0)

                        self.final_reverse_start_x = odom_x
                        self.final_reverse_start_y = odom_y
                        self.final_reverse_start_yaw = odom_yaw
                        self.final_reverse_distance = max(
                            0.0,
                            self.filt_dock_dist - self.target_dist
                        )

                        self.get_logger().info(
                            f'ICP 종료 -> 최종 직선 후진 시작: ' 
                            f'dock_dist={self.filt_dock_dist:.3f}m, ' 
                            f'target={self.target_dist:.3f}m, ' 
                            f'남은후진={self.final_reverse_distance:.3f}m, ' 
                            f'speed={self.p["final_reverse_speed"]:.3f}m/s'
                        )

                        if self.final_reverse_distance <= self.p['final_reverse_tolerance']:
                            self.get_logger().info('추가 후진 거리 없음 -> 도킹 완료')
                            return DockingState.COMPLETED, curr_time

                        return DockingState.FINAL_REVERSE, curr_time

                    # --------------------------------
                    # 목표 도킹 거리 도달
                    # --------------------------------
                    if dist_error <= 0.015:
                        self.publish_vel(
                            0.0,
                            0.0
                        )

                        self.get_logger().info(
                            f'도크 접점 도달 완료 '
                            f'(남은오차: {dist_error:.3f}m). '
                            f'도킹 완료.'
                        )

                        return (
                            DockingState.COMPLETED,
                            curr_time
                        )

                    # --------------------------------
                    # 후진 속도
                    # --------------------------------
                    v_mag = self.linear_pid.update(
                        dist_error,
                        dt
                    )

                    v = -v_mag

                    # --------------------------------
                    # lateral 오차 -> 목표 yaw -> yaw PD 제어
                    # --------------------------------
                    # 차동구동(논홀로노믹) 로봇은 옆으로 직접 움직일 수 없으므로
                    # 도크 중심선 lateral 오차를 작은 목표 yaw로 변환한다.
                    # lateral이 줄어들수록 target_yaw도 0으로 수렴하고,
                    # yaw PD가 차체를 다시 도크 축과 평행하게 만들어
                    # 후진 궤적이 자연스럽게 S자 형태가 되도록 한다.
                    lateral_gain = self.p['reverse_lateral_gain']

                    target_yaw = (
                        lateral_gain
                        * self.filt_lateral_error
                    )

                    max_target_yaw = math.radians(
                        self.p['reverse_max_target_yaw_deg']
                    )

                    target_yaw = float(
                        np.clip(
                            target_yaw,
                            -max_target_yaw,
                            max_target_yaw
                        )
                    )

                    yaw_control_error = normalize_angle(
                        self.filt_rel_yaw
                        + target_yaw
                    )

                    # --------------------------------
                    # 도크가 가까워질수록
                    # steering을 서서히 감소
                    # --------------------------------
                    steering_start_dist = self.p[
                        'steering_lock_dist'
                    ]

                    steering_stop_dist = (
                        self.target_dist + 0.015
                    )

                    if (
                        self.filt_dock_dist
                        >= steering_start_dist
                    ):
                        steering_scale = 1.0

                    elif (
                        self.filt_dock_dist
                        <= steering_stop_dist
                    ):
                        steering_scale = 0.0

                    else:
                        steering_scale = (
                            self.filt_dock_dist
                            - steering_stop_dist
                        ) / (
                            steering_start_dist
                            - steering_stop_dist
                        )

                    # --------------------------------
                    # 최종 각속도
                    # --------------------------------
                    w = (
                        steering_scale
                        * self.angular_pid.update(
                            yaw_control_error,
                            dt
                        )
                    )

                    self.publish_vel(
                        v,
                        w
                    )
                else:
                    self.get_logger().info("도킹 상태 오류: 알 수 없는 상태")


        return state, valid_time

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
        node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()