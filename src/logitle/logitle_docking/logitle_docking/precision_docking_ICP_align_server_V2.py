import math
import numpy as np
from enum import Enum
from scipy.spatial import cKDTree

import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, PoseStamped
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
    ALIGN_YAW = 7          # [Odom/Pose] 도킹 체결 후 도크 방향으로 최종 미세 정렬
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
        distances, indices = target_tree.query(transformed, workers=2)

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

    # 도크 모델 점 개수(len(target_pts)) 기준으로 정규화하여 배경 노이즈로 인한 필터 지연 방지
    fitness = float(np.sum(distances < 0.08) / max(len(target_pts), 1)) if len(distances) > 0 else 0.0
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
        self.set_parameters([rp.parameter.Parameter('use_sim_time', rp.Parameter.Type.BOOL, True)])
        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(
            self, PrecisionDock, 'precision_dock',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = self.pose_sub = None

        self._declare_and_update_params()
        self._reset_state_variables()

        # [PID 제어기 설정]
        # out_max를 0.15로 설정하여 남은 거리(0.55m -> 0.0m) 동안 P 제어로 자연스럽게 감속
        # 라이다 센서 주기(5~10Hz)와 루프(20Hz) 간 미분 헌팅을 방지하기 위해 d=0.02 적용
        self.linear_pid = PID(p=0.5, i=0.0, d=0.02, out_min=0.01, out_max=0.05)
        self.angular_pid = PID(1.2, 0.1, 0.05, -0.25, 0.25)
        self.staging_linear_pid = PID(0.6, 0.0, 0.02, 0.0, 0.15)
        self.staging_angular_pid = PID(1.8, 0.05, 0.1, -0.5, 0.5)

        self.get_logger().info('Two-Phase 정밀 도킹 서버 준비 완료 (Idle 상태)')

    def _declare_and_update_params(self):
        params = {
            'charger_width': 0.25, 'wing_length': 0.35, 'wing_angle_deg': 45.0,
            'robot_rear_length': 0.20, 'roi_x_min': -2.0, 'roi_x_max': -0.07, 'roi_y_limit': 2.5,
            'staging_distance': 0.8, 'final_yaw_tolerance_deg': 0.1,
            'steering_lock_dist': 0.35, 'odom_frame': 'odom', 'base_frame': 'base_footprint'
        }
        for name, val in params.items():
            self.declare_parameter(name, val)
        self.p = {name: self.get_parameter(name).value for name in params.keys()}
        self.wing_rad = math.radians(self.p['wing_angle_deg'])
        self.final_yaw_tol_rad = math.radians(self.p['final_yaw_tolerance_deg'])

    def _reset_state_variables(self):
        self.latest_source_pts = self.target_tree = None
        self.new_scan_available = False
        self.current_transform = np.identity(3, dtype=np.float32)
        self.filt_rel_yaw = self.filt_dock_dist = self.prev_yaw = self.prev_dist = None
        self.last_fitness, self.icp_fail_count, self.settle_timer = 0.0, 0, 0.0
        self.target_dist = self.p['robot_rear_length']
        self.target_dock_yaw = 0.0

    def generate_v_funnel(self, gap=0.0):
        pts = []
        half_w = self.p['charger_width'] / 2.0
        
        # 1. 안쪽 충전 단자 바닥면 (x = 0)
        for y in np.arange(-half_w, half_w + 0.005, 0.02):
            pts.append([0.0, y])
            
        # 2. V자 양 날개 (+X 방향 전개)
        for l in np.arange(0.0, self.p['wing_length'] + 0.005, 0.02):
            px = l * math.cos(self.wing_rad)
            py = half_w + l * math.sin(self.wing_rad)
            pts.extend([[px, py], [px, -py]])
            
        return np.array(pts, dtype=np.float32)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment

        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        xs, ys = ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])
        roi = (xs < self.p['roi_x_max']) & (xs > self.p['roi_x_min']) & (np.abs(ys) < self.p['roi_y_limit'])
        if np.sum(roi) >= 10:
            self.latest_source_pts, self.new_scan_available = np.column_stack((xs[roi], ys[roi])).astype(np.float32), True
        else:
            self.latest_source_pts = None

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
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data, callback_group=self.cb_group)
        self._reset_state_variables()
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
                    if not self.new_scan_available or odom_x is None:
                        rate.sleep()
                        continue

                    req_x = goal_handle.request.target_pose.pose.position.x
                    self.target_dist = req_x if req_x > 0 else self.p['robot_rear_length']
                    active_target_pts = self.generate_v_funnel(self.target_dist)
                    self.target_tree = cKDTree(active_target_pts)
                    self.new_scan_available = False

                    init_t = np.mean(self.latest_source_pts, axis=0)
                    init_T = np.identity(3, dtype=np.float32)
                    init_T[:2, 2] = -init_t

                    T, _, match_cnt = icp_2d(
                        self.latest_source_pts, 
                        self.target_tree, 
                        active_target_pts, 
                        initial_transform=init_T, 
                        max_iter=40, 
                        search_radius=0.8
                    )
                    
                    if match_cnt >= 10:
                        R, t = T[:2, :2], T[:2, 2]
                        R_base_dock = R.T
                        t_base_dock = -R.T @ t

                        rel_x = float(t_base_dock[0])
                        rel_y = float(t_base_dock[1])
                        dock_yaw_in_base = math.atan2(R_base_dock[1, 0], R_base_dock[0, 0])

                        # Odom 상의 도크 위치 및 도크 정면 방향 계산
                        dock_odom_x = odom_x + rel_x * math.cos(odom_yaw) - rel_y * math.sin(odom_yaw)
                        dock_odom_y = odom_y + rel_x * math.sin(odom_yaw) + rel_y * math.cos(odom_yaw)
                        dock_odom_yaw = normalize_angle(odom_yaw + dock_yaw_in_base)

                        # [중요] 도크가 바라보는 방향을 최종 정렬 목표 각도로 저장
                        self.target_dock_yaw = dock_odom_yaw

                        # 경유지 Odom 좌표 계산
                        stg_dist = self.p['staging_distance']
                        target_odom['x'] = dock_odom_x + stg_dist * math.cos(dock_odom_yaw)
                        target_odom['y'] = dock_odom_y + stg_dist * math.sin(dock_odom_yaw)
                        target_odom['stg_yaw'] = math.atan2(target_odom['y'] - odom_y, target_odom['x'] - odom_x)
                        target_odom['dock_yaw'] = dock_odom_yaw

                        self.get_logger().info(f'도크 Odom 추정: x={dock_odom_x:.2f}, y={dock_odom_y:.2f}, 도크방향={math.degrees(dock_odom_yaw):.1f}°')
                        self.get_logger().info(f'경유지 설정: x={target_odom["x"]:.2f}, y={target_odom["y"]:.2f}')
                        state = DockingState.STAGING_TURN

                # Phase 2: Odom 기반 경유지 전진 주행 및 후진 자세 회전
                elif 1 <= state.value <= 3:
                    if odom_x is None:
                        rate.sleep()
                        continue
                    state = self._handle_macro_drive(state, dt, odom_x, odom_y, odom_yaw, target_odom)

                # Phase 3 & 4: 센서 기반 후진 도킹 및 최종 정면 정렬
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
            self.get_logger().info('도킹 액션 종료: 리소스 반환 완료')

    def _handle_macro_drive(self, state, dt, x, y, yaw, tgt):
        # 1. 경유지 방향으로 제자리 회전
        if state == DockingState.STAGING_TURN:
            err = normalize_angle(tgt['stg_yaw'] - yaw)
            if abs(err) < math.radians(1.5):
                self.publish_vel(0.0, 0.0)
                return DockingState.STAGING_DRIVE
            w = self.staging_angular_pid.update(err, dt)
            self.publish_vel(0.0, w)

        # 2. 경유지를 향해 직진 주행
        elif state == DockingState.STAGING_DRIVE:
            dist = math.hypot(tgt['x'] - x, tgt['y'] - y)
            if dist < 0.05:
                self.publish_vel(0.0, 0.0)
                return DockingState.STAGING_ALIGN
            v = self.staging_linear_pid.update(dist, dt)
            self.publish_vel(v, 0.0)

        # 3. 경유지 도착 후 도킹 진입축(도크를 등지는 방향)으로 제자리 회전
        elif state == DockingState.STAGING_ALIGN:
            err = normalize_angle(tgt['dock_yaw'] - yaw)
            if abs(err) < math.radians(1.5):
                self.publish_vel(0.0, 0.0)
                self.new_scan_available = False
                self.current_transform = np.identity(3, dtype=np.float32)
                self.filt_dock_dist = self.filt_rel_yaw = self.prev_dist = self.prev_yaw = None
                self.settle_timer = 0.5
                return DockingState.ICP_SETTLE
            w = self.staging_angular_pid.update(err, dt)
            self.publish_vel(0.0, w)

        return state

    def _handle_micro_docking(self, state, dt, curr_time, valid_time, active_pts, odom_yaw):
        if state == DockingState.ICP_SETTLE:
            self.publish_vel(0, 0)
            self.settle_timer -= dt
            if self.settle_timer <= 0:
                self.get_logger().info('경유지 도착: 후진 도킹 시작')
                return DockingState.ALIGN_HEADING, valid_time

        # [수정] 마지막 정렬 단계: 임의의 0도가 아니라 실제 도크 방향(self.target_dock_yaw)과 정렬
        elif state == DockingState.ALIGN_YAW:
            if odom_yaw is None:
                self.publish_vel(0, 0)
                return state, valid_time

            # 이미 도크를 등지고 후진해 들어왔으므로 오차는 1~2도 미만입니다. 180도 회전하지 않습니다.
            yaw_err = normalize_angle(self.target_dock_yaw - odom_yaw)
            if abs(yaw_err) < self.final_yaw_tol_rad:
                self.publish_vel(0, 0)
                self.get_logger().info(f'도킹 완료: 최종 정렬 완료 (오차: {math.degrees(yaw_err):.2f}°)')
                return DockingState.COMPLETED, valid_time

            w_cmd = self.staging_angular_pid.update(yaw_err, dt)
            self.publish_vel(0, w_cmd)
            return state, valid_time

        # 근거리 후진 도킹 루프
        else:
            if self.new_scan_available and self.latest_source_pts is not None:
                self.new_scan_available = False
                s_rad, max_iter = (1.5, 25) if self.filt_dock_dist is None else (0.5, 12)
                T, fitness, match_cnt = icp_2d(self.latest_source_pts, self.target_tree, active_pts, self.current_transform, max_iter, s_rad)

                if match_cnt >= 10:
                    valid_time, self.icp_fail_count, self.current_transform = curr_time, 0, T
                    R, t = T[:2, :2], T[:2, 2]
                    raw_dist = float(t[0])
                    raw_yaw = normalize_angle(-math.atan2(R[1, 0], R[0, 0]))

                    if self.prev_dist is None or (abs(raw_dist - self.prev_dist) <= 0.1 and abs(normalize_angle(raw_yaw - self.prev_yaw)) <= math.radians(10.0)):
                        # 거리 추종 지연(Lag)을 방지하여 실시간 거리 반영
                        alpha = 0.6 if fitness > 0.6 else (0.4 if fitness > 0.3 else 0.2)
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
                        self.linear_pid.reset()
                        # D 제어 튐 방지를 위해 초기 오차값으로 prev_err 설정
                        init_err = max(0.0, self.filt_dock_dist - self.target_dist)
                        self.linear_pid.prev_err = init_err
                        return DockingState.STRAIGHT_REVERSE, valid_time
                    self.publish_vel(0, self.angular_pid.update(self.filt_rel_yaw, dt))

                elif state == DockingState.STRAIGHT_REVERSE:
                    # [핵심] 실제 도크 접점까지 남은 오차 거리 (로봇 중심 거리 - 목표 오프셋)
                    dist_error = max(0.0, self.filt_dock_dist - self.target_dist)

                    # 남은 거리가 1.5cm 이내로 들어오면 도크 체결 완료로 판단하고 정지
                    if dist_error <= 0.015:
                        self.publish_vel(0.0, 0.0)
                        self.get_logger().info(f'도크 접점 도달 완료 (남은오차: {dist_error:.3f}m). 최종 정렬로 이동.')
                        return DockingState.ALIGN_YAW, valid_time

                    # [PID 제어기 갱신]
                    # 남은 오차(dist_error)가 0.55m -> 0.0m로 줄어듦에 따라 linear_pid 출력(v_mag)이 0.15 m/s에서 0.01 m/s로 선형 감속
                    v_mag = self.linear_pid.update(dist_error, dt)
                    
                    # 후진 방향이므로 마이너스 부호 부여
                    v = -v_mag
                    w = 0.0 if self.filt_dock_dist < self.p['steering_lock_dist'] else self.angular_pid.update(self.filt_rel_yaw, dt)
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