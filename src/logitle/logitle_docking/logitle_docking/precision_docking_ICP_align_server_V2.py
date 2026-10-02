import math
import numpy as np
from enum import Enum
from scipy.spatial import cKDTree

import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import LaserScan

import tf2_ros

from turtlebot3_my_msg.action import PrecisionDock


# ---------------------------------------------------------
# [상태 머신 정의]
# ---------------------------------------------------------
class DockingState(Enum):
    INIT = 0               # 점군 수신 대기 및 도크 파라미터 초기화
    CENTER_ALIGN = 1       # 0단계: 도킹스테이션 중심 라인(Y=0) 및 헤딩 진입 정렬
    ICP_APPROACH = 2       # 1차 도킹: 중앙 라인 기반 ICP 정밀 후진 진입
    ALIGN_YAW = 3          # 2차 정렬: logitle_pose(map 기준) 기반 최종 Yaw 정렬
    COMPLETED = 4          # 도킹 및 정렬 성공 완료
    FAILED = 5             # 도킹 실패/중단


def normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화합니다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw_from_quaternion(q) -> float:
    """쿼터니언 (x, y, z, w)에서 Z축 회전각(Yaw, 라디안)을 계산합니다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def icp_2d(source_pts: np.ndarray, target_pts: np.ndarray, initial_transform=None, max_iterations=35, search_radius=0.5):
    if initial_transform is None:
        T = np.identity(3)
    else:
        T = np.copy(initial_transform)

    tree = cKDTree(target_pts)
    src_h = np.hstack((source_pts, np.ones((source_pts.shape[0], 1))))

    distances = np.array([])
    matched_count = 0

    for _ in range(max_iterations):
        transformed = (T @ src_h.T).T[:, :2]
        distances, indices = tree.query(transformed)

        valid = distances < search_radius
        matched_count = int(np.sum(valid))
        if matched_count < 10:
            break

        pts_src = source_pts[valid]
        pts_tgt = target_pts[indices[valid]]

        centroid_src = np.mean(pts_src, axis=0)
        centroid_tgt = np.mean(pts_tgt, axis=0)

        src_centered = pts_src - centroid_src
        tgt_centered = pts_tgt - centroid_tgt

        H = src_centered.T @ tgt_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_tgt - (R @ centroid_src)

        T = np.identity(3)
        T[:2, :2] = R
        T[:2, 2] = t

    fitness = float(np.sum(distances < 0.08) / max(len(source_pts), 1)) if len(distances) > 0 else 0.0
    return T, fitness, matched_count


class PID:
    def __init__(self, p: float, i: float, d: float, out_min: float, out_max: float):
        self.p = p
        self.i = i
        self.d = d
        self.out_min = out_min
        self.out_max = out_max
        self.previous_err = 0.0
        self.integral = 0.0

    def update(self, error: float, dt: float = 0.05) -> float:
        if dt <= 0.0:
            dt = 0.05
        self.integral += error * dt
        derivative = (error - self.previous_err) / dt
        output = (self.p * error) + (self.i * self.integral) + (self.d * derivative)

        if output > self.out_max:
            output = self.out_max
            self.integral -= error * dt
        elif output < self.out_min:
            output = self.out_min
            self.integral -= error * dt

        self.previous_err = error
        return float(output)

    def reset(self):
        self.previous_err = 0.0
        self.integral = 0.0


class PrecisionDockingServer(Node):
    def __init__(self):
        super().__init__('precision_docking_server')
        self.cb_group = ReentrantCallbackGroup()

        self.is_docking_active = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(
            self,
            PrecisionDock,
            'precision_dock',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data, callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # ---------------------------------------------------------
        # 파라미터 선언
        # ---------------------------------------------------------
        self.declare_parameter('charger_width', 0.25)
        self.declare_parameter('wing_length', 0.35)
        self.declare_parameter('wing_angle_deg', 45.0)
        self.declare_parameter('robot_rear_length', 0.25)

        # 중앙 라인 사전 정렬(Pre-dock Center Align) 파라미터
        self.declare_parameter('predock_distance', 0.50)           # 중앙 정렬을 수행할 전방 대기 거리 (m)
        self.declare_parameter('center_y_tolerance', 0.02)         # 중심선 허용 횡방향 오차 (m)
        self.declare_parameter('center_yaw_tolerance_deg', 3.0)    # 중심선 허용 각도 오차 (deg)

        # ROI 및 안전 거리
        self.declare_parameter('roi_x_min', -1.2)                  # 사전 대기점 관측을 위해 탐색 범위 확장
        self.declare_parameter('roi_x_max', -0.15)
        self.declare_parameter('roi_y_limit', 0.35)
        self.declare_parameter('safety_stop_dist', 0.01)
        self.declare_parameter('blind_spot_dist', 0.20)
        self.declare_parameter('heading_align_deg', 15.0)

        # 최종 Yaw 정렬 파라미터
        self.declare_parameter('final_target_yaw_deg', 0.0)
        self.declare_parameter('final_yaw_tolerance_deg', 0.1)

        # 포즈 및 TF 파라미터
        self.declare_parameter('pose_topic', 'logitle_pose')
        self.declare_parameter('use_logitle_pose_topic', True)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.update_parameters()
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.latest_logitle_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.logitle_pose_callback,
            10,
            callback_group=self.cb_group
        )

        self.latest_source_pts = None

        # PID 컨트롤러 설정
        self.linear_pid = PID(p=0.2, i=0.2, d=0.05, out_min=-0.6, out_max=0.03)
        self.angular_pid = PID(p=1.0, i=0.5, d=0.20, out_min=-0.12, out_max=0.35)
        self.center_y_pid = PID(p=1.2, i=0.0, d=0.15, out_min=-0.35, out_max=0.35)
        self.final_yaw_pid = PID(p=0.8, i=0.0, d=0.10, out_min=-0.12, out_max=0.25)

        self.dist_tolerance = 0.001
        self.get_logger().info('중앙 라인 정렬 지원 정밀 도킹 서버 초기화 완료.')

    def logitle_pose_callback(self, msg: PoseStamped):
        if not self.is_docking_active:
            return
        self.latest_logitle_pose = msg

    def update_parameters(self):
        self.charger_width = self.get_parameter('charger_width').value
        self.wing_length = self.get_parameter('wing_length').value
        self.wing_angle_deg = self.get_parameter('wing_angle_deg').value
        self.robot_rear_length = self.get_parameter('robot_rear_length').value

        self.predock_distance = self.get_parameter('predock_distance').value
        self.center_y_tolerance = self.get_parameter('center_y_tolerance').value
        self.center_yaw_tolerance_rad = math.radians(self.get_parameter('center_yaw_tolerance_deg').value)

        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_limit = self.get_parameter('roi_y_limit').value
        self.safety_stop_dist = self.get_parameter('safety_stop_dist').value
        self.blind_spot_dist = self.get_parameter('blind_spot_dist').value
        self.heading_align_deg = self.get_parameter('heading_align_deg').value
        self.final_target_yaw_rad = math.radians(self.get_parameter('final_target_yaw_deg').value)
        self.final_yaw_tolerance_rad = math.radians(self.get_parameter('final_yaw_tolerance_deg').value)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.use_logitle_pose_topic = self.get_parameter('use_logitle_pose_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'파라미터 변경: {param.name} -> {param.value}')
        self.update_parameters()
        return SetParametersResult(successful=True)

    def generate_v_funnel_target(self, charger_w: float, wing_len: float, wing_deg: float, gap: float) -> np.ndarray:
        points = []
        back_x = -gap
        half_w = charger_w / 2.0
        for y in np.arange(-half_w, half_w + 0.005, 0.01):
            points.append([back_x, y])
        rad = math.radians(wing_deg)
        for l in np.arange(0.0, wing_len + 0.005, 0.01):
            px = back_x + l * math.cos(rad)
            points.append([px, half_w + l * math.sin(rad)])
            points.append([px, -(half_w + l * math.sin(rad))])
        return np.array(points, dtype=np.float64)

    def auto_estimate_v_funnel_params(self, pts: np.ndarray):
        if len(pts) < 30: return None
        min_x = np.min(pts[:, 0])
        station_mask = pts[:, 0] < (min_x + 0.45)
        valid_pts = pts[station_mask]
        if len(valid_pts) < 20: return None
        deepest_pts = valid_pts[valid_pts[:, 0] < (min_x + 0.08)]
        est_charger_w = float(np.ptp(deepest_pts[:, 1])) if len(deepest_pts) > 0 else 0.05
        est_charger_w = max(0.03, min(est_charger_w, 0.25))
        half_w = est_charger_w / 2.0
        left_wing_pts = valid_pts[valid_pts[:, 1] > half_w]
        right_wing_pts = valid_pts[valid_pts[:, 1] < -half_w]
        if len(left_wing_pts) < 5 or len(right_wing_pts) < 5: return None
        dx_span = float(np.max(valid_pts[:, 0]) - min_x)
        est_wing_len = max(0.20, min(dx_span / math.cos(math.radians(45.0)), 0.40))
        dx = np.max(left_wing_pts[:, 0]) - np.min(left_wing_pts[:, 0])
        dy = np.max(left_wing_pts[:, 1]) - np.min(left_wing_pts[:, 1])
        est_angle_deg = math.degrees(math.atan2(dy, max(dx, 0.05)))
        est_angle_deg = max(35.0, min(est_angle_deg, 55.0))
        return est_charger_w, est_wing_len, est_angle_deg

    def scan_callback(self, msg: LaserScan):
        if not self.is_docking_active:
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        rear_fov_mask = np.abs(angles) >= math.radians(100.0)
        ranges = ranges[rear_fov_mask]
        angles = angles[rear_fov_mask]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        rear_roi = (xs < self.roi_x_max) & (xs > self.roi_x_min) & (np.abs(ys) < self.roi_y_limit)
        roi_xs = xs[rear_roi]
        roi_ys = ys[rear_roi]

        if len(roi_xs) >= 10:
            self.latest_source_pts = np.column_stack((roi_xs, roi_ys)).astype(np.float64)
        else:
            self.latest_source_pts = None

    def get_current_logitle_yaw(self) -> float:
        if self.use_logitle_pose_topic and self.latest_logitle_pose is not None:
            q = self.latest_logitle_pose.pose.orientation
            return get_yaw_from_quaternion(q)

        t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rp.time.Time())
        return get_yaw_from_quaternion(t.transform.rotation)

    def goal_callback(self, goal_request):
        if self.is_docking_active:
            self.get_logger().warn('이미 도킹 작업이 진행 중입니다. 새 요청을 거부합니다.')
            return GoalResponse.REJECT

        self.get_logger().info('도킹 액션 요청 수락')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('도킹 액션 취소 요청 수락')
        return CancelResponse.ACCEPT

    def publish_cmd_vel(self, v: float, w: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd_vel(0.0, 0.0)

    def execute_callback(self, goal_handle):
        self.is_docking_active = True
        self.latest_source_pts = None
        self.latest_logitle_pose = None
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.center_y_pid.reset()
        self.final_yaw_pid.reset()

        self.get_logger().info('도킹 시퀀스 가동 (중앙 라인 정렬 -> ICP 후진)')
        state = DockingState.INIT

        active_target_pts = None
        current_transform = np.identity(3)
        dist_err = 999.0
        is_first_frame = True
        icp_fail_count = 0
        success_message = ""

        feedback_msg = PrecisionDock.Feedback()
        rate = self.create_rate(20.0)
        last_time = self.get_clock().now()

        try:
            while rp.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.publish_stop()
                    return PrecisionDock.Result(success=False, message="Canceled")

                current_time = self.get_clock().now()
                dt = (current_time - last_time).nanoseconds / 1e9
                last_time = current_time

                # -------------------------------------------------------------
                # [STATE 0: INIT] 템플릿 생성 및 초기 위치 파악
                # -------------------------------------------------------------
                if state == DockingState.INIT:
                    if self.latest_source_pts is None:
                        self.publish_stop()
                        rate.sleep()
                        continue

                    req_pos = goal_handle.request.target_pose.pose.position
                    if req_pos.y <= 0.0 or req_pos.z <= 0.0:
                        estimated = self.auto_estimate_v_funnel_params(self.latest_source_pts)
                        if estimated is not None:
                            w, l, ang = estimated
                        else:
                            w, l, ang = self.charger_width, self.wing_length, self.wing_angle_deg
                    else:
                        w, l, ang = req_pos.y, req_pos.z, self.wing_angle_deg

                    safe_gap = self.robot_rear_length + 0.005
                    gap = req_pos.x if req_pos.x > 0.0 else safe_gap
                    active_target_pts = self.generate_v_funnel_target(w, l, ang, gap)

                    self.get_logger().info('>> [상태 전이] INIT -> CENTER_ALIGN (도크 중심축 정렬 시작)')
                    state = DockingState.CENTER_ALIGN

                # -------------------------------------------------------------
                # [STATE 1: CENTER_ALIGN] 중앙 라인(Y=0) 및 후진 진입 헤딩 정렬
                # -------------------------------------------------------------
                elif state == DockingState.CENTER_ALIGN:
                    if self.latest_source_pts is None:
                        self.publish_stop()
                        rate.sleep()
                        continue

                    # 1. 현재 도크의 상대 좌표 추정
                    search_r = 1.5 if is_first_frame else 0.40
                    current_transform, fitness, match_cnt = icp_2d(
                        self.latest_source_pts, active_target_pts,
                        initial_transform=current_transform, max_iterations=35, search_radius=search_r
                    )

                    if match_cnt < 10:
                        self.publish_stop()
                        rate.sleep()
                        continue

                    is_first_frame = False
                    T = current_transform
                    R_mat = T[:2, :2]
                    t_vec = T[:2, 2]

                    rel_pos = -R_mat.T @ t_vec
                    rel_x, rel_y = rel_pos[0], rel_pos[1]
                    rel_yaw = normalize_angle(-math.atan2(R_mat[1, 0], R_mat[0, 0]))

                    # 2. 중심축(Y=0) 조향 오차 산출
                    # lookahead 지점을 이용해 중앙선을 타도록 목표 방향 계산
                    lookahead = 0.4
                    target_heading_to_center = math.atan2(rel_y, lookahead)
                    heading_err = normalize_angle(rel_yaw - target_heading_to_center)

                    feedback_msg.distance_remaining = float(abs(rel_y))
                    feedback_msg.angle_remaining = float(heading_err)
                    goal_handle.publish_feedback(feedback_msg)

                    # 3. 중앙 정렬 완료 판정: 횡방향 오차 < tolerance & 헤딩 오차 < tolerance
                    if abs(rel_y) <= self.center_y_tolerance and abs(rel_yaw) <= self.center_yaw_tolerance_rad:
                        self.get_logger().info(
                            f'중심 라인 정렬 완료! (Y 오차: {rel_y*100:.1f}cm, 각도: {math.degrees(rel_yaw):.1f}°)'
                        )
                        self.publish_stop()
                        self.get_logger().info('>> [상태 전이] CENTER_ALIGN -> ICP_APPROACH (정밀 후진 시작)')
                        state = DockingState.ICP_APPROACH
                        continue

                    # 4. 중앙 정렬 제어 입력 생성 (제자리 회전 및 저속 접근 조향)
                    if abs(heading_err) > math.radians(20.0):
                        v_cmd = 0.0
                        w_cmd = float(np.clip(self.angular_pid.update(heading_err, dt), -0.25, 0.25))
                    else:
                        # 중심선 방향으로 부드럽게 감속 접근
                        v_cmd = -0.015 if abs(rel_y) > self.center_y_tolerance else 0.0
                        w_cmd = float(np.clip(self.center_y_pid.update(heading_err, dt), -0.20, 0.20))

                    if 0.0 < abs(w_cmd) < 0.03:
                        w_cmd = 0.03 if w_cmd > 0 else -0.03

                    self.publish_cmd_vel(v_cmd, w_cmd)

                # -------------------------------------------------------------
                # [STATE 2: ICP_APPROACH] 중앙 라인을 따라 1차 정밀 후진 진입
                # -------------------------------------------------------------
                elif state == DockingState.ICP_APPROACH:
                    if self.latest_source_pts is None:
                        self.publish_stop()
                        if dist_err < self.blind_spot_dist:
                            self.get_logger().info('사각지대 진입 감지 -> ALIGN_YAW 상태로 전이')
                            state = DockingState.ALIGN_YAW
                        rate.sleep()
                        continue

                    pts = self.latest_source_pts
                    if len(pts) > 0:
                        lidar_to_wall_dist = abs(np.min(pts[:, 0]))
                        rear_to_wall_dist = lidar_to_wall_dist - self.robot_rear_length
                        if rear_to_wall_dist <= self.safety_stop_dist:
                            self.get_logger().info('후미 완전 밀착 안전 거리 도달 -> ALIGN_YAW 상태로 전이')
                            self.publish_stop()
                            state = DockingState.ALIGN_YAW
                            continue

                    current_transform, fitness, match_cnt = icp_2d(
                        self.latest_source_pts, active_target_pts,
                        initial_transform=current_transform, max_iterations=35, search_radius=0.35
                    )

                    if match_cnt < 10:
                        icp_fail_count += 1
                        if icp_fail_count > 5:
                            self.get_logger().warn('ICP 매칭 연속 실패, 초기 위치 복원!')
                            current_transform = np.identity(3)
                            is_first_frame = True
                            icp_fail_count = 0
                        self.publish_stop()
                        rate.sleep()
                        continue

                    icp_fail_count = 0
                    T = current_transform
                    R_mat = T[:2, :2]
                    t_vec = T[:2, 2]

                    rel_pos = -R_mat.T @ t_vec
                    rel_x, rel_y = rel_pos[0], rel_pos[1]
                    rel_yaw = normalize_angle(-math.atan2(R_mat[1, 0], R_mat[0, 0]))
                    dist_err = math.hypot(rel_x, rel_y)

                    feedback_msg.distance_remaining = float(dist_err)
                    feedback_msg.angle_remaining = float(rel_yaw)
                    goal_handle.publish_feedback(feedback_msg)

                    if dist_err < self.dist_tolerance and abs(rel_y) < 0.015 and fitness > 0.4:
                        self.get_logger().info('ICP 허용 오차 도달 -> ALIGN_YAW 상태로 전이')
                        self.publish_stop()
                        state = DockingState.ALIGN_YAW
                        continue

                    lookahead_dist = 0.25
                    raw_correction = math.atan2(rel_y, lookahead_dist)
                    clamped_correction = float(np.clip(raw_correction, -math.radians(35.0), math.radians(35.0)))
                    total_heading_err = normalize_angle(rel_yaw - clamped_correction)

                    if abs(total_heading_err) > math.radians(self.heading_align_deg):
                        v_cmd = 0.0
                        w_raw = self.angular_pid.update(total_heading_err, dt)
                        w_cmd = float(np.clip(w_raw, -0.35, 0.35))
                    else:
                        max_v, max_w, min_v = 0.035, 0.30, 0.008
                        if dist_err < 0.35:
                            max_v, max_w = 0.012, 0.35

                        heading_damping = max(0.0, math.cos(total_heading_err))
                        speed_mag = self.linear_pid.update(dist_err, dt)

                        v_cmd = -float(np.clip(speed_mag * heading_damping, min_v, max_v))
                        w_raw = self.angular_pid.update(total_heading_err, dt)
                        w_cmd = float(np.clip(w_raw, -max_w, max_w))

                    self.publish_cmd_vel(v_cmd, w_cmd)

                # -------------------------------------------------------------
                # [STATE 3: ALIGN_YAW] 최종 Yaw 정렬
                # -------------------------------------------------------------
                elif state == DockingState.ALIGN_YAW:
                    try:
                        current_yaw = self.get_current_logitle_yaw()
                    except Exception as e:
                        self.get_logger().warn(f'logitle_pose / TF 룩업 대기 중: {e}')
                        self.publish_stop()
                        rate.sleep()
                        continue

                    req_ori = goal_handle.request.target_pose.pose.orientation
                    norm_q = req_ori.x**2 + req_ori.y**2 + req_ori.z**2 + req_ori.w**2
                    if norm_q > 0.5:
                        target_yaw = get_yaw_from_quaternion(req_ori)
                    else:
                        target_yaw = self.final_target_yaw_rad

                    yaw_error = normalize_angle(target_yaw - current_yaw)

                    feedback_msg.distance_remaining = 0.0
                    feedback_msg.angle_remaining = float(yaw_error)
                    goal_handle.publish_feedback(feedback_msg)

                    if abs(yaw_error) <= self.final_yaw_tolerance_rad:
                        self.get_logger().info(
                            f'최종 정렬 완료! 오차: {math.degrees(yaw_error):.2f}°'
                        )
                        self.publish_stop()
                        state = DockingState.COMPLETED
                        success_message = (
                            f"Successfully docked and aligned to {math.degrees(target_yaw):.1f} deg"
                        )
                        continue

                    w_out = self.final_yaw_pid.update(yaw_error, dt)
                    w_cmd = float(np.clip(w_out, -0.20, 0.20))

                    if abs(w_cmd) < 0.03:
                        w_cmd = 0.03 if w_cmd > 0 else -0.03

                    self.publish_cmd_vel(0.0, w_cmd)

                # -------------------------------------------------------------
                # [STATE 4: COMPLETED] 완료
                # -------------------------------------------------------------
                elif state == DockingState.COMPLETED:
                    self.publish_stop()
                    goal_handle.succeed()
                    return PrecisionDock.Result(success=True, message=success_message)

                rate.sleep()

            self.publish_stop()
            goal_handle.abort()
            return PrecisionDock.Result(success=False, message="Aborted")

        finally:
            self.publish_stop()
            self.is_docking_active = False


def main(args=None):
    rp.init(args=args)
    server_node = PrecisionDockingServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server_node.destroy_node()
        if rp.ok():
            rp.shutdown()


if __name__ == '__main__':
    main()