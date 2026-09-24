import math
import numpy as np
from enum import Enum
from scipy.spatial import cKDTree
from collections import deque

import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

import tf2_ros

from turtlebot3_my_msg.action import PrecisionDock


# ---------------------------------------------------------
# [상태 머신 정의] 3단계 제어 시퀀스로 변경 (하드웨어 한계 극복)
# ---------------------------------------------------------
class DockingState(Enum):
    INIT = 0               # 점군 수신 대기 및 도크 파라미터 초기화
    ALIGN_IN_PLACE = 1     # 1차: 사각지대 진입 전 제자리 회전으로 완벽한 평행(Yaw) 맞추기
    BLIND_INSERT = 2       # 2차: 눈 감고 오도메트리 기반 일직선 후진
    COMPLETED = 3          # 도킹 완료
    FAILED = 4             # 도킹 실패/중단


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

        # TF2 버퍼 및 리스너 초기화
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
        # 파라미터 선언 (터틀봇3 버거 및 현재 하드웨어 맞춤형)
        # ---------------------------------------------------------
        self.declare_parameter('charger_width', 0.20)
        self.declare_parameter('wing_length', 0.35)
        self.declare_parameter('wing_angle_deg', 35.0)  # 수정된 가벽 각도 반영
        self.declare_parameter('robot_rear_length', 0.20) # 부착물 포함 로봇 후면 길이 (필요시 조절)

        # ROI 및 안전 거리
        self.declare_parameter('roi_x_min', -0.5)
        self.declare_parameter('roi_x_max', -0.9) # 로봇 자신의 기둥/바퀴가 찍히지 않도록 뒤로 밀음
        self.declare_parameter('roi_y_limit', 0.30)
        self.declare_parameter('safety_stop_dist', 0.01)
        self.declare_parameter('blind_spot_dist', 0.25)
        self.declare_parameter('heading_align_deg', 15.0)

        # 최종 Yaw 정렬 파라미터
        self.declare_parameter('final_target_yaw_deg', 0.0)
        self.declare_parameter('final_yaw_tolerance_deg', 0.8)

        self.update_parameters()
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.latest_source_pts = None

        self.linear_pid = PID(p=0.15, i=0.01, d=0.05, out_min=0.0, out_max=0.03)
        self.angular_pid = PID(p=0.5, i=0.01, d=0.20, out_min=-0.25, out_max=0.25)
        self.final_yaw_pid = PID(p=0.8, i=0.0, d=0.10, out_min=-0.15, out_max=0.15)

        self.get_logger().info('3단계 정밀 도킹 서버(제자리 정렬 -> 일직선 후진) 준비 완료.')

    def update_parameters(self):
        self.charger_width = self.get_parameter('charger_width').value
        self.wing_length = self.get_parameter('wing_length').value
        self.wing_angle_deg = self.get_parameter('wing_angle_deg').value
        self.robot_rear_length = self.get_parameter('robot_rear_length').value
        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_limit = self.get_parameter('roi_y_limit').value
        self.safety_stop_dist = self.get_parameter('safety_stop_dist').value
        self.blind_spot_dist = self.get_parameter('blind_spot_dist').value
        self.heading_align_deg = self.get_parameter('heading_align_deg').value
        self.final_target_yaw_rad = math.radians(self.get_parameter('final_target_yaw_deg').value)
        self.final_yaw_tolerance_rad = math.radians(self.get_parameter('final_yaw_tolerance_deg').value)

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
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        rear_fov_mask = np.abs(angles) >= math.radians(110.0)
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

    def get_current_odom_yaw(self) -> float:
        """TF 버퍼에서 odom -> base_link Yaw 각도를 순수 math 함수로 추출합니다."""
        t = self.tf_buffer.lookup_transform('odom', 'base_link', rp.time.Time())
        return get_yaw_from_quaternion(t.transform.rotation)

    def get_current_odom_pose(self):
        """TF 버퍼에서 odom 기준 현재 위치(X, Y)와 각도(Yaw)를 모두 가져옵니다."""
        t = self.tf_buffer.lookup_transform('odom', 'base_link', rp.time.Time())
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = get_yaw_from_quaternion(t.transform.rotation)
        return x, y, yaw

    def goal_callback(self, goal_request):
        self.get_logger().info('도킹 액션 요청 수락')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('도킹 액션 취소 요청')
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
        self.get_logger().info('3단계(정지 정렬 -> 블라인드 후진) 도킹 시퀀스 가동')
        state = DockingState.INIT
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.final_yaw_pid.reset()

        active_target_pts = None
        current_transform = np.identity(3)
        is_first_frame = True
        icp_fail_count = 0
        success_message = ""

        # 블라인드 후진을 위한 래칭 변수
        latched_target_odom_yaw = 0.0
        latched_start_odom_pos = (0.0, 0.0)
        target_insert_dist = 0.0
        
        # 노이즈 필터링 변수
        rel_yaw_history = deque(maxlen=5)
        last_valid_rel_yaw = 0.0

        feedback_msg = PrecisionDock.Feedback()
        rate = self.create_rate(20.0)
        last_time = self.get_clock().now()

        while rp.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_stop()
                return PrecisionDock.Result(success=False, message="Canceled")

            current_time = self.get_clock().now()
            dt = (current_time - last_time).nanoseconds / 1e9
            last_time = current_time

            # -------------------------------------------------------------
            # [STATE 0: INIT]
            # -------------------------------------------------------------
            if state == DockingState.INIT:
                if self.latest_source_pts is None:
                    self.get_logger().info('라이다 ROI 구역 내 점군 데이터 대기 중...', throttle_duration_sec=2.0)
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

                self.get_logger().info('>> [상태 전이] INIT -> ALIGN_IN_PLACE (제자리 평행 정렬)')
                state = DockingState.ALIGN_IN_PLACE

            # -------------------------------------------------------------
            # [STATE 1: ALIGN_IN_PLACE] 제자리 회전으로 평행 맞추기
            # -------------------------------------------------------------
            elif state == DockingState.ALIGN_IN_PLACE:
                if self.latest_source_pts is None:
                    self.publish_stop()
                    rate.sleep()
                    continue

                search_r = 1.5 if is_first_frame else 0.35
                current_transform, fitness, match_cnt = icp_2d(
                    self.latest_source_pts, active_target_pts,
                    initial_transform=current_transform, max_iterations=35, search_radius=search_r
                )

                if match_cnt < 10:
                    icp_fail_count += 1
                    if icp_fail_count > 5:
                        self.get_logger().warn('ICP 매칭 실패, 초기 위치 복원!')
                        current_transform = np.identity(3)
                        is_first_frame = True
                        icp_fail_count = 0
                    self.publish_stop()
                    rate.sleep()
                    continue

                icp_fail_count = 0
                is_first_frame = False

                T = current_transform
                R_mat = T[:2, :2]
                t_vec = T[:2, 2]

                rel_pos = -R_mat.T @ t_vec
                rel_x, rel_y = rel_pos[0], rel_pos[1]
                raw_rel_yaw = normalize_angle(-math.atan2(R_mat[1, 0], R_mat[0, 0]))

                # 노이즈 필터링
                if fitness > 0.35 and match_cnt >= 15:
                    rel_yaw_history.append(raw_rel_yaw)

                if len(rel_yaw_history) > 0:
                    sum_sin = sum(math.sin(y) for y in rel_yaw_history)
                    sum_cos = sum(math.cos(y) for y in rel_yaw_history)
                    last_valid_rel_yaw = math.atan2(sum_sin, sum_cos)
                else:
                    last_valid_rel_yaw = raw_rel_yaw

                feedback_msg.distance_remaining = float(math.hypot(rel_x, rel_y))
                feedback_msg.angle_remaining = float(last_valid_rel_yaw)
                goal_handle.publish_feedback(feedback_msg)

                # 각도 오차가 1.5도 이내로 맞춰지면 후진 거리 래칭 후 전이
                if abs(last_valid_rel_yaw) <= math.radians(1.5):
                    self.publish_stop()
                    
                    # 후진해야 할 물리적 거리 계산
                    lidar_to_wall_dist = abs(np.min(self.latest_source_pts[:, 0])) if len(self.latest_source_pts) > 0 else 999.0
                    rear_to_wall_dist = lidar_to_wall_dist - self.robot_rear_length
                    target_insert_dist = max(0.0, rear_to_wall_dist - self.safety_stop_dist)
                    
                    try:
                        curr_x, curr_y, curr_yaw = self.get_current_odom_pose()
                        latched_target_odom_yaw = curr_yaw
                        latched_start_odom_pos = (curr_x, curr_y)
                        
                        self.get_logger().info(f'>> [정렬 완료] Y축 중심오차: {rel_y*100:.1f}cm')
                        self.get_logger().info(f'>> {target_insert_dist*100:.1f}cm 블라인드 후진 시작 (BLIND_INSERT)')
                        state = DockingState.BLIND_INSERT
                    except Exception as e:
                        self.get_logger().warn(f'TF 룩업 실패: {e}')
                    continue

                # PID 제어: 직진(v_cmd)은 막고 제자리 회전(w_cmd)만 수행
                w_out = self.angular_pid.update(last_valid_rel_yaw, dt)
                w_cmd = float(np.clip(w_out, -0.25, 0.25))
                
                # 터틀봇 모터 마찰 극복 최소 속도 보장
                if abs(w_cmd) < 0.05:
                    w_cmd = 0.05 if w_cmd > 0 else -0.05
                    
                self.publish_cmd_vel(0.0, w_cmd) 

            # -------------------------------------------------------------
            # [STATE 2: BLIND_INSERT] 오도메트리 기반 일직선 후진
            # -------------------------------------------------------------
            elif state == DockingState.BLIND_INSERT:
                try:
                    curr_x, curr_y, curr_yaw = self.get_current_odom_pose()
                except Exception as e:
                    self.publish_stop()
                    rate.sleep()
                    continue

                # 시작 위치에서부터 이동한 직선 거리 계산
                moved_dist = math.hypot(curr_x - latched_start_odom_pos[0], curr_y - latched_start_odom_pos[1])
                remain_dist = target_insert_dist - moved_dist

                feedback_msg.distance_remaining = float(remain_dist)
                feedback_msg.angle_remaining = float(normalize_angle(latched_target_odom_yaw - curr_yaw))
                goal_handle.publish_feedback(feedback_msg)

                # 목표 거리 도달 시 멈춤
                if remain_dist <= 0.005:
                    self.publish_stop()
                    self.get_logger().info('블라인드 후진 도킹 완료!')
                    state = DockingState.COMPLETED
                    success_message = "Successfully docked via Blind Insertion"
                    continue

                # 직진성 유지를 위한 미세 Yaw 보정 (래칭된 각도 유지)
                yaw_err = normalize_angle(latched_target_odom_yaw - curr_yaw)
                w_cmd = self.final_yaw_pid.update(yaw_err, dt)
                w_cmd = float(np.clip(w_cmd, -0.15, 0.15))

                # 고정 속도로 안전하게 후진 (속도를 0.015m/s로 아주 천천히 제한)
                v_cmd = -0.015
                self.publish_cmd_vel(v_cmd, w_cmd)

            # -------------------------------------------------------------
            # [STATE 3: COMPLETED]
            # -------------------------------------------------------------
            elif state == DockingState.COMPLETED:
                self.publish_stop()
                goal_handle.succeed()
                return PrecisionDock.Result(success=True, message=success_message)

            rate.sleep()

        self.publish_stop()
        goal_handle.abort()
        return PrecisionDock.Result(success=False, message="Aborted")


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