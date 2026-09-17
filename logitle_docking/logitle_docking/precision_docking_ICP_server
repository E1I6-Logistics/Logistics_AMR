import math
import numpy as np
from scipy.spatial import cKDTree

import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

from turtlebot3_my_msg.action import PrecisionDock

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def icp_2d(source_pts: np.ndarray, target_pts: np.ndarray, initial_transform=None, max_iterations=35, search_radius=0.5):
    # (이전과 동일)
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
    # (이전과 동일)
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
        # [수정 1] 하드코딩되었던 도킹/안전/ROI 파라미터 선언
        # ---------------------------------------------------------
        self.declare_parameter('charger_width', 0.20)
        self.declare_parameter('wing_length', 0.40)
        self.declare_parameter('wing_angle_deg', 45.0)
        self.declare_parameter('robot_rear_length', 0.065)
        
        # 보안(안전) 및 ROI 관련 파라미터 추가
        self.declare_parameter('roi_x_min', -1.8)
        self.declare_parameter('roi_x_max', -0.01)
        self.declare_parameter('roi_y_limit', 0.8)
        self.declare_parameter('safety_stop_dist', 0.005)     # 완전 밀착 안전 정지 거리
        self.declare_parameter('blind_spot_dist', 0.20)       # 사각지대 진입 시 성공 판정 거리
        self.declare_parameter('heading_align_deg', 15.0)     # 제자리 정렬을 시작할 헤딩 오차 한계

        # 파라미터 초기화 적용
        self.update_parameters()

        # 파라미터 동적 변경 콜백 등록 (런타임에 파라미터 변경 시 자동 반영)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.latest_source_pts = None

        self.linear_pid = PID(p=0.15, i=0.0, d=0.05, out_min=0.0, out_max=0.03)
        self.angular_pid = PID(p=0.5, i=0.0, d=0.20, out_min=-0.35, out_max=0.35)

        self.dist_tolerance = 0.06
        self.angle_tolerance = 0.01

        self.get_logger().info('중앙 정렬 강화 2D ICP 정밀 도킹 서버(파라미터 동적 적용) 준비 완료.')

    def update_parameters(self):
        """선언된 파라미터 값을 멤버 변수로 갱신합니다."""
        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_limit = self.get_parameter('roi_y_limit').value
        self.safety_stop_dist = self.get_parameter('safety_stop_dist').value
        self.blind_spot_dist = self.get_parameter('blind_spot_dist').value
        self.heading_align_deg = self.get_parameter('heading_align_deg').value
        self.robot_rear_length = self.get_parameter('robot_rear_length').value

    def parameter_callback(self, params):
        """런타임에 파라미터가 수정되면 호출됩니다."""
        for param in params:
            self.get_logger().info(f'파라미터 변경 감지: {param.name} -> {param.value}')
        self.update_parameters()
        return SetParametersResult(successful=True)

    # (generate_v_funnel_target, auto_estimate_v_funnel_params 등은 이전과 동일하므로 생략 없이 사용)
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

        # [수정 2] 파라미터화된 ROI 적용
        rear_roi = (xs < self.roi_x_max) & (xs > self.roi_x_min) & (np.abs(ys) < self.roi_y_limit)
        roi_xs = xs[rear_roi]
        roi_ys = ys[rear_roi]

        if len(roi_xs) >= 10:
            self.latest_source_pts = np.column_stack((roi_xs, roi_ys)).astype(np.float64)
        else:
            self.latest_source_pts = None

    def goal_callback(self, goal_request):
        self.get_logger().info('★ 도킹 액션 요청 수락')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('도킹 액션 취소 요청 수락')
        return CancelResponse.ACCEPT

    def publish_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('🚀 V-Funnel 정밀 도킹 시작')
        self.angular_pid.reset()
        self.linear_pid.reset()

        wait_count = 0
        while self.latest_source_pts is None and rp.ok() and wait_count < 30:
            self.create_rate(10.0).sleep()
            wait_count += 1

        if self.latest_source_pts is None:
            goal_handle.abort()
            return PrecisionDock.Result(success=False, message="No ROI points")

        req_pos = goal_handle.request.target_pose.pose.position
        if req_pos.y <= 0.0 or req_pos.z <= 0.0:
            estimated = self.auto_estimate_v_funnel_params(self.latest_source_pts)
            if estimated is not None:
                w, l, ang = estimated
            else:
                w = self.get_parameter('charger_width').value
                l = self.get_parameter('wing_length').value
                ang = self.get_parameter('wing_angle_deg').value
        else:
            w, l, ang = req_pos.y, req_pos.z, self.get_parameter('wing_angle_deg').value

        safe_gap = self.robot_rear_length + 0.005
        gap = req_pos.x if req_pos.x > 0.0 else safe_gap
        active_target_pts = self.generate_v_funnel_target(w, l, ang, gap)

        feedback_msg = PrecisionDock.Feedback()
        rate = self.create_rate(20.0)
        last_time = self.get_clock().now()

        current_transform = np.identity(3)
        dist_err = 999.0
        is_first_frame = True
        icp_fail_count = 0  # [수정 3] ICP 실패 카운트 변수 추가

        while rp.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_stop()
                return PrecisionDock.Result(success=False, message="Canceled")

            if self.latest_source_pts is None:
                self.publish_stop()
                # [수정 4] 파라미터화된 사각지대 성공 판정 거리 사용
                if dist_err < self.blind_spot_dist:
                    goal_handle.succeed()
                    return PrecisionDock.Result(success=True, message="Docked (Blind Spot)")
                rate.sleep()
                continue

            current_time = self.get_clock().now()
            dt = (current_time - last_time).nanoseconds / 1e9
            last_time = current_time

            pts = self.latest_source_pts
            if len(pts) > 0:
                lidar_to_wall_dist = abs(np.min(pts[:, 0]))
                rear_to_wall_dist = lidar_to_wall_dist - self.robot_rear_length

                # [수정 5] 파라미터화된 안전 정지 거리 사용
                if rear_to_wall_dist <= self.safety_stop_dist:
                    self.publish_stop()
                    goal_handle.succeed()
                    self.get_logger().info('🛑 후미 완전 밀착 안전 정지 -> 안착 완료')
                    return PrecisionDock.Result(success=True, message="Safety stopped")

            search_r = 1.5 if is_first_frame else 0.35
            current_transform, fitness, match_cnt = icp_2d(
                self.latest_source_pts, active_target_pts,
                initial_transform=current_transform, max_iterations=35, search_radius=search_r
            )

            # [수정 6] 매칭 연속 실패 시 ICP 초기화 (오작동 방지 Recovery)
            if match_cnt < 10:
                icp_fail_count += 1
                if icp_fail_count > 5:
                    self.get_logger().warn('ICP 매칭 장기 실패, 초기 위치로 복원 시도!')
                    current_transform = np.identity(3)
                    is_first_frame = True
                    icp_fail_count = 0
                self.publish_stop()
                rate.sleep()
                continue
            
            icp_fail_count = 0 # 매칭 성공 시 실패 카운트 초기화
            is_first_frame = False

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
                self.publish_stop()
                goal_handle.succeed()
                return PrecisionDock.Result(success=True, message="Successfully docked")

            lookahead_dist = 0.25
            raw_correction = math.atan2(rel_y, lookahead_dist)
            clamped_correction = float(np.clip(raw_correction, -math.radians(35.0), math.radians(35.0)))
            total_heading_err = normalize_angle(rel_yaw - clamped_correction)

            # [수정 7] 파라미터화된 제자리 정렬 각도 사용
            if abs(total_heading_err) > math.radians(self.heading_align_deg):
                v_cmd = 0.0
                w_raw = self.angular_pid.update(total_heading_err, dt)
                w_cmd = float(np.clip(w_raw, -0.35, 0.35))
            else:
                max_v, max_w, min_v = 0.035, 0.30, 0.008
                if dist_err < 0.35:
                    max_v, max_w = 0.012, 0.20

                heading_damping = max(0.0, math.cos(total_heading_err))
                speed_mag = self.linear_pid.update(dist_err, dt)

                v_cmd = -float(np.clip(speed_mag * heading_damping, min_v, max_v))
                w_raw = self.angular_pid.update(total_heading_err, dt)
                w_cmd = float(np.clip(w_raw, -max_w, max_w))

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = v_cmd
            twist_msg.twist.angular.z = w_cmd
            self.cmd_vel_pub.publish(twist_msg)

            rate.sleep()

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