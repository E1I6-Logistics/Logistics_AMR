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
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import tf2_ros

from turtlebot3_my_msg.action import PrecisionDock


# ---------------------------------------------------------
# [상태 머신 정의]
# ---------------------------------------------------------
class DockingState(Enum):
    INIT = 0               # 점군 수신 대기 및 도크 모델 초기화
    ALIGN_IN_PLACE = 1     # 1단계: 제자리 PID 회전으로 완벽한 평행(Yaw) 맞추기
    BLIND_INSERT = 2       # 2단계: 오도메트리 기반 직진 PID 유지 일직선 후진
    COMPLETED = 3          # 도킹 완료
    FAILED = 4             # 도킹 실패/중단


def normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화합니다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw_from_quaternion(q) -> float:
    """쿼터니언에서 Yaw(라디안)를 계산합니다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------
# [개선된 PID 제어기] Anti-windup 및 불감대 처리 적용
# ---------------------------------------------------------
class PID:
    def __init__(self, p: float, i: float, d: float, out_min: float, out_max: float, i_limit: float = 0.05):
        self.p = p
        self.i = i
        self.d = d
        self.out_min = out_min
        self.out_max = out_max
        self.i_limit = i_limit  # 적분 누적 제한 (Anti-windup)
        self.previous_err = 0.0
        self.integral = 0.0

    def update(self, error: float, dt: float = 0.05) -> float:
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        # 목표치 근처 도달 시 적분 초기화 (오버슈트/채터링 방지)
        if abs(error) < math.radians(0.5):
            self.integral = 0.0
        else:
            self.integral += error * dt
            self.integral = float(np.clip(self.integral, -self.i_limit, self.i_limit))

        derivative = (error - self.previous_err) / dt
        self.previous_err = error

        output = (self.p * error) + (self.i * self.integral) + (self.d * derivative)
        return float(np.clip(output, self.out_min, self.out_max))

    def reset(self):
        self.previous_err = 0.0
        self.integral = 0.0


# ---------------------------------------------------------
# [2D ICP 정합 함수]
# ---------------------------------------------------------
def icp_2d(source_pts: np.ndarray, target_pts: np.ndarray, tree: cKDTree, initial_transform=None, max_iterations=30, search_radius=0.4):
    T = np.identity(3) if initial_transform is None else np.copy(initial_transform)
    src_h = np.hstack((source_pts, np.ones((source_pts.shape[0], 1))))

    distances = np.array([])
    matched_count = 0

    for _ in range(max_iterations):
        transformed = (T @ src_h.T).T[:, :2]
        distances, indices = tree.query(transformed)

        valid = distances < search_radius
        matched_count = int(np.sum(valid))
        if matched_count < 8:
            break

        pts_src = source_pts[valid]
        pts_tgt = target_pts[indices[valid]]

        centroid_src = np.mean(pts_src, axis=0)
        centroid_tgt = np.mean(pts_tgt, axis=0)

        H = (pts_src - centroid_src).T @ (pts_tgt - centroid_tgt)
        U, _, Vt = np.linalg.svd(H)

        # 2D SVD 반사(Reflection) 방지
        d = np.linalg.det(Vt.T @ U.T)
        if d < 0:
            S = np.array([[1.0, 0.0], [0.0, -1.0]])
            R = Vt.T @ S @ U.T
        else:
            R = Vt.T @ U.T

        t = centroid_tgt - (R @ centroid_src)
        T = np.identity(3)
        T[:2, :2] = R
        T[:2, 2] = t

    fitness = float(np.sum(distances < 0.05) / max(len(source_pts), 1)) if len(distances) > 0 else 0.0
    return T, fitness, matched_count


# ---------------------------------------------------------
# [정밀 도킹 액션 서버 노드]
# ---------------------------------------------------------
class PrecisionDockingServer(Node):
    def __init__(self):
        super().__init__('precision_docking_server')
        self.cb_group = ReentrantCallbackGroup()

        # TF 버퍼 및 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 액션 서버
        self._action_server = ActionServer(
            self,
            PrecisionDock,
            'precision_dock',
            execute_callback=self.execute_callback,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=self.cb_group
        )

        # 라이다 구독 및 속도 퍼블리셔
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data, callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # 파라미터 정의
        self.declare_parameter('charger_width', 0.15)
        self.declare_parameter('wing_length', 0.25)
        self.declare_parameter('wing_angle_deg', 45.0)
        self.declare_parameter('robot_rear_length', 0.15)   # base_link에서 범퍼 후단까지 거리
        self.declare_parameter('yaw_offset_deg', 0.0)       # 라이다 설치 영점 오차 오프셋
        self.declare_parameter('y_tolerance', 0.15)         # V-깔때기 입구 여유폭 반영 (15cm)

        self.latest_source_pts = None

        # PID 컨트롤러 복원
        self.align_pid = PID(p=0.8, i=0.03, d=0.08, out_min=-0.22, out_max=0.22, i_limit=0.04)
        self.latch_yaw_pid = PID(p=0.6, i=0.01, d=0.05, out_min=-0.12, out_max=0.12, i_limit=0.03)

        self.get_logger().info('PID 기반 V-Funnel 정밀 도킹 서버 준비 완료.')

    def generate_v_funnel_model(self, w: float, l: float, deg: float) -> np.ndarray:
        """도크 바닥 중심을 (0,0)으로 하는 로컬 타겟 포인트 클라우드 생성"""
        points = []
        half_w = w / 2.0
        # 바닥면
        for y in np.arange(-half_w, half_w + 0.005, 0.01):
            points.append([0.0, y])
        rad = math.radians(deg)
        # 양쪽 날개 (+X 방향으로 펼쳐짐)
        for d in np.arange(0.01, l + 0.005, 0.01):
            px = d * math.cos(rad)
            points.append([px, half_w + d * math.sin(rad)])
            points.append([px, -(half_w + d * math.sin(rad))])
        return np.array(points, dtype=np.float64)

    def scan_callback(self, msg: LaserScan):
        # base_scan -> base_link 변환 룩업
        try:
            transform = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rp.time.Time())
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            yaw_tf = get_yaw_from_quaternion(transform.transform.rotation)
        except Exception:
            tx, ty, yaw_tf = -0.032, 0.0, 0.0  # TB3 Burger 기본값

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # LDS-01 유효 거리 필터링 (최소 12cm 이상)
        valid = (ranges > 0.12) & (ranges < 1.2) & np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        # 후방 120도 영역 필터링
        rear_mask = np.abs(angles) >= math.radians(120.0)
        ranges = ranges[rear_mask]
        angles = angles[rear_mask]

        xs_sensor = ranges * np.cos(angles)
        ys_sensor = ranges * np.sin(angles)

        # base_link 좌표계 변환 투영
        xs = xs_sensor * math.cos(yaw_tf) - ys_sensor * math.sin(yaw_tf) + tx
        ys = xs_sensor * math.sin(yaw_tf) + ys_sensor * math.cos(yaw_tf) + ty

        # 후방 ROI (-0.80m ~ -0.15m, Y폭 +-0.35m)
        roi_mask = (xs < -0.15) & (xs > -0.80) & (np.abs(ys) < 0.35)
        if np.sum(roi_mask) >= 8:
            self.latest_source_pts = np.column_stack((xs[roi_mask], ys[roi_mask])).astype(np.float64)
        else:
            self.latest_source_pts = None

    def get_current_odom_pose(self):
        t = self.tf_buffer.lookup_transform('odom', 'base_link', rp.time.Time())
        return (t.transform.translation.x,
                t.transform.translation.y,
                get_yaw_from_quaternion(t.transform.rotation))

    def publish_cmd_vel(self, v: float, w: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def execute_callback(self, goal_handle):
        state = DockingState.INIT
        w = self.get_parameter('charger_width').value
        l = self.get_parameter('wing_length').value
        deg = self.get_parameter('wing_angle_deg').value
        rear_len = self.get_parameter('robot_rear_length').value
        yaw_offset = math.radians(self.get_parameter('yaw_offset_deg').value)
        y_tol = self.get_parameter('y_tolerance').value

        # 타겟 모델 및 KD-Tree 사전 빌드
        target_model = self.generate_v_funnel_model(w, l, deg)
        target_tree = cKDTree(target_model)

        default_transform = np.identity(3)
        default_transform[0, 2] = 0.45  # 초기 가배치 (후방 45cm)
        current_transform = np.copy(default_transform)

        rel_yaw_history = deque(maxlen=5)
        rate = self.create_rate(20.0)
        target_insert_dist = 0.0
        latched_yaw = 0.0
        start_x, start_y = 0.0, 0.0
        icp_fail_cnt = 0

        # PID 컨트롤러 리셋
        self.align_pid.reset()
        self.latch_yaw_pid.reset()

        feedback_msg = PrecisionDock.Feedback()
        last_time = self.get_clock().now()

        while rp.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_cmd_vel(0.0, 0.0)
                return PrecisionDock.Result(success=False, message="Canceled")

            current_time = self.get_clock().now()
            dt = (current_time - last_time).nanoseconds / 1e9
            last_time = current_time

            # -------------------------------------------------------------
            # [STATE 0: INIT]
            # -------------------------------------------------------------
            if state == DockingState.INIT:
                if self.latest_source_pts is not None:
                    state = DockingState.ALIGN_IN_PLACE
                    self.get_logger().info(">> 1단계: 제자리 PID 회전 정렬 시작")

            # -------------------------------------------------------------
            # [STATE 1: ALIGN_IN_PLACE] 제자리 회전으로 헤딩 일치
            # -------------------------------------------------------------
            elif state == DockingState.ALIGN_IN_PLACE:
                if self.latest_source_pts is None:
                    self.publish_cmd_vel(0.0, 0.0)
                    rate.sleep()
                    continue

                # ICP 매칭
                current_transform, fitness, match_cnt = icp_2d(
                    self.latest_source_pts, target_model, target_tree, initial_transform=current_transform
                )

                # 매칭 실패 시 발산 방지
                if match_cnt < 8 or fitness < 0.2:
                    icp_fail_cnt += 1
                    if icp_fail_cnt > 5:
                        current_transform = np.copy(default_transform)
                        icp_fail_cnt = 0
                    self.publish_cmd_vel(0.0, 0.0)
                    rate.sleep()
                    continue
                icp_fail_cnt = 0

                R_mat = current_transform[:2, :2]
                t_vec = current_transform[:2, 2]

                # 안전한 스칼라 추출
                r10 = float(R_mat.item((1, 0)))
                r00 = float(R_mat.item((0, 0)))
                dock_yaw_err = normalize_angle(-math.atan2(r10, r00) + yaw_offset)

                dock_pos = -R_mat.T @ t_vec
                dock_x = float(dock_pos.item(0))
                dock_y = float(dock_pos.item(1))

                rel_yaw_history.append(dock_yaw_err)
                avg_yaw_err = math.atan2(sum(math.sin(y) for y in rel_yaw_history),
                                         sum(math.cos(y) for y in rel_yaw_history))

                feedback_msg.distance_remaining = float(abs(dock_x))
                feedback_msg.angle_remaining = float(avg_yaw_err)
                goal_handle.publish_feedback(feedback_msg)

                # 각도 정렬 완료 판정 (1도 이내)
                if abs(avg_yaw_err) <= math.radians(1.0):
                    self.publish_cmd_vel(0.0, 0.0)

                    # 각도가 평행해진 상태에서 실제 측면 편차 검사
                    if abs(dock_y) > y_tol:
                        self.get_logger().error(f"정렬 완료 후 도크 Y축 편차 과다 ({dock_y * 100:.1f}cm > {y_tol * 100:.1f}cm). 도킹 중단!")
                        goal_handle.abort()
                        return PrecisionDock.Result(success=False, message="Lateral error too large")

                    # ICP 기반 실제 후진 거리 산출 (안전 마진 1.5cm)
                    target_insert_dist = max(0.0, abs(dock_x) - rear_len - 0.015)
                    try:
                        curr_x, curr_y, curr_yaw = self.get_current_odom_pose()
                        start_x, start_y = curr_x, curr_y
                        latched_yaw = curr_yaw
                        state = DockingState.BLIND_INSERT
                        self.get_logger().info(f">> 각도 정렬 완료 (최종 Y 편차: {dock_y * 100:.1f}cm). {target_insert_dist * 100:.1f}cm 후진 시작")
                    except Exception as e:
                        self.get_logger().warn(f"TF 조회 실패: {e}")
                    continue

                # 회전 정렬 PID 제어 및 모터 마찰 극복 데드밴드
                w_out = self.align_pid.update(avg_yaw_err, dt)
                if abs(avg_yaw_err) > math.radians(0.8) and abs(w_out) < 0.04:
                    w_cmd = 0.04 if w_out > 0 else -0.04
                else:
                    w_cmd = w_out

                self.publish_cmd_vel(0.0, w_cmd)

            # -------------------------------------------------------------
            # [STATE 2: BLIND_INSERT] 오도메트리 기반 일직선 후진
            # -------------------------------------------------------------
            elif state == DockingState.BLIND_INSERT:
                try:
                    curr_x, curr_y, curr_yaw = self.get_current_odom_pose()
                except Exception:
                    self.publish_cmd_vel(0.0, 0.0)
                    rate.sleep()
                    continue

                moved = math.hypot(curr_x - start_x, curr_y - start_y)
                remain = target_insert_dist - moved

                feedback_msg.distance_remaining = float(remain)
                feedback_msg.angle_remaining = float(normalize_angle(latched_yaw - curr_yaw))
                goal_handle.publish_feedback(feedback_msg)

                if remain <= 0.008:
                    self.publish_cmd_vel(0.0, 0.0)
                    state = DockingState.COMPLETED
                    goal_handle.succeed()
                    return PrecisionDock.Result(success=True, message="Docking Success")

                # 후진 직진성 유지 PID 제어
                yaw_err = normalize_angle(latched_yaw - curr_yaw)
                w_cmd = self.latch_yaw_pid.update(yaw_err, dt)
                v_cmd = -0.035  # Stall 방지를 위한 최소 후진 속도 (3.5 cm/s)

                self.publish_cmd_vel(v_cmd, w_cmd)

            rate.sleep()

        self.publish_cmd_vel(0.0, 0.0)
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