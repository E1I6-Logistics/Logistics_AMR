#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path

from nav2_msgs.action import ComputeRoute, FollowPath


class RouteGoalExecutor(Node):

    def __init__(self):
        super().__init__('route_goal_executor')

        # =========================================================
        # Parameters
        # =========================================================

        self.declare_parameter('start_node', 0)
        self.declare_parameter('goal_node', 19)

        self.start_node = (
            self.get_parameter('start_node')
            .get_parameter_value()
            .integer_value
        )

        self.goal_node = (
            self.get_parameter('goal_node')
            .get_parameter_value()
            .integer_value
        )

        # =========================================================
        # Action Clients
        # =========================================================

        self.compute_route_client = ActionClient(
            self,
            ComputeRoute,
            '/compute_route'
        )

        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )

        # =========================================================
        # Debug Path Publisher
        #
        # Route Server가 실제로 생성한 nav_msgs/Path를
        # RViz에서 확인하기 위한 topic
        #
        # Transient Local:
        # RViz를 나중에 켜도 마지막 Path를 볼 수 있도록 함
        # =========================================================

        path_qos = QoSProfile(depth=1)

        path_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        path_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.route_path_pub = self.create_publisher(
            Path,
            '/route_path_debug',
            path_qos
        )

    # =============================================================
    # Main Execution
    # =============================================================

    def execute(self):

        # =========================================================
        # 1. Wait for ComputeRoute
        # =========================================================

        self.get_logger().info(
            'Waiting for /compute_route action server...'
        )

        while not self.compute_route_client.wait_for_server(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                '/compute_route not available, waiting...'
            )

        # =========================================================
        # 2. Create ComputeRoute Goal
        # =========================================================

        route_goal = ComputeRoute.Goal()

        route_goal.start_id = self.start_node
        route_goal.goal_id = self.goal_node

        # Node ID 기반 Route 계산
        route_goal.use_poses = False

        # 현재 위치 TF가 아니라
        # start_id를 시작점으로 사용
        route_goal.use_start = True

        self.get_logger().info(
            f'ComputeRoute request: '
            f'{self.start_node} -> {self.goal_node}'
        )

        # =========================================================
        # 3. Send ComputeRoute Goal
        # =========================================================

        send_future = (
            self.compute_route_client.send_goal_async(
                route_goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                'ComputeRoute goal handle is None'
            )
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'ComputeRoute goal rejected'
            )
            return

        self.get_logger().info(
            'ComputeRoute goal accepted'
        )

        # =========================================================
        # 4. Wait for ComputeRoute Result
        # =========================================================

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        route_response = result_future.result()

        if route_response is None:
            self.get_logger().error(
                'ComputeRoute returned no result'
            )
            return

        route_result = route_response.result

        # =========================================================
        # 5. Print ComputeRoute Result
        # =========================================================

        self.get_logger().info(
            f'ComputeRoute result: '
            f'status={route_response.status}, '
            f'error_code={route_result.error_code}'
        )

        # Nav2 Route Server 내부 오류
        if (
            route_result.error_code
            != ComputeRoute.Result.NONE
        ):
            self.get_logger().error(
                f'ComputeRoute FAILED: '
                f'status={route_response.status}, '
                f'error_code={route_result.error_code}'
            )

            self.print_route_error(
                route_result.error_code
            )

            return

        # ROS2 Action 상태 확인
        if (
            route_response.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().error(
                f'ComputeRoute action failed: '
                f'status={route_response.status}'
            )
            return

        # =========================================================
        # 6. Get Path
        # =========================================================

        path = route_result.path

        if len(path.poses) == 0:
            self.get_logger().error(
                'Computed route path is empty'
            )
            return

        self.get_logger().info(
            f'Route computed successfully: '
            f'{len(path.poses)} path poses'
        )

        self.get_logger().info(
            f'Route contains '
            f'{len(route_result.route.nodes)} nodes, '
            f'{len(route_result.route.edges)} edges'
        )

        # =========================================================
        # 7. Publish Debug Path
        #
        # ★ 여기서 RViz에 보이는 Path와
        #   아래 FollowPath에 보내는 Path가 완전히 동일함
        # =========================================================

        self.route_path_pub.publish(path)

        self.get_logger().info(
            f'Published computed route path '
            f'to /route_path_debug'
        )

        # =========================================================
        # 8. Wait for FollowPath
        # =========================================================

        self.get_logger().info(
            'Waiting for /follow_path action server...'
        )

        while not self.follow_path_client.wait_for_server(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                '/follow_path not available, waiting...'
            )

        # =========================================================
        # 9. Create FollowPath Goal
        # =========================================================

        follow_goal = FollowPath.Goal()

        # ★ Route Server가 계산한 바로 그 Path
        follow_goal.path = path

        # burger_route.yaml:
        #
        # controller_plugins:
        #   - "FollowPath"
        #
        follow_goal.controller_id = 'FollowPath'

        # 빈 문자열이면 Controller Server의
        # 기본 plugin 사용
        follow_goal.goal_checker_id = ''
        follow_goal.progress_checker_id = ''

        self.get_logger().warn(
            f'SENDING ROUTE PATH TO ROBOT: '
            f'{self.start_node} -> {self.goal_node}'
        )

        # =========================================================
        # 10. Send FollowPath Goal
        # =========================================================

        follow_send_future = (
            self.follow_path_client.send_goal_async(
                follow_goal,
                feedback_callback=self.follow_feedback
            )
        )

        rclpy.spin_until_future_complete(
            self,
            follow_send_future
        )

        follow_handle = (
            follow_send_future.result()
        )

        if follow_handle is None:
            self.get_logger().error(
                'FollowPath goal handle is None'
            )
            return

        if not follow_handle.accepted:
            self.get_logger().error(
                'FollowPath goal rejected'
            )
            return

        self.get_logger().info(
            'FollowPath accepted - robot is moving'
        )

        # =========================================================
        # 11. Wait for FollowPath Result
        # =========================================================

        follow_result_future = (
            follow_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            follow_result_future
        )

        follow_response = (
            follow_result_future.result()
        )

        if follow_response is None:
            self.get_logger().error(
                'FollowPath returned no result'
            )
            return

        follow_result = (
            follow_response.result
        )

        self.get_logger().info(
            f'FollowPath result: '
            f'status={follow_response.status}, '
            f'error_code={follow_result.error_code}'
        )

        # =========================================================
        # 12. Final Result
        # =========================================================

        if (
            follow_response.status
            == GoalStatus.STATUS_SUCCEEDED
            and
            follow_result.error_code
            == FollowPath.Result.NONE
        ):
            self.get_logger().info(
                'Route execution SUCCEEDED'
            )

        else:
            self.get_logger().error(
                f'Route execution FAILED: '
                f'status={follow_response.status}, '
                f'error_code='
                f'{follow_result.error_code}, '
                f'error_msg='
                f'"{follow_result.error_msg}"'
            )

    # =============================================================
    # FollowPath Feedback
    # =============================================================

    def follow_feedback(
        self,
        feedback_msg
    ):

        feedback = (
            feedback_msg.feedback
        )

        self.get_logger().info(
            f'distance_to_goal='
            f'{feedback.distance_to_goal:.3f} m, '
            f'speed='
            f'{feedback.speed:.3f} m/s'
        )

    # =============================================================
    # ComputeRoute Error Decoder
    # =============================================================

    def print_route_error(
        self,
        error_code
    ):

        error_map = {
            0: 'NONE',
            400: 'UNKNOWN',
            401: 'TF_ERROR',
            402: 'NO_VALID_GRAPH',
            403: 'INDETERMINANT_NODES_ON_GRAPH',
            404: 'TIMEOUT',
            405: 'NO_VALID_ROUTE',
            407: 'INVALID_EDGE_SCORER_USE',
        }

        error_name = error_map.get(
            error_code,
            'UNRECOGNIZED_ERROR'
        )

        self.get_logger().error(
            f'Route Server error: '
            f'{error_code} '
            f'({error_name})'
        )


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = RouteGoalExecutor()

    try:
        node.execute()

    except KeyboardInterrupt:
        node.get_logger().warn(
            'Route execution interrupted by user'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()