#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputeRoute, FollowPath


class RouteGoalExecutor(Node):

    def __init__(self):
        super().__init__('route_goal_executor')

        # ==========================================
        # Parameters
        # ==========================================
        self.declare_parameter('start_node', 6)
        self.declare_parameter('goal_node', 13)

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

        # ==========================================
        # Action Clients
        # ==========================================
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

        self.started = False

        self.timer = self.create_timer(
            1.0,
            self.start_navigation
        )

    # ==========================================
    # Navigation 시작
    # ==========================================
    def start_navigation(self):

        if self.started:
            return

        self.started = True
        self.timer.cancel()

        self.get_logger().info(
            f'Route request: '
            f'{self.start_node} -> {self.goal_node}'
        )

        self.send_compute_route()

    # ==========================================
    # ComputeRoute 요청
    # ==========================================
    def send_compute_route(self):

        self.get_logger().info(
            'Waiting for /compute_route action server...'
        )

        if not self.compute_route_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                '/compute_route action server not available'
            )
            return

        goal_msg = ComputeRoute.Goal()

        # ------------------------------------------
        # Node ID 기반 Route 계산
        # ------------------------------------------
        goal_msg.use_poses = False

        goal_msg.start_id = int(self.start_node)
        goal_msg.goal_id = int(self.goal_node)

        self.get_logger().info(
            f'Sending ComputeRoute: '
            f'{self.start_node} -> {self.goal_node}'
        )

        future = self.compute_route_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self.compute_route_goal_response
        )

    # ==========================================
    # ComputeRoute Goal 응답
    # ==========================================
    def compute_route_goal_response(self, future):

        goal_handle = future.result()

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

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.compute_route_result
        )

    # ==========================================
    # ComputeRoute 결과
    # ==========================================
    def compute_route_result(self, future):

        wrapped_result = future.result()

        status = wrapped_result.status
        result = wrapped_result.result

        self.get_logger().info(
            f'ComputeRoute status = {status}'
        )

        self.get_logger().info(
            f'ComputeRoute error_code = '
            f'{result.error_code}'
        )

        # ------------------------------------------
        # Route 정보 출력
        # ------------------------------------------
        if hasattr(result, 'route'):

            route = result.route

            self.get_logger().info(
                f'Route cost = {route.route_cost}'
            )

            if len(route.nodes) > 0:

                node_ids = [
                    str(node.nodeid)
                    for node in route.nodes
                ]

                self.get_logger().info(
                    'Route nodes: '
                    + ' -> '.join(node_ids)
                )

            if len(route.edges) > 0:

                for edge in route.edges:

                    self.get_logger().info(
                        f'Edge {edge.edgeid}: '
                        f'{edge.start} -> {edge.end}'
                    )

        # ------------------------------------------
        # Path 검증
        # ------------------------------------------
        if result.error_code != 0:

            self.get_logger().error(
                'ComputeRoute failed: '
                f'error_code={result.error_code}'
            )
            return

        if len(result.path.poses) == 0:

            self.get_logger().error(
                'ComputeRoute returned empty path'
            )
            return

        self.get_logger().info(
            f'Route successfully computed: '
            f'{len(result.path.poses)} poses'
        )

        # ------------------------------------------
        # 계산된 Path를 Controller에 전달
        # ------------------------------------------
        self.send_follow_path(
            result.path
        )

    # ==========================================
    # FollowPath 요청
    # ==========================================
    def send_follow_path(self, path):

        self.get_logger().info(
            'Waiting for /follow_path action server...'
        )

        if not self.follow_path_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                '/follow_path action server not available'
            )
            return

        goal_msg = FollowPath.Goal()

        goal_msg.path = path

        # burger_route.yaml의 controller_plugins
        # ["FollowPath"]와 맞춤
        goal_msg.controller_id = 'FollowPath'

        self.get_logger().info(
            f'Sending {len(path.poses)} poses '
            f'to FollowPath'
        )

        future = self.follow_path_client.send_goal_async(
            goal_msg,
            feedback_callback=self.follow_path_feedback
        )

        future.add_done_callback(
            self.follow_path_goal_response
        )

    # ==========================================
    # FollowPath Goal 응답
    # ==========================================
    def follow_path_goal_response(self, future):

        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error(
                'FollowPath goal handle is None'
            )
            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'FollowPath goal rejected'
            )
            return

        self.get_logger().info(
            'FollowPath goal accepted'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.follow_path_result
        )

    # ==========================================
    # FollowPath Feedback
    # ==========================================
    def follow_path_feedback(self, feedback_msg):

        feedback = feedback_msg.feedback

        if hasattr(feedback, 'distance_to_goal'):

            self.get_logger().info(
                f'Distance to goal: '
                f'{feedback.distance_to_goal:.2f} m'
            )

    # ==========================================
    # FollowPath Result
    # ==========================================
    def follow_path_result(self, future):

        wrapped_result = future.result()

        status = wrapped_result.status
        result = wrapped_result.result

        self.get_logger().info(
            f'FollowPath finished: '
            f'status={status}'
        )

        if hasattr(result, 'error_code'):

            self.get_logger().info(
                f'FollowPath error_code='
                f'{result.error_code}'
            )

        if status == 4:
            self.get_logger().info(
                'Navigation succeeded!'
            )
        else:
            self.get_logger().warn(
                'Navigation did not finish successfully.'
            )


def main(args=None):

    rclpy.init(args=args)

    node = RouteGoalExecutor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()