#!/usr/bin/env python3

import json
import math
from functools import partial

import yaml

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def load_robot_names(config_path):
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    robots = config.get('robots', [])
    names = [robot.get('name') for robot in robots]
    if not names or any(not name for name in names):
        raise RuntimeError(f'No valid robot names configured in {config_path}')
    if len(names) != len(set(names)):
        raise RuntimeError(f'Robot names must be unique in {config_path}')
    return names


def validate_goal_pose(goal_pose, required_frame):
    if goal_pose.header.frame_id != required_frame:
        return (
            f'goal frame must be {required_frame!r}; '
            f'got {goal_pose.header.frame_id!r}'
        )

    position = goal_pose.pose.position
    orientation = goal_pose.pose.orientation
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        return 'goal pose contains a non-finite value'

    quaternion_norm = math.sqrt(
        orientation.x ** 2
        + orientation.y ** 2
        + orientation.z ** 2
        + orientation.w ** 2
    )
    if quaternion_norm < 1.0e-6:
        return 'goal orientation quaternion is invalid'
    return None


class MultiRobotGoalDispatcher(Node):
    def __init__(self):
        super().__init__('multi_robot_goal_dispatcher')

        self.declare_parameter('robots_file', '')
        self.declare_parameter('selected_robot', '')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('action_server_timeout', 1.0)

        robots_file = self.get_parameter('robots_file').value
        if not robots_file:
            raise RuntimeError('robots_file parameter is required')

        self.robot_names = load_robot_names(robots_file)
        self.goal_frame = self.get_parameter('goal_frame').value
        self.action_server_timeout = float(
            self.get_parameter('action_server_timeout').value
        )
        initial_robot = self.get_parameter('selected_robot').value
        self.selected_robot = ''
        if initial_robot:
            if initial_robot not in self.robot_names:
                raise RuntimeError(
                    f'selected_robot must be one of {self.robot_names}; '
                    f'got {initial_robot!r}'
                )
            self.selected_robot = initial_robot

        self.action_clients = {
            robot_name: ActionClient(
                self,
                NavigateToPose,
                f'/{robot_name}/navigate_to_pose',
            )
            for robot_name in self.robot_names
        }
        self.pending_robots = set()
        self.active_goal_handles = {}
        self.last_feedback_log_ns = {}

        self.status_publisher = self.create_publisher(
            String,
            '/multi_robot/goal_status',
            10,
        )
        robot_list_qos = QoSProfile(depth=1)
        robot_list_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        robot_list_qos.reliability = ReliabilityPolicy.RELIABLE
        self.robot_list_publisher = self.create_publisher(
            String,
            '/multi_robot/robots',
            robot_list_qos,
        )
        self.create_subscription(
            String,
            '/multi_robot/selected_robot',
            self._selected_robot_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/multi_robot/goal_pose',
            self._goal_pose_callback,
            10,
        )
        self.create_subscription(
            String,
            '/multi_robot/cancel_goal',
            self._cancel_goal_callback,
            10,
        )

        self.get_logger().info(
            'Goal dispatcher ready for: ' + ', '.join(self.robot_names)
        )
        robot_list_message = String()
        robot_list_message.data = json.dumps({'robots': self.robot_names})
        self.robot_list_publisher.publish(robot_list_message)
        if self.selected_robot:
            self._publish_status('selected', self.selected_robot)
        else:
            self.get_logger().info(
                'Select a robot on /multi_robot/selected_robot before sending a goal'
            )

    def _publish_status(self, state, robot='', detail=''):
        message = String()
        message.data = json.dumps(
            {
                'state': state,
                'robot': robot,
                'detail': detail,
            },
            ensure_ascii=False,
        )
        self.status_publisher.publish(message)

    def _reject(self, detail, robot=''):
        self.get_logger().error(detail)
        self._publish_status('rejected', robot, detail)

    def _selected_robot_callback(self, message):
        robot_name = message.data.strip()
        if robot_name not in self.robot_names:
            self._reject(
                f'Unknown robot {robot_name!r}; choose one of {self.robot_names}',
                robot_name,
            )
            return

        self.selected_robot = robot_name
        self.get_logger().info(f'Selected robot: {robot_name}')
        self._publish_status('selected', robot_name)

    def _goal_pose_callback(self, goal_pose):
        robot_name = self.selected_robot
        if not robot_name:
            self._reject('No robot selected')
            return

        validation_error = validate_goal_pose(goal_pose, self.goal_frame)
        if validation_error:
            self._reject(validation_error, robot_name)
            return

        robot_busy = (
            robot_name in self.pending_robots
            or robot_name in self.active_goal_handles
        )
        if robot_busy:
            self._reject(
                f'{robot_name} already has an active or pending goal',
                robot_name,
            )
            return

        action_client = self.action_clients[robot_name]
        if not action_client.wait_for_server(
            timeout_sec=self.action_server_timeout
        ):
            self._reject(
                f'/{robot_name}/navigate_to_pose is not ready',
                robot_name,
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.pending_robots.add(robot_name)
        self._publish_status('sending', robot_name)
        self.get_logger().info(
            f'Sending goal to {robot_name}: '
            f'x={goal_pose.pose.position.x:.3f}, '
            f'y={goal_pose.pose.position.y:.3f}'
        )

        send_future = action_client.send_goal_async(
            goal,
            feedback_callback=partial(self._feedback_callback, robot_name),
        )
        send_future.add_done_callback(
            partial(self._goal_response_callback, robot_name)
        )

    def _goal_response_callback(self, robot_name, future):
        self.pending_robots.discard(robot_name)
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self._reject(f'Failed to send goal: {error}', robot_name)
            return

        if not goal_handle.accepted:
            self._reject('Nav2 rejected the goal', robot_name)
            return

        self.active_goal_handles[robot_name] = goal_handle
        self.get_logger().info(f'{robot_name}: goal accepted')
        self._publish_status('accepted', robot_name)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._result_callback, robot_name)
        )

    def _feedback_callback(self, robot_name, feedback_message):
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.last_feedback_log_ns.get(robot_name, 0)
        if now_ns - last_ns < 1_000_000_000:
            return
        self.last_feedback_log_ns[robot_name] = now_ns

        distance = feedback_message.feedback.distance_remaining
        self.get_logger().info(
            f'{robot_name}: distance remaining {distance:.2f} m'
        )
        self._publish_status(
            'active',
            robot_name,
            f'{distance:.2f} m remaining',
        )

    def _result_callback(self, robot_name, future):
        self.active_goal_handles.pop(robot_name, None)
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:  # noqa: BLE001
            self._reject(f'Failed to receive goal result: {error}', robot_name)
            return

        states = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }
        state = states.get(status, f'finished_status_{status}')
        self.get_logger().info(f'{robot_name}: goal {state}')
        self._publish_status(state, robot_name)

    def _cancel_goal_callback(self, message):
        robot_name = message.data.strip() or self.selected_robot
        if robot_name not in self.robot_names:
            self._reject(
                f'Unknown robot {robot_name!r}; choose one of {self.robot_names}',
                robot_name,
            )
            return

        goal_handle = self.active_goal_handles.get(robot_name)
        if goal_handle is None:
            self._reject(f'{robot_name} has no active goal', robot_name)
            return

        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            partial(self._cancel_response_callback, robot_name)
        )

    def _cancel_response_callback(self, robot_name, future):
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self._reject(f'Failed to cancel goal: {error}', robot_name)
            return

        if response.goals_canceling:
            self.get_logger().info(f'{robot_name}: cancel requested')
            self._publish_status('canceling', robot_name)
        else:
            self._reject('Nav2 did not accept the cancel request', robot_name)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MultiRobotGoalDispatcher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
