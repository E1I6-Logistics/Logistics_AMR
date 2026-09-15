#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


class ReadinessChecker(Node):
    def __init__(self, mode, robot_name, timeout):
        super().__init__(f'{robot_name}_{mode}_readiness_checker')
        self.mode = mode
        self.robot_name = robot_name
        self.deadline = time.monotonic() + timeout
        self.odom_received = False
        self.scan_received = False
        self.topic_subscriptions = []

        if mode == 'topics':
            self.topic_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f'/{robot_name}/odom',
                    self._on_odom,
                    qos_profile_sensor_data,
                )
            )
            self.topic_subscriptions.append(
                self.create_subscription(
                    LaserScan,
                    f'/{robot_name}/scan',
                    self._on_scan,
                    qos_profile_sensor_data,
                )
            )
        else:
            prefix = f'/{robot_name}'
            self.lifecycle_clients = [
                self.create_client(
                    Trigger,
                    f'{prefix}/lifecycle_manager_localization/is_active',
                ),
                self.create_client(
                    Trigger,
                    f'{prefix}/lifecycle_manager_navigation/is_active',
                ),
            ]

    def _on_odom(self, _message):
        self.odom_received = True

    def _on_scan(self, _message):
        self.scan_received = True

    def wait(self):
        if self.mode == 'topics':
            return self._wait_for_topics()
        return self._wait_for_nav2()

    def _wait_for_topics(self):
        while rclpy.ok() and time.monotonic() < self.deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.odom_received and self.scan_received:
                self.get_logger().info(
                    f'{self.robot_name}: odom and scan are ready'
                )
                return True

        self.get_logger().error(
            f'{self.robot_name}: timed out waiting for odom and scan'
        )
        return False

    def _wait_for_nav2(self):
        while rclpy.ok() and time.monotonic() < self.deadline:
            if not all(client.wait_for_service(timeout_sec=0.2)
                       for client in self.lifecycle_clients):
                continue

            responses = []
            for client in self.lifecycle_clients:
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                responses.append(future.result())

            if all(response is not None and response.success
                   for response in responses):
                self.get_logger().info(
                    f'{self.robot_name}: Nav2 localization and navigation are ACTIVE'
                )
                return True

            time.sleep(0.2)

        self.get_logger().error(
            f'{self.robot_name}: timed out waiting for Nav2 ACTIVE state'
        )
        return False


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('topics', 'nav2'), required=True)
    parser.add_argument('--robot-name', required=True)
    parser.add_argument('--timeout', type=float, default=60.0)
    return parser.parse_known_args()[0]


def main():
    args = _parse_args()
    rclpy.init()
    checker = ReadinessChecker(args.mode, args.robot_name, args.timeout)
    try:
        success = checker.wait()
    finally:
        checker.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
