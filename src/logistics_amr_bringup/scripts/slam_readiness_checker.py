#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


class SlamReadinessChecker(Node):
    def __init__(self, node_name, timeout):
        super().__init__('slam_readiness_checker')
        self.deadline = time.monotonic() + timeout
        self.client = self.create_client(GetState, f'{node_name}/get_state')

    def wait(self):
        while rclpy.ok() and time.monotonic() < self.deadline:
            if not self.client.wait_for_service(timeout_sec=0.2):
                continue

            future = self.client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            response = future.result()
            if (response is not None and
                    response.current_state.id == State.PRIMARY_STATE_ACTIVE):
                self.get_logger().info('SLAM Toolbox is ACTIVE')
                return True

            time.sleep(0.2)

        self.get_logger().error('Timed out waiting for SLAM Toolbox ACTIVE state')
        return False


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node-name', default='/slam_toolbox')
    parser.add_argument('--timeout', type=float, default=60.0)
    return parser.parse_known_args()[0]


def main():
    args = _parse_args()
    rclpy.init()
    checker = SlamReadinessChecker(args.node_name, args.timeout)
    success = False
    try:
        success = checker.wait()
    finally:
        checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
