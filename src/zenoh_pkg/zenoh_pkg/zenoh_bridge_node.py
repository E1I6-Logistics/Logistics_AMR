#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ZenohBridgeNode(Node):

    def __init__(self):
        super().__init__('zenoh_bridge_node')

        self.declare_parameter(
            'robot_id',
            'unknown_robot',
        )

        robot_id = str(self.get_parameter('robot_id').value)

        self.get_logger().info(f'Zenoh bridge node started: {robot_id}')

        # ==================================================
        # 추후 추가할 내용
        # ==================================================


def main(args=None):
    rclpy.init(args=args)
    node = ZenohBridgeNode()

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