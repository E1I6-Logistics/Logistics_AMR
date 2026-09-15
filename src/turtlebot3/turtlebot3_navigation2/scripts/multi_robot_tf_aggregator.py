#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class MultiRobotTfAggregator(Node):
    """Republish namespaced robot TF streams for a single global RViz."""

    def __init__(self):
        super().__init__('multi_robot_tf_aggregator')

        # A non-empty default makes rclpy infer STRING_ARRAY. An empty list is
        # otherwise inferred as BYTE_ARRAY and rejects the launch override.
        robot_names = self.declare_parameter('robot_names', ['robot1']).value
        if not robot_names:
            raise RuntimeError('robot_names parameter must not be empty')

        dynamic_qos = QoSProfile(depth=100)
        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.dynamic_publisher = self.create_publisher(
            TFMessage,
            '/tf',
            dynamic_qos,
        )
        self.static_publisher = self.create_publisher(
            TFMessage,
            '/tf_static',
            static_qos,
        )

        self._static_transforms = {}
        self._tf_subscriptions = []

        for robot_name in robot_names:
            self._tf_subscriptions.append(
                self.create_subscription(
                    TFMessage,
                    f'/{robot_name}/tf',
                    self.dynamic_publisher.publish,
                    dynamic_qos,
                )
            )
            self._tf_subscriptions.append(
                self.create_subscription(
                    TFMessage,
                    f'/{robot_name}/tf_static',
                    self._on_static_tf,
                    static_qos,
                )
            )

        self.get_logger().info(
            'Aggregating TF for: ' + ', '.join(robot_names)
        )

    def _on_static_tf(self, message):
        for transform in message.transforms:
            self._static_transforms[transform.child_frame_id] = transform

        merged_message = TFMessage()
        merged_message.transforms = list(self._static_transforms.values())
        self.static_publisher.publish(merged_message)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotTfAggregator()
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
