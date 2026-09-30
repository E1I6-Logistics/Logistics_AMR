import rclpy

from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)


class LogitlePosePublisher(Node):

    def __init__(self):
        super().__init__('logitle_pose_publisher')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_footprint')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('topic_name', 'logitle_pose')

        self.target_frame = (
            self.get_parameter('target_frame')
            .get_parameter_value()
            .string_value
        )

        self.source_frame = (
            self.get_parameter('source_frame')
            .get_parameter_value()
            .string_value
        )

        self.publish_rate_hz = (
            self.get_parameter('publish_rate_hz')
            .get_parameter_value()
            .double_value
        )

        self.topic_name = (
            self.get_parameter('topic_name')
            .get_parameter_value()
            .string_value
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.topic_name,
            10,
        )

        timer_period = 1.0 / self.publish_rate_hz

        self.timer = self.create_timer(
            timer_period,
            self.publish_pose,
        )

        self.get_logger().info(
            f'Publishing {self.target_frame} -> '
            f'{self.source_frame} as /{self.topic_name} '
            f'at {self.publish_rate_hz:.1f} Hz'
        )

    def publish_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time(),
            )

        except TransformException:
            # Nav2 / AMCL이 아직 준비되지 않았으면
            # TF가 생길 때까지 기다린다.
            return

        msg = PoseStamped()

        msg.header.stamp = transform.header.stamp
        msg.header.frame_id = self.target_frame

        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z

        msg.pose.orientation.x = transform.transform.rotation.x
        msg.pose.orientation.y = transform.transform.rotation.y
        msg.pose.orientation.z = transform.transform.rotation.z
        msg.pose.orientation.w = transform.transform.rotation.w

        self.pose_publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = LogitlePosePublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()