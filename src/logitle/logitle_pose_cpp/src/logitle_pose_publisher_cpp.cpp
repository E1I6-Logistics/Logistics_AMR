#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

class LogitlePosePublisherCpp final : public rclcpp::Node
{
public:
  LogitlePosePublisherCpp()
  : rclcpp::Node("logitle_pose_publisher_cpp")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    source_frame_ = declare_parameter<std::string>("source_frame", "base_footprint");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    topic_name_ = declare_parameter<std::string>("topic_name", "logitle_pose");

    // The Python implementation also requires a valid, positive timer rate.
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive and finite");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());

    // Use the node's executor for TF callbacks, as in rclpy.spin(node).
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_, this, false);

    // rclpy.create_publisher(PoseStamped, topic, 10) defaults to
    // KeepLast(10), RELIABLE, VOLATILE.
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)));

    // Use this node's ROS clock (not a wall timer), matching rclpy.create_timer.
    timer_ = rclcpp::create_timer(
      get_node_base_interface(),
      get_node_timers_interface(),
      get_clock(),
      rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_),
      std::bind(&LogitlePosePublisherCpp::publish_pose, this));

    RCLCPP_INFO(
      get_logger(), "Publishing %s -> %s as %s at %.1f Hz",
      target_frame_.c_str(), source_frame_.c_str(),
      topic_name_.c_str(), publish_rate_hz_);
  }

private:
  void publish_pose()
  {
    geometry_msgs::msg::TransformStamped transform;

    try {
      // TimePointZero retrieves the latest available transform, NOT
      // necessarily a transform measured at the current time.
      transform = tf_buffer_->lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      // Match the Python node: silently skip publishing until TF is available.
      // Deliberately do not reject stale transforms in the parity baseline.
      return;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = transform.header.stamp;
    msg.header.frame_id = target_frame_;

    msg.pose.position.x = transform.transform.translation.x;
    msg.pose.position.y = transform.transform.translation.y;
    msg.pose.position.z = transform.transform.translation.z;
    msg.pose.orientation.x = transform.transform.rotation.x;
    msg.pose.orientation.y = transform.transform.rotation.y;
    msg.pose.orientation.z = transform.transform.rotation.z;
    msg.pose.orientation.w = transform.transform.rotation.w;

    pose_publisher_->publish(msg);
  }

  std::string target_frame_;
  std::string source_frame_;
  std::string topic_name_;
  double publish_rate_hz_{5.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LogitlePosePublisherCpp>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("logitle_pose_publisher_cpp"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
