#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_graph_waypoint_bt/geometry.hpp"

namespace graph_bt {
using Pose = geometry_msgs::msg::PoseStamped;
using Goals = std::vector<Pose>;

inline Point2 point(const Pose & p) {
  return {p.pose.position.x, p.pose.position.y};
}
inline bool samePose(const Pose & a, const Pose & b) {
  constexpr double e = 1e-5;
  return a.header.frame_id == b.header.frame_id &&
    distance(point(a), point(b)) < e &&
    std::abs(a.pose.orientation.z - b.pose.orientation.z) < e &&
    std::abs(a.pose.orientation.w - b.pose.orientation.w) < e &&
    std::abs(a.pose.orientation.x - b.pose.orientation.x) < e &&
    std::abs(a.pose.orientation.y - b.pose.orientation.y) < e;
}
inline bool sameGoals(const Goals & a, const Goals & b) {
  if (a.size() != b.size()) {return false;}
  for (size_t i = 0; i < a.size(); ++i) {
    if (!samePose(a[i], b[i])) {return false;}
  }
  return true;
}

// Replaces RemovePassedGoals. It never removes the final goal of the active segment.
class PruneGraphGoals final : public BT::ActionNodeBase {
public:
  PruneGraphGoals(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config) {
    tf_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    transform_tolerance_ = node_->get_parameter("transform_tolerance").as_double();
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<Goals>("input_goals"),
      BT::OutputPort<Goals>("output_goals"),
      BT::InputPort<double>("near_radius", 0.04, "Near-center tolerance (m)"),
      BT::InputPort<double>("lateral_tolerance", 0.06, "Cross-track tolerance (m)"),
      BT::InputPort<double>("max_pass_distance", 0.12, "Maximum past-goal distance (m)"),
      BT::InputPort<double>("pass_margin", 0.0, "Distance beyond waypoint plane (m)"),
      BT::InputPort<std::string>("global_frame", "map", "Pose lookup frame fallback"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint", "Robot base TF frame"),
      BT::InputPort<int>("segment_id", 0, "Segment generation to reset state")
    };
  }

private:
  BT::NodeStatus tick() override {
    Goals goals;
    if (!getInput("input_goals", goals)) {return BT::NodeStatus::FAILURE;}
    if (goals.size() <= 1) {
      setOutput("output_goals", goals);
      return BT::NodeStatus::SUCCESS;
    }

    double near_radius = 0.04, lateral = 0.06, max_pass = 0.12, pass_margin = 0.0;
    std::string global_frame = "map", base_frame = "base_footprint";
    int segment_id = 0;
    getInput("near_radius", near_radius);
    getInput("lateral_tolerance", lateral);
    getInput("max_pass_distance", max_pass);
    getInput("pass_margin", pass_margin);
    getInput("global_frame", global_frame);
    getInput("robot_base_frame", base_frame);
    getInput("segment_id", segment_id);
    if (near_radius <= 0 || lateral <= 0 || max_pass <= 0 || pass_margin < 0) {
      RCLCPP_ERROR(node_->get_logger(), "PruneGraphGoals invalid parameters");
      return BT::NodeStatus::FAILURE;
    }

    const std::string frame = goals.front().header.frame_id.empty() ?
      global_frame : goals.front().header.frame_id;
    Pose robot;
    if (!nav2_util::getCurrentPose(robot, *tf_, frame, base_frame, transform_tolerance_)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "PruneGraphGoals: cannot get robot TF pose");
      return BT::NodeStatus::FAILURE;
    }

    if (segment_id != last_segment_id_) {
      have_anchor_ = false;
      last_segment_id_ = segment_id;
    }
    const Point2 rp = point(robot);
    size_t removed = 0;
    while (goals.size() > 1) {
      const Point2 gp = point(goals.front());
      if (!have_anchor_ || distance(gp, current_goal_) > 1e-5) {
        current_goal_ = gp;
        anchor_ = rp;
        have_anchor_ = true;
      }
      auto passage = measurePassage(anchor_, gp, rp);
      // Degenerate segment: only remove if practically at the point.
      const bool at_same_point = !passage.valid && passage.distance_m <= 0.015;
      if (!at_same_point && !shouldPrune(passage, near_radius, lateral, max_pass, pass_margin)) {
        break;
      }
      RCLCPP_INFO(node_->get_logger(),
        "PruneGraphGoals: passed (%.3f, %.3f), dist=%.3f, lateral=%.3f, progress=%.3f",
        gp.x, gp.y, passage.distance_m, passage.lateral_m, passage.progress_m);
      goals.erase(goals.begin());
      ++removed;
      anchor_ = gp;  // The previous waypoint is the start of the next edge.
      if (goals.size() > 1) { current_goal_ = point(goals.front()); }
    }
    (void)removed;
    setOutput("output_goals", goals);
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {
    have_anchor_ = false;
    last_segment_id_ = -1;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Node::SharedPtr node_;
  double transform_tolerance_{0.5};
  bool have_anchor_{false};
  Point2 anchor_{0, 0}, current_goal_{0, 0};
  int last_segment_id_{-1};
};

// Executes one existing planner/controller PipelineSequence child per turning segment.
// Does NOT create ROS Actions; the original NavigateThroughPoses Action remains active.
class TurnSegmentControl final : public BT::ControlNode {
public:
  TurnSegmentControl(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    transform_tolerance_ = node_->get_parameter("transform_tolerance").as_double();
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<Goals>("input_goals", "Remaining navigator goals"),
      BT::OutputPort<Goals>("remaining_goals", "Remaining navigator goals for feedback"),
      BT::OutputPort<Goals>("segment_goals", "Active navigation segment"),
      BT::OutputPort<int>("segment_id", "Segment ID for passage-state reset"),
      BT::InputPort<double>("turn_angle_deg", 45.0, "Minimum turn angle (degrees)"),
      BT::InputPort<double>("min_leg_length", 0.07, "Ignore turns on shorter legs (m)")
    };
  }

private:
  void resetState() {
    initialized_ = false;
    segments_.clear();
    original_goals_.clear();
    expected_goals_.clear();
    current_segment_ = 0;
  }

  void halt() override {
    BT::ControlNode::halt();
    resetState();
  }

  void initialize(const Goals & goals) {
    original_goals_ = goals;
    expected_goals_ = goals;
    segments_.clear();
    double min_angle = 45.0, min_leg = 0.07;
    getInput("turn_angle_deg", min_angle);
    getInput("min_leg_length", min_leg);
    min_angle = std::clamp(min_angle, 1.0, 179.0);
    min_leg = std::max(0.0, min_leg);
    // Infer the incoming direction from current pose for the first waypoint.
    Pose robot;
    const std::string frame = goals.front().header.frame_id.empty() ?
      std::string("map") : goals.front().header.frame_id;
    const bool have_robot_pose = nav2_util::getCurrentPose(
      robot, *tf_, frame, "base_footprint", transform_tolerance_);
    if (!have_robot_pose) {
      RCLCPP_WARN(node_->get_logger(),
        "TurnSegmentControl: no robot pose, first waypoint turn cannot be classified");
    }
    std::vector<Point2> points;
    points.reserve(goals.size());
    for (const auto & goal : goals) {points.push_back(point(goal));}
    const Point2 incoming = have_robot_pose ? point(robot) : Point2{0, 0};
    segments_ = splitAtTurns(points, min_angle, min_leg,
      have_robot_pose ? &incoming : nullptr);
    current_segment_ = 0;
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(),
      "TurnSegmentControl: %zu goals, %zu segments", goals.size(), segments_.size());
    publishSegment();
  }

  void publishSegment() {
    if (current_segment_ >= segments_.size()) {return;}
    const auto bounds = segments_[current_segment_];
    Goals segment(
      original_goals_.begin() + bounds.first,
      original_goals_.begin() + bounds.second + 1);
    // At intermediate turn points, the segment's final target must face the next edge.
    if (bounds.second + 1 < original_goals_.size()) {
      const double yaw = heading(point(original_goals_[bounds.second]),
        point(original_goals_[bounds.second + 1]));
      auto & q = segment.back().pose.orientation;
      q.x = 0.0;
      q.y = 0.0;
      q.z = std::sin(0.5 * yaw);
      q.w = std::cos(0.5 * yaw);
    }
    setOutput("segment_goals", segment);
    setOutput("segment_id", static_cast<int>(current_segment_ + 1));
    RCLCPP_INFO(node_->get_logger(),
      "TurnSegmentControl: start segment %zu/%zu, %zu goals, endpoint=(%.3f,%.3f)",
      current_segment_ + 1, segments_.size(), segment.size(),
      segment.back().pose.position.x, segment.back().pose.position.y);
  }

  BT::NodeStatus tick() override {
    if (childrenCount() != 1) {
      RCLCPP_ERROR(node_->get_logger(), "TurnSegmentControl requires exactly one child");
      return BT::NodeStatus::FAILURE;
    }
    Goals incoming;
    if (!getInput("input_goals", incoming)) {return BT::NodeStatus::FAILURE;}

    if (!initialized_ || !sameGoals(incoming, expected_goals_)) {
      // Goal preemption: cancel active child before replacing segment data.
      if (initialized_) {haltChild(0);}
      if (incoming.empty()) {return BT::NodeStatus::FAILURE;}
      initialize(incoming);
    }

    setStatus(BT::NodeStatus::RUNNING);
    const auto status = children_nodes_.front()->executeTick();
    if (status == BT::NodeStatus::RUNNING) {return status;}
    if (status == BT::NodeStatus::FAILURE) {
      haltChild(0);
      return BT::NodeStatus::FAILURE;
    }
    if (status != BT::NodeStatus::SUCCESS) {
      return BT::NodeStatus::FAILURE;
    }
    haltChild(0);
    const size_t completed_end = segments_[current_segment_].second;
    ++current_segment_;
    if (current_segment_ == segments_.size()) {
      // Leave last goal in blackboard for navigator feedback while BT finishes.
      RCLCPP_INFO(node_->get_logger(), "TurnSegmentControl: all segments completed");
      return BT::NodeStatus::SUCCESS;
    }

    expected_goals_.assign(original_goals_.begin() + completed_end + 1,
      original_goals_.end());
    setOutput("remaining_goals", expected_goals_);
    publishSegment();
    return BT::NodeStatus::RUNNING;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_{0.5};
  bool initialized_{false};
  Goals original_goals_, expected_goals_;
  std::vector<std::pair<size_t, size_t>> segments_;
  size_t current_segment_{0};
};
}  // namespace graph_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<graph_bt::PruneGraphGoals>("PruneGraphGoals");
  factory.registerNodeType<graph_bt::TurnSegmentControl>("TurnSegmentControl");
}
