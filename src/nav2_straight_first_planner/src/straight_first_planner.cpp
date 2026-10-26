#include "nav2_straight_first_planner/straight_first_planner.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_straight_first_planner {
void StraightFirstPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  if (!node_) {throw std::runtime_error("Planner parent expired");}
  name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
  node_->declare_parameter(name_ + ".interpolation_resolution", 0.05);
  node_->declare_parameter(name_ + ".safety_radius", 0.12);
  node_->declare_parameter(name_ + ".max_cell_cost", 252);
  node_->declare_parameter(name_ + ".allow_unknown", false);
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".safety_radius", safety_radius_);
  node_->get_parameter(name_ + ".max_cell_cost", max_cell_cost_);
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  if (interpolation_resolution_ <= 0.0 || safety_radius_ < 0.0 || max_cell_cost_ < 0 || max_cell_cost_ > 252) {
    throw std::runtime_error("StraightFirst invalid configuration");
  }
  // Delegate to stock Smac2D. It reads its own existing `Smac2D.*` parameters.
  loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
    "nav2_core", "nav2_core::GlobalPlanner");
  astar_ = loader_->createSharedInstance("nav2_smac_planner::SmacPlanner2D");
  astar_->configure(parent, "Smac2D", tf, costmap_ros_);
  RCLCPP_INFO(node_->get_logger(), "%s: straight-first enabled, A* fallback=Smac2D", name_.c_str());
}
void StraightFirstPlanner::activate() {if (astar_) {astar_->activate();}}
void StraightFirstPlanner::deactivate() {if (astar_) {astar_->deactivate();}}
void StraightFirstPlanner::cleanup() {
  if (astar_) {astar_->cleanup(); astar_.reset();}
  loader_.reset(); costmap_ros_.reset(); node_.reset();
}
bool StraightFirstPlanner::isFootprintSafe(double x, double y) const {
  auto *cm = costmap_ros_->getCostmap();
  const auto res = cm->getResolution();
  // Conservative circle raster scan: reject even partially outside map.
  const int steps = static_cast<int>(std::ceil(safety_radius_ / res));
  for (int ix = -steps; ix <= steps; ++ix) {
    for (int iy = -steps; iy <= steps; ++iy) {
      const double dx = ix * res, dy = iy * res;
      if (dx * dx + dy * dy > safety_radius_ * safety_radius_) {continue;}
      unsigned int mx = 0, my = 0;
      if (!cm->worldToMap(x + dx, y + dy, mx, my)) {return false;}
      const unsigned char cost = cm->getCost(mx, my);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        if (!allow_unknown_) {return false;}
      } else if (cost > max_cell_cost_) {return false;}
    }
  }
  return true;
}
bool StraightFirstPlanner::isLineSafe(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  const std::function<bool()> &cancel_checker) const {
  auto *cm = costmap_ros_->getCostmap();
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double length = std::hypot(dx, dy);
  const double step = std::min(interpolation_resolution_, cm->getResolution() * 0.5);
  const size_t count = std::max<size_t>(1, static_cast<size_t>(std::ceil(length / step)));
  for (size_t i = 0; i <= count; ++i) {
    if (cancel_checker && cancel_checker()) {throw std::runtime_error("Planning canceled");}
    const double t = static_cast<double>(i) / static_cast<double>(count);
    if (!isFootprintSafe(start.pose.position.x + t * dx, start.pose.position.y + t * dy)) {
      return false;
    }
  }
  return true;
}
nav_msgs::msg::Path StraightFirstPlanner::straightPath(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal) const {
  nav_msgs::msg::Path path;
  path.header = goal.header;
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double length = std::hypot(dx, dy);
  const size_t count = std::max<size_t>(1, static_cast<size_t>(std::ceil(length / interpolation_resolution_)));
  tf2::Quaternion q;
  if (length > 1e-9) {
    q.setRPY(0, 0, std::atan2(dy, dx));
  } else {
    tf2::fromMsg(goal.pose.orientation, q);
  }
  for (size_t i = 0; i <= count; ++i) {
    geometry_msgs::msg::PoseStamped point = start;
    point.header = goal.header;
    const double t = static_cast<double>(i) / static_cast<double>(count);
    point.pose.position.x += dx * t;
    point.pose.position.y += dy * t;
    point.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(std::move(point));
  }
  path.poses.back().pose.orientation = goal.pose.orientation;
  return path;
}
nav_msgs::msg::Path StraightFirstPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  std::function<bool()> cancel_checker) {
  if (start.header.frame_id != goal.header.frame_id ||
      start.header.frame_id != costmap_ros_->getGlobalFrameID()) {
    throw std::runtime_error("StraightFirst: start/goal must share global frame");
  }
  // read costmap safely while it is asynchronously updated.
  bool free = false;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_ros_->getCostmap()->getMutex());
    free = isLineSafe(start, goal, cancel_checker);
  }
  if (free) {return straightPath(start, goal);}
  RCLCPP_DEBUG(node_->get_logger(), "%s: direct line blocked; using Smac2D A*", name_.c_str());
  return astar_->createPlan(start, goal, cancel_checker);
}
}
PLUGINLIB_EXPORT_CLASS(nav2_straight_first_planner::StraightFirstPlanner, nav2_core::GlobalPlanner)
