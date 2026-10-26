#pragma once
#include <functional>
#include <memory>
#include <string>
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_straight_first_planner {
class StraightFirstPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                 const geometry_msgs::msg::PoseStamped &goal,
                                 std::function<bool()> cancel_checker) override;
private:
  bool isLineSafe(const geometry_msgs::msg::PoseStamped &start,
                  const geometry_msgs::msg::PoseStamped &goal,
                  const std::function<bool()> &cancel_checker) const;
  bool isFootprintSafe(double x, double y) const;
  nav_msgs::msg::Path straightPath(const geometry_msgs::msg::PoseStamped &start,
                                   const geometry_msgs::msg::PoseStamped &goal) const;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader_;
  nav2_core::GlobalPlanner::Ptr astar_;
  std::string name_;
  double interpolation_resolution_{0.05};
  double safety_radius_{0.12};
  int max_cell_cost_{252};
  bool allow_unknown_{false};
};
}
