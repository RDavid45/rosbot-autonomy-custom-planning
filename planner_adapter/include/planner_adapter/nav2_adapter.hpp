#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "planner_adapter/graph_planner.hpp"
#include "planner_adapter/pipeline.hpp"

namespace planner_adapter {

// Nav2 global planner that runs any registered GraphPlanner through the shared pipeline.
// Which planner runs is the "planner" parameter; list several instances of this plugin in
// planner_plugins (one per planner) to switch between them by planner_id.
class GraphPlannerAdapter : public nav2_core::GlobalPlanner {
 public:
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal) override;

 private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("GraphPlannerAdapter")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string planner_name_;
  std::unique_ptr<GraphPlanner> planner_;
  PipelineParams pipeline_;
  std::string results_csv_;
  std::string label_;
};

}  // namespace planner_adapter
