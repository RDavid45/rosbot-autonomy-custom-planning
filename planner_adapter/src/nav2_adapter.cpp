#include "planner_adapter/nav2_adapter.hpp"

#include <cmath>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "planner_adapter/planner_registry.hpp"

namespace planner_adapter {

namespace {

// Reads "<prefix><name>" from the node's parameters, declaring it with the default first.
// Declared with dynamic typing so that e.g. "grid_spacing: 1" in YAML still works as a double.
class NodeParameterSource : public ParameterSource {
 public:
  NodeParameterSource(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string prefix)
      : node_(std::move(node)), prefix_(std::move(prefix) + ".") {}

  double getDouble(const std::string& n, double d) override {
    const auto v = get(n, rclcpp::ParameterValue(d));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return static_cast<double>(v.get<int64_t>());
    return v.get<double>();
  }
  int getInt(const std::string& n, int d) override {
    return static_cast<int>(get(n, rclcpp::ParameterValue(d)).get<int64_t>());
  }
  bool getBool(const std::string& n, bool d) override { return get(n, rclcpp::ParameterValue(d)).get<bool>(); }
  std::string getString(const std::string& n, const std::string& d) override {
    return get(n, rclcpp::ParameterValue(d)).get<std::string>();
  }

 private:
  rclcpp::ParameterValue get(const std::string& n, const rclcpp::ParameterValue& d) {
    const std::string full = prefix_ + n;
    if (!node_->has_parameter(full)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      node_->declare_parameter(full, d, descriptor);
    }
    return node_->get_parameter(full).get_parameter_value();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string prefix_;
};

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

void GraphPlannerAdapter::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  auto node = parent.lock();
  if (!node) throw std::runtime_error("GraphPlannerAdapter: parent node is gone");
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();

  NodeParameterSource params(node, name_);
  planner_name_ = params.getString("planner", "dijkstra");
  results_csv_ = params.getString("results_csv", "");
  label_ = params.getString("label", "");
  try {
    pipeline_ = readPipelineParams(params);
  } catch (const std::exception& e) {
    // planner_server only reports pluginlib errors itself, so say what went wrong before failing.
    RCLCPP_FATAL(logger_, "%s: %s", name_.c_str(), e.what());
    throw;
  }
  if (pipeline_.rulebook) {
    RCLCPP_INFO(logger_, "%s: rulebook %s", name_.c_str(), pipeline_.rulebook->description.c_str());
  }

  planner_ = createPlanner(planner_name_);
  if (!planner_) {
    std::ostringstream known;
    for (const auto& n : plannerNames()) known << ' ' << n;
    throw std::runtime_error(
        "GraphPlannerAdapter: unknown planner '" + planner_name_ + "'; registered:" + known.str());
  }
  PrefixedParameterSource own(params, planner_name_);
  planner_->configure(own);

  RCLCPP_INFO(logger_,
      "%s: planner=%s grid_spacing=%.2f robot_radius=%.2f safety_margin=%.2f results_csv=%s",
      name_.c_str(), planner_name_.c_str(), pipeline_.graph.grid_spacing, pipeline_.graph.robot_radius,
      pipeline_.graph.safety_margin, results_csv_.empty() ? "(off)" : results_csv_.c_str());
}

void GraphPlannerAdapter::cleanup() { planner_.reset(); }
void GraphPlannerAdapter::activate() {}
void GraphPlannerAdapter::deactivate() {}

nav_msgs::msg::Path GraphPlannerAdapter::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(logger_, "%s: start/goal must be in %s (got %s / %s)", name_.c_str(), global_frame_.c_str(),
        start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return path;
  }

  // Copy the costmap under its lock so the costmap thread can keep updating while we plan.
  grid_graph::OccupancyMap map;
  {
    nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
    map = fromCostmap(costmap->getCharMap(), static_cast<int>(costmap->getSizeInCellsX()),
        static_cast<int>(costmap->getSizeInCellsY()), costmap->getResolution(), costmap->getOriginX(),
        costmap->getOriginY());
  }

  PlanOutcome out = runPlanner(*planner_, planner_name_, map, pipeline_, start.pose.position.x,
      start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
  out.record.label = label_;
  const PlanRecord& r = out.record;

  if (!results_csv_.empty() && !appendCsv(results_csv_, r))
    RCLCPP_WARN(logger_, "%s: cannot write %s", name_.c_str(), results_csv_.c_str());

  if (!r.success) {
    RCLCPP_WARN(logger_, "%s/%s: no path (%s)", name_.c_str(), planner_name_.c_str(), r.message.c_str());
    return path;  // an empty path tells planner_server that planning failed
  }
  RCLCPP_INFO(logger_, "%s/%s: %zu nodes, %.2f m, build %.1f ms, plan %.1f ms (graph %zu nodes / %zu edges)",
      name_.c_str(), planner_name_.c_str(), r.path_nodes, r.path_length_m, r.build_ms, r.plan_ms, r.graph_nodes,
      r.graph_edges);

  const auto& wp = out.waypoints;
  for (std::size_t k = 0; k < wp.size(); k++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wp[k].first;
    pose.pose.position.y = wp[k].second;
    if (k + 1 < wp.size()) {
      pose.pose.orientation =
          yawToQuaternion(std::atan2(wp[k + 1].second - wp[k].second, wp[k + 1].first - wp[k].first));
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace planner_adapter

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner_adapter::GraphPlannerAdapter, nav2_core::GlobalPlanner)
