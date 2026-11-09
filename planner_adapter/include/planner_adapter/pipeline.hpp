#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "grid_graph.hpp"
#include "planner_adapter/graph_planner.hpp"
#include "planner_adapter/rulebook_config.hpp"

namespace planner_adapter {

// Converts raw Nav2 costmap values (row-major, row 0 at the origin, like Costmap2D::getCharMap())
// into an OccupancyMap. Only LETHAL_OBSTACLE (254) becomes an obstacle and NO_INFORMATION (255)
// becomes unknown; every other value, including Nav2's own inflation, becomes free, because
// grid_graph applies robot_radius + safety_margin itself.
grid_graph::OccupancyMap fromCostmap(
    const unsigned char* costs,
    int width,
    int height,
    double resolution,
    double origin_x,
    double origin_y
  );

struct PipelineParams {
  grid_graph::GraphParams graph;
  // Start/goal farther than this (m) from every node fail instead of snapping.
  double max_snap_distance = 0.25;
  // Loaded from rulebook_file; null if none. When set, graph.num_costs is its rule count and the
  // pipeline fills Edge::costs before any planner runs.
  std::shared_ptr<const LoadedRulebook> rulebook;
};

// Reads the shared settings (grid_spacing, robot_radius, ..., max_snap_distance, rulebook_file)
// with the same names and defaults in Nav2 and in the offline tool. Loads the rulebook if one is
// given; throws std::runtime_error if it is invalid.
PipelineParams readPipelineParams(ParameterSource& params);

// Gives a planner its own namespace: getDouble("x") reads "<prefix>.x".
class PrefixedParameterSource : public ParameterSource {
 public:
  PrefixedParameterSource(ParameterSource& inner, std::string prefix)
      : inner_(inner), prefix_(std::move(prefix) + ".") {}
  double getDouble(const std::string& n, double d) override { return inner_.getDouble(prefix_ + n, d); }
  int getInt(const std::string& n, int d) override { return inner_.getInt(prefix_ + n, d); }
  bool getBool(const std::string& n, bool d) override { return inner_.getBool(prefix_ + n, d); }
  std::string getString(const std::string& n, const std::string& d) override {
    return inner_.getString(prefix_ + n, d);
  }

 private:
  ParameterSource& inner_;
  std::string prefix_;
};

// One row of results; identical columns for every planner and for both the Nav2 plugin and
// the offline tool, so runs can be compared directly.
struct PlanRecord {
  std::string planner;
  std::string label;     // free text identifying the map/scenario
  std::string rulebook;  // LoadedRulebook::description, empty without a rulebook
  double start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
  std::size_t graph_nodes = 0;
  std::size_t graph_edges = 0;
  double build_ms = 0;
  double cost_ms = 0;  // filling Edge::costs from the rulebook
  double plan_ms = 0;
  bool success = false;
  std::size_t path_nodes = 0;
  double path_length_m = 0;
  // The path's value for each rule (rule name, aggregated with the rule's sum/max), in rule id order.
  std::vector<std::pair<std::string, double>> rule_costs;
  std::string message;
  std::map<std::string, double> stats;
};

struct PlanOutcome {
  PlanRecord record;
  std::vector<std::pair<double, double>> waypoints;  // world (x, y) of each path node
};

// The one code path every planner goes through: build the graph, fill rule costs, snap start and
// goal to nodes, run the planner, check that its path only uses real edges, and measure it.
PlanOutcome runPlanner(
    GraphPlanner& planner,
    const std::string& planner_name,
    const grid_graph::OccupancyMap& map,
    const PipelineParams& params,
    double start_x,
    double start_y,
    double goal_x,
    double goal_y
  );

// Appends one row to a CSV file, writing the header first if the file is new or empty.
// Returns false if the file could not be opened.
bool appendCsv(
    const std::string& path,
    const PlanRecord& record
  );

}  // namespace planner_adapter
