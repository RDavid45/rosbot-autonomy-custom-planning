#pragma once

#include <map>
#include <string>
#include <vector>

#include "grid_graph.hpp"

namespace planner_adapter {

// Where a planner reads its own settings. In Nav2 these come from the YAML under
// <plugin name>.<planner name>.*; in the offline tool they come from the command line.
class ParameterSource {
 public:
  virtual ~ParameterSource() = default;
  virtual double getDouble(const std::string& name, double default_value) = 0;
  virtual int getInt(const std::string& name, int default_value) = 0;
  virtual bool getBool(const std::string& name, bool default_value) = 0;
  virtual std::string getString(const std::string& name, const std::string& default_value) = 0;
};

struct PlanResult {
  // Node ids from start to goal, both included. Empty means the planner found no path.
  std::vector<int> nodes;
  // Why planning failed, or a note worth logging on success.
  std::string message;
  // Planner-specific numbers (expansions, iterations, cost vector, ...). Logged next to the
  // common metrics, so planners can report extras without changing the CSV columns.
  std::map<std::string, double> stats;
};

struct LoadedRulebook;  // rulebook_config.hpp

struct PlanRequest {
  // Rebuilt for every request. When a rulebook is configured, Edge::costs[i] already holds rule i's
  // value for that edge (filled once by the adapter, so every planner ranks identical costs).
  grid_graph::Graph& graph;
  int start;  // valid node ids, different from each other
  int goal;
  const LoadedRulebook* rulebook;  // null when no rulebook_file is configured
};

// What every planner under test implements. The adapter does everything else the same way for
// all of them: map conversion, graph building, rule costs, snapping start/goal to nodes, timing,
// path validation, and metrics. A planner only sees the request.
class GraphPlanner {
 public:
  virtual ~GraphPlanner() = default;

  // Called once, before any plan() call.
  virtual void configure(ParameterSource& params) { (void)params; }

  virtual PlanResult plan(const PlanRequest& request) = 0;
};

}  // namespace planner_adapter
