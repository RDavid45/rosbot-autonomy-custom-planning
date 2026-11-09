#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "grid_graph.hpp"

namespace planner_adapter {

// What a metric can look at when scoring one edge.
struct MetricContext {
  const grid_graph::OccupancyMap& map;
  // Distance (m) from each map cell to the nearest blocked cell. Only computed when some metric in
  // the rulebook sets needs_clearance; null otherwise.
  const std::vector<double>* clearance;
};

// Computes one rule's value for one edge. Rulebook YAML refers to metrics by name; YAML can choose,
// order and tune rules, but a new kind of cost means adding a metric here in C++.
struct Metric {
  bool needs_clearance = false;
  std::function<double(
      const MetricContext& ctx,
      const grid_graph::Node& from,
      const grid_graph::Node& to,
      const grid_graph::Edge& edge)>
      evaluate;
};

using MetricParams = std::map<std::string, double>;
// Validates the params (throws std::invalid_argument on unknown/missing/bad ones) and returns the metric.
using MetricFactory = std::function<Metric(const MetricParams& params)>;

// To add a metric: add an entry to metricRegistry() in rule_metrics.cpp.
const std::map<std::string, MetricFactory>& metricRegistry();

// Throws std::invalid_argument if the name is unknown or the params are wrong.
Metric createMetric(
    const std::string& name,
    const MetricParams& params
  );

std::vector<std::string> metricNames();

}  // namespace planner_adapter
