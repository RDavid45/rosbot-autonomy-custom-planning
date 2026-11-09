#include "planner_adapter/rule_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>

namespace planner_adapter {

namespace {

// Splits the straight segment a→b into equal pieces at most half a map cell long and calls
// visit(cell index, piece length) for each, with cell index -1 for pieces outside the map.
// The piece lengths add up to the segment length.
template <typename Visit>
void walkSegment(
    const grid_graph::OccupancyMap& map,
    const grid_graph::Node& a,
    const grid_graph::Node& b,
    Visit visit
  ) {
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (0.5 * map.resolution))));
  const double piece = length / steps;
  for (int k = 0; k < steps; k++) {
    const double t = (k + 0.5) / steps;
    const int col = static_cast<int>(std::floor((a.x + t * (b.x - a.x) - map.origin_x) / map.resolution));
    const int row = static_cast<int>(std::floor((a.y + t * (b.y - a.y) - map.origin_y) / map.resolution));
    const bool inside = col >= 0 && col < map.width && row >= 0 && row < map.height;
    visit(inside ? row * map.width + col : -1, piece);
  }
}

void checkParams(
    const std::string& metric,
    const MetricParams& params,
    std::initializer_list<const char*> allowed
  ) {
  for (const auto& [key, value] : params) {
    (void)value;
    if (std::none_of(allowed.begin(), allowed.end(), [&key](const char* a) { return key == a; }))
      throw std::invalid_argument("metric '" + metric + "' has no parameter '" + key + "'");
  }
}

double positiveParam(
    const std::string& metric,
    const MetricParams& params,
    const std::string& key
  ) {
  const auto it = params.find(key);
  if (it == params.end()) throw std::invalid_argument("metric '" + metric + "' needs params." + key);
  if (!(it->second > 0)) throw std::invalid_argument("metric '" + metric + "': " + key + " must be > 0");
  return it->second;
}

}  // namespace

const std::map<std::string, MetricFactory>& metricRegistry() {
  static const std::map<std::string, MetricFactory> registry = {
      // Edge length (m).
      {"length",
       [](const MetricParams& p) {
         checkParams("length", p, {});
         Metric m;
         m.evaluate = [](const MetricContext&, const grid_graph::Node&, const grid_graph::Node&,
                          const grid_graph::Edge& e) { return e.distance; };
         return m;
       }},

      // Length (m) of the edge that runs closer than params.clearance to an obstacle. Suits a sum rule.
      {"clearance_violation",
       [](const MetricParams& p) {
         checkParams("clearance_violation", p, {"clearance"});
         const double clearance = positiveParam("clearance_violation", p, "clearance");
         Metric m;
         m.needs_clearance = true;
         m.evaluate = [clearance](const MetricContext& ctx, const grid_graph::Node& a, const grid_graph::Node& b,
                                  const grid_graph::Edge&) {
           double total = 0.0;
           walkSegment(ctx.map, a, b, [&](int cell, double piece) {
             if (cell >= 0 && (*ctx.clearance)[cell] < clearance) total += piece;
           });
           return total;
         };
         return m;
       }},

      // How far (m) the edge's closest approach to an obstacle falls short of params.clearance;
      // 0 if it never gets that close. Suits a max rule ("worst intrusion along the path").
      {"clearance_deficit",
       [](const MetricParams& p) {
         checkParams("clearance_deficit", p, {"clearance"});
         const double clearance = positiveParam("clearance_deficit", p, "clearance");
         Metric m;
         m.needs_clearance = true;
         m.evaluate = [clearance](const MetricContext& ctx, const grid_graph::Node& a, const grid_graph::Node& b,
                                  const grid_graph::Edge&) {
           double closest = std::numeric_limits<double>::infinity();
           walkSegment(ctx.map, a, b, [&](int cell, double) {
             if (cell >= 0) closest = std::min(closest, (*ctx.clearance)[cell]);
           });
           return std::max(0.0, clearance - closest);
         };
         return m;
       }},

      // Length (m) of the edge through unknown cells. Always 0 unless unknown_is_blocked is false,
      // because otherwise the graph never crosses unknown space.
      {"unknown_overlap",
       [](const MetricParams& p) {
         checkParams("unknown_overlap", p, {});
         Metric m;
         m.evaluate = [](const MetricContext& ctx, const grid_graph::Node& a, const grid_graph::Node& b,
                          const grid_graph::Edge&) {
           double total = 0.0;
           walkSegment(ctx.map, a, b, [&](int cell, double piece) {
             if (cell < 0 || ctx.map.data[cell] < 0) total += piece;
           });
           return total;
         };
         return m;
       }},
  };
  return registry;
}

Metric createMetric(
    const std::string& name,
    const MetricParams& params
  ) {
  const auto& registry = metricRegistry();
  const auto it = registry.find(name);
  if (it == registry.end()) {
    std::string known;
    for (const auto& n : metricNames()) known += " " + n;
    throw std::invalid_argument("unknown metric '" + name + "' (known:" + known + ")");
  }
  return it->second(params);
}

std::vector<std::string> metricNames() {
  std::vector<std::string> names;
  for (const auto& [name, factory] : metricRegistry()) names.push_back(name);
  return names;
}

}  // namespace planner_adapter
