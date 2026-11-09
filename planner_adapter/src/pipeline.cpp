#include "planner_adapter/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace planner_adapter {

namespace {

constexpr unsigned char kLethal = 254;
constexpr unsigned char kNoInformation = 255;

double millisecondsSince(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// Snaps a world position to the nearest node, or returns -1 if none is within max_distance.
int snap(
    const grid_graph::Graph& graph,
    double x,
    double y,
    double max_distance
  ) {
  const int id = graph.nearestNode(x, y);
  if (id < 0) return -1;
  const auto& node = graph.nodes[id];
  return std::hypot(node.x - x, node.y - y) <= max_distance ? id : -1;
}

// The edge from a to b, or null if there is none.
const grid_graph::Edge* findEdge(
    const grid_graph::Graph& graph,
    int a,
    int b
  ) {
  for (const auto& e : graph.adjacency[a])
    if (e.to == b) return &e;
  return nullptr;
}

// Every consecutive pair of nodes must be joined by an edge of the graph.
bool pathUsesRealEdges(
    const grid_graph::Graph& graph,
    const std::vector<int>& nodes
  ) {
  const int n = static_cast<int>(graph.nodes.size());
  for (std::size_t k = 0; k < nodes.size(); k++)
    if (nodes[k] < 0 || nodes[k] >= n) return false;
  for (std::size_t k = 0; k + 1 < nodes.size(); k++)
    if (!findEdge(graph, nodes[k], nodes[k + 1])) return false;
  return true;
}

// Writes every rule's value into every edge's costs (both directions of each edge).
void fillRuleCosts(
    grid_graph::Graph& graph,
    const grid_graph::OccupancyMap& map,
    const grid_graph::GraphParams& graph_params,
    const LoadedRulebook& rulebook
  ) {
  std::vector<double> clearance;
  if (rulebook.needsClearance()) clearance = grid_graph::clearanceMap(map, graph_params);
  const MetricContext ctx{map, rulebook.needsClearance() ? &clearance : nullptr};
  const std::size_t rules = rulebook.metrics.size();
  for (std::size_t a = 0; a < graph.adjacency.size(); a++) {
    for (auto& edge : graph.adjacency[a]) {
      edge.costs.resize(rules);
      for (std::size_t i = 0; i < rules; i++)
        edge.costs[i] = rulebook.metrics[i].evaluate(ctx, graph.nodes[a], graph.nodes[edge.to], edge);
    }
  }
}

// CSV-quotes a field if it contains a separator, quote, or newline.
std::string csvField(const std::string& s) {
  if (s.find_first_of(",\"\n") == std::string::npos) return s;
  std::string out = "\"";
  for (char c : s) {
    if (c == '"') out += '"';
    out += c;
  }
  return out + "\"";
}

}  // namespace

grid_graph::OccupancyMap fromCostmap(
    const unsigned char* costs,
    int width,
    int height,
    double resolution,
    double origin_x,
    double origin_y
  ) {
  grid_graph::OccupancyMap map;
  map.width = width;
  map.height = height;
  map.resolution = resolution;
  map.origin_x = origin_x;
  map.origin_y = origin_y;
  map.data.resize(static_cast<std::size_t>(width) * height);
  for (std::size_t i = 0; i < map.data.size(); i++) {
    if (costs[i] == kNoInformation) map.data[i] = -1;
    else if (costs[i] == kLethal) map.data[i] = 100;
    else map.data[i] = 0;
  }
  return map;
}

PipelineParams readPipelineParams(ParameterSource& p) {
  PipelineParams out;
  grid_graph::GraphParams& g = out.graph;
  g.grid_spacing = p.getDouble("grid_spacing", g.grid_spacing);
  g.robot_radius = p.getDouble("robot_radius", g.robot_radius);
  g.safety_margin = p.getDouble("safety_margin", g.safety_margin);
  g.occupied_threshold = p.getInt("occupied_threshold", g.occupied_threshold);
  g.unknown_is_blocked = p.getBool("unknown_is_blocked", g.unknown_is_blocked);
  g.eight_connected = p.getBool("eight_connected", g.eight_connected);
  g.keep_largest_component = p.getBool("keep_largest_component", g.keep_largest_component);
  g.num_costs = p.getInt("num_costs", g.num_costs);
  out.max_snap_distance = p.getDouble("max_snap_distance", g.grid_spacing);
  const std::string rulebook_file = p.getString("rulebook_file", "");
  if (!rulebook_file.empty()) {
    out.rulebook = loadRulebook(rulebook_file);
    g.num_costs = static_cast<int>(out.rulebook->spec.rules.size());
  }
  return out;
}

PlanOutcome runPlanner(
    GraphPlanner& planner,
    const std::string& planner_name,
    const grid_graph::OccupancyMap& map,
    const PipelineParams& params,
    double start_x,
    double start_y,
    double goal_x,
    double goal_y
  ) {
  PlanOutcome outcome;
  PlanRecord& rec = outcome.record;
  rec.planner = planner_name;
  if (params.rulebook) rec.rulebook = params.rulebook->description;
  rec.start_x = start_x;
  rec.start_y = start_y;
  rec.goal_x = goal_x;
  rec.goal_y = goal_y;

  grid_graph::Graph graph;
  const auto build_start = std::chrono::steady_clock::now();
  try {
    graph = grid_graph::buildGraph(map, params.graph);
  } catch (const std::exception& e) {
    rec.message = std::string("graph build failed: ") + e.what();
    return outcome;
  }
  rec.build_ms = millisecondsSince(build_start);
  rec.graph_nodes = graph.nodes.size();
  rec.graph_edges = graph.edgeCount();

  if (params.rulebook) {
    const auto cost_start = std::chrono::steady_clock::now();
    fillRuleCosts(graph, map, params.graph, *params.rulebook);
    rec.cost_ms = millisecondsSince(cost_start);
  }

  const int start = snap(graph, start_x, start_y, params.max_snap_distance);
  const int goal = snap(graph, goal_x, goal_y, params.max_snap_distance);
  if (start < 0) {
    rec.message = "start is not within max_snap_distance of the graph";
    return outcome;
  }
  if (goal < 0) {
    rec.message = "goal is not within max_snap_distance of the graph";
    return outcome;
  }

  PlanResult result;
  if (start == goal) {
    result.nodes = {start};  // already there; no need to bother the planner
  } else {
    const auto plan_start = std::chrono::steady_clock::now();
    try {
      result = planner.plan(PlanRequest{graph, start, goal, params.rulebook.get()});
    } catch (const std::exception& e) {
      result = {};
      result.message = std::string("planner threw: ") + e.what();
    }
    rec.plan_ms = millisecondsSince(plan_start);
  }
  rec.stats = result.stats;
  rec.message = result.message;

  if (result.nodes.empty()) {
    if (rec.message.empty()) rec.message = "no path found";
    return outcome;
  }
  if (result.nodes.front() != start || result.nodes.back() != goal) {
    rec.message = "planner path does not run from start to goal";
    return outcome;
  }
  if (!pathUsesRealEdges(graph, result.nodes)) {
    rec.message = "planner path uses a pair of nodes with no edge between them";
    return outcome;
  }

  rec.success = true;
  rec.path_nodes = result.nodes.size();
  for (std::size_t k = 0; k < result.nodes.size(); k++) {
    const auto& node = graph.nodes[result.nodes[k]];
    outcome.waypoints.emplace_back(node.x, node.y);
    if (k > 0) {
      const auto& prev = graph.nodes[result.nodes[k - 1]];
      rec.path_length_m += std::hypot(node.x - prev.x, node.y - prev.y);
    }
  }
  if (params.rulebook) {
    const auto& rules = params.rulebook->spec.rules;
    for (std::size_t i = 0; i < rules.size(); i++) {
      double total = 0.0;
      for (std::size_t k = 0; k + 1 < result.nodes.size(); k++) {
        const double v = findEdge(graph, result.nodes[k], result.nodes[k + 1])->costs[i];
        total = rules[i].aggregate == Aggregate::Sum ? total + v : std::max(total, v);
      }
      rec.rule_costs.emplace_back(rules[i].name, total);
    }
  }
  return outcome;
}

bool appendCsv(
    const std::string& path,
    const PlanRecord& record
  ) {
  bool needs_header;
  {
    std::ifstream existing(path, std::ios::ate);
    needs_header = !existing || existing.tellg() <= 0;
  }
  std::ofstream out(path, std::ios::app);
  if (!out) return false;
  if (needs_header) {
    out << "planner,label,rulebook,start_x,start_y,goal_x,goal_y,graph_nodes,graph_edges,"
           "build_ms,cost_ms,plan_ms,success,path_nodes,path_length_m,rule_costs,message,stats\n";
  }
  std::ostringstream rule_costs;
  for (const auto& [name, value] : record.rule_costs) {
    if (rule_costs.tellp() > 0) rule_costs << ';';
    rule_costs << name << '=' << value;
  }
  std::ostringstream stats;
  for (const auto& [key, value] : record.stats) {
    if (stats.tellp() > 0) stats << ';';
    stats << key << '=' << value;
  }
  out << std::fixed << std::setprecision(3)
      << csvField(record.planner) << ',' << csvField(record.label) << ',' << csvField(record.rulebook) << ','
      << record.start_x << ',' << record.start_y << ',' << record.goal_x << ',' << record.goal_y << ','
      << record.graph_nodes << ',' << record.graph_edges << ','
      << record.build_ms << ',' << record.cost_ms << ',' << record.plan_ms << ','
      << (record.success ? 1 : 0) << ',' << record.path_nodes << ',' << record.path_length_m << ','
      << csvField(rule_costs.str()) << ','
      << csvField(record.message) << ',' << csvField(stats.str()) << '\n';
  return true;
}

}  // namespace planner_adapter
