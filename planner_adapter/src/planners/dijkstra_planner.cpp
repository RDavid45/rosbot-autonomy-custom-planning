// Baseline: shortest path by Euclidean edge length. Use it as the reference the other planners
// are compared against, and as a template for adding a new planner.

#include "planners/dijkstra_planner.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace planner_adapter {

PlanResult DijkstraPlanner::plan(const PlanRequest& request) {
  const grid_graph::Graph& graph = request.graph;
  const int start = request.start;
  const int goal = request.goal;
  const std::size_t n = graph.nodes.size();
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> parent(n, -1);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  dist[start] = 0.0;
  open.push({0.0, start});
  int expansions = 0;
  while (!open.empty()) {
    const auto [d, u] = open.top();
    open.pop();
    if (d > dist[u]) continue;  // stale entry
    expansions++;
    if (u == goal) break;
    for (const auto& e : graph.adjacency[u]) {
      const double nd = d + e.distance;
      if (nd < dist[e.to]) {
        dist[e.to] = nd;
        parent[e.to] = u;
        open.push({nd, e.to});
      }
    }
  }

  PlanResult result;
  result.stats["expansions"] = expansions;
  if (parent[goal] < 0) {
    result.message = "goal unreachable from start";
    return result;
  }
  for (int v = goal; v != -1; v = parent[v]) result.nodes.push_back(v);
  std::reverse(result.nodes.begin(), result.nodes.end());
  return result;
}

}  // namespace planner_adapter
