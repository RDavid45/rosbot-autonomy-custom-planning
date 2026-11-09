#include "planners/rulebook_planner.hpp"

#include <exception>

#include "RulebookPlanner.h"  // from the planning library
#include "planner_adapter/rulebook_config.hpp"

namespace planner_adapter {

namespace {

// Presents the adapter's graph to the planning library as a ScenarioDiscrete, so its
// RulebookPlanner can run unchanged. A state is a grid_graph node id.
class GridGraphScenario : public ScenarioDiscrete<int> {
 public:
  GridGraphScenario(
      const grid_graph::Graph& graph,
      const RulebookSpec& spec,
      int start,
      int goal)
      : ScenarioDiscrete<int>(start, goal), source_(graph), spec_(spec) {
    setup();  // builds the rulebook (and RulebookCost's copy of it), then the weighted graph
  }

  int nodeOf(std::size_t vid) const { return vid2state.at(vid); }

 protected:
  void defineRulebook() override { populateRulebook(rulebook, spec_); }

  void buildGraph() override {
    for (const auto& node : source_.nodes) addState(node.id);
    for (std::size_t from = 0; from < source_.adjacency.size(); from++) {
      for (const auto& edge : source_.adjacency[from]) {
        RulebookCost cost;
        for (std::size_t i = 0; i < edge.costs.size(); i++) cost.setRuleCost(i, edge.costs[i]);
        graph.addEdge(state2vid.at(static_cast<int>(from)), state2vid.at(edge.to), cost);
      }
    }
  }

 private:
  const grid_graph::Graph& source_;
  const RulebookSpec& spec_;
};

}  // namespace

PlanResult RulebookGraphPlanner::plan(const PlanRequest& request) {
  PlanResult result;
  if (!request.rulebook) {
    result.message = "the rulebook planner needs a rulebook_file";
    return result;
  }
  // RulebookPlanner::getOptimalPlan only assert()s this, and asserts are compiled out in Release,
  // where a partial order would silently be treated as some arbitrary total order.
  if (!request.rulebook->rulebook.isTotallyOrdered()) {
    result.message = "the rulebook planner needs totally ordered rules (no equivalent classes, "
                     "and priority must chain every rule)";
    return result;
  }

  GridGraphScenario scenario(request.graph, request.rulebook->spec, request.start, request.goal);
  RulebookPlanner<int> planner(scenario);
  const auto edges = planner.getOptimalPlan(scenario.getInit(), scenario.getGoal());
  if (edges.empty()) {
    result.message = "goal unreachable from start";
    return result;
  }
  result.nodes.push_back(request.start);
  for (const auto& edge : edges) result.nodes.push_back(scenario.nodeOf(edge->to->vid));
  return result;
}

}  // namespace planner_adapter
