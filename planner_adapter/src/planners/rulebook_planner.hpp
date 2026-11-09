#pragma once

#include "planner_adapter/graph_planner.hpp"

namespace planner_adapter {

// Runs the planning library's RulebookPlanner (sequential optimal-subgraph reduction, one rule
// class at a time in priority order) on the adapter's graph and rule costs.
// Needs a rulebook_file whose rules are totally ordered.
class RulebookGraphPlanner : public GraphPlanner {
 public:
  PlanResult plan(const PlanRequest& request) override;
};

}  // namespace planner_adapter
