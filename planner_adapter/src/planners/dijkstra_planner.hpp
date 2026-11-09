#pragma once

#include "planner_adapter/graph_planner.hpp"

namespace planner_adapter {

class DijkstraPlanner : public GraphPlanner {
 public:
  PlanResult plan(const PlanRequest& request) override;
};

}  // namespace planner_adapter
