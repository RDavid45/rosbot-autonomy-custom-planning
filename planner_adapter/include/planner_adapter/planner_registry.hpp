#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "planner_adapter/graph_planner.hpp"

namespace planner_adapter {

using PlannerFactory = std::function<std::unique_ptr<GraphPlanner>()>;

// Every planner the adapter can run, by the name used in the YAML / on the command line.
// To add one: implement GraphPlanner, add a line to plannerRegistry() in planner_registry.cpp,
// and add the source file to CMakeLists.txt.
const std::map<std::string, PlannerFactory>& plannerRegistry();

// nullptr if no planner has that name.
std::unique_ptr<GraphPlanner> createPlanner(const std::string& name);

std::vector<std::string> plannerNames();

}  // namespace planner_adapter
