#include "planner_adapter/planner_registry.hpp"

#include "planners/dijkstra_planner.hpp"
#include "planners/rulebook_planner.hpp"

namespace planner_adapter {

const std::map<std::string, PlannerFactory>& plannerRegistry() {
  static const std::map<std::string, PlannerFactory> registry = {
      {"dijkstra", [] { return std::make_unique<DijkstraPlanner>(); }},
      {"rulebook", [] { return std::make_unique<RulebookGraphPlanner>(); }},
  };
  return registry;
}

std::unique_ptr<GraphPlanner> createPlanner(const std::string& name) {
  const auto& registry = plannerRegistry();
  const auto it = registry.find(name);
  return it == registry.end() ? nullptr : it->second();
}

std::vector<std::string> plannerNames() {
  std::vector<std::string> names;
  for (const auto& [name, factory] : plannerRegistry()) names.push_back(name);
  return names;
}

}  // namespace planner_adapter
