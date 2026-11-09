#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Rulebook.h"  // from the planning library
#include "planner_adapter/rule_metrics.hpp"

namespace planner_adapter {

enum class Aggregate { Sum, Max };  // RuleSum / RuleMax

struct RuleSpec {
  std::string name;
  Aggregate aggregate = Aggregate::Sum;
  std::string metric;
  MetricParams params;
};

// A rulebook as written in YAML:
//
//   rules:                      # rule id = position in this list (also the Edge::costs index)
//     - { name: keep_clearance, aggregate: max, metric: clearance_deficit, params: { clearance: 0.3 } }
//     - { name: min_length,     aggregate: sum, metric: length }
//   equivalent:                 # optional: rules of equal rank; unlisted rules are their own class
//     - [rule_a, rule_b]
//   priority:                   # optional: [higher, lower] pairs (Rulebook::addGTRelation)
//     - [keep_clearance, min_length]
struct RulebookSpec {
  std::string source;  // file it came from
  std::vector<RuleSpec> rules;
  std::vector<std::vector<std::string>> equivalent;
  std::vector<std::pair<std::string, std::string>> priority;
};

// Parses and checks the file: known keys only, unique rule names, aggregate is sum/max,
// numeric params, and every name in equivalent/priority refers to a rule.
// Throws std::runtime_error("<path>: <what is wrong>").
RulebookSpec parseRulebookYaml(const std::string& path);

// Adds the spec's rules, equivalence classes and priorities to an empty Rulebook (does not build it).
void populateRulebook(
    Rulebook& rulebook,
    const RulebookSpec& spec
  );

struct LoadedRulebook {
  RulebookSpec spec;
  Rulebook rulebook;            // built
  std::vector<Metric> metrics;  // metrics[i] computes rule i
  std::string description;      // e.g. "office.yaml: keep_clearance > min_length", for logs and CSV

  bool needsClearance() const;
};

// Parse + build + create metrics, then RulebookCost::setRulebook(). RulebookCost keeps the rulebook
// in a static variable, so a process holds one rulebook at a time: loading another replaces it.
std::shared_ptr<const LoadedRulebook> loadRulebook(const std::string& path);

}  // namespace planner_adapter
