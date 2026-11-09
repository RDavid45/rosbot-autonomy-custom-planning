#include "planner_adapter/rulebook_config.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_set>

#include <yaml-cpp/yaml.h>

#include "Rule.h"
#include "RulebookCost.h"

namespace planner_adapter {

namespace {

std::string baseName(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

std::map<std::string, std::size_t> ruleIds(const RulebookSpec& spec) {
  std::map<std::string, std::size_t> ids;
  for (std::size_t i = 0; i < spec.rules.size(); i++) ids[spec.rules[i].name] = i;
  return ids;
}

void checkKeys(
    const YAML::Node& map,
    std::initializer_list<const char*> allowed,
    const std::string& where,
    const std::string& path
  ) {
  for (const auto& kv : map) {
    const std::string key = kv.first.as<std::string>();
    if (std::none_of(allowed.begin(), allowed.end(), [&key](const char* a) { return key == a; })) {
      std::string list;
      for (const char* a : allowed) list += std::string(list.empty() ? "" : ", ") + a;
      throw std::runtime_error(path + ": " + where + "unknown key '" + key + "' (expected " + list + ")");
    }
  }
}

std::string describe(const LoadedRulebook& lr) {
  const Rulebook& rb = lr.rulebook;
  const auto& rules = lr.spec.rules;
  std::string out = baseName(lr.spec.source) + ": ";
  if (rb.isTotallyOrdered()) {
    bool first = true;
    for (auto it = rb.begin(); it != rb.end(); ++it) {
      for (std::size_t id : *it) {
        out += (first ? "" : " > ") + rules[id].name;
        first = false;
      }
    }
    return out;
  }
  // Partial order: list the relations as written, plus any equivalence classes.
  bool first = true;
  for (const auto& [hi, lo] : lr.spec.priority) {
    out += (first ? "" : ", ") + hi + " > " + lo;
    first = false;
  }
  for (const auto& cls : lr.spec.equivalent) {
    out += first ? "{" : ", {";
    for (std::size_t k = 0; k < cls.size(); k++) out += (k ? " = " : "") + cls[k];
    out += "}";
    first = false;
  }
  return out;
}

}  // namespace

RulebookSpec parseRulebookYaml(const std::string& path) {
  auto fail = [&path](const std::string& msg) -> void { throw std::runtime_error(path + ": " + msg); };

  YAML::Node loaded;
  try {
    loaded = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error(path + ": " + e.what());
  }
  const YAML::Node root = loaded;
  if (!root.IsMap()) fail("top level must be a map with a 'rules' list");
  checkKeys(root, {"rules", "equivalent", "priority"}, "", path);

  RulebookSpec spec;
  spec.source = path;

  const YAML::Node rules = root["rules"];
  if (!rules || !rules.IsSequence() || rules.size() == 0) fail("'rules' must be a non-empty list");
  std::set<std::string> names;
  for (std::size_t i = 0; i < rules.size(); i++) {
    const YAML::Node r = rules[i];
    std::string where = "rules[" + std::to_string(i) + "]";
    if (!r.IsMap()) fail(where + " must be a map");
    checkKeys(r, {"name", "aggregate", "metric", "params"}, where + ": ", path);
    auto text = [&](const char* key) {
      const YAML::Node n = r[key];
      if (!n || !n.IsScalar() || n.as<std::string>().empty()) fail(where + ": missing '" + key + "'");
      return n.as<std::string>();
    };

    RuleSpec rule;
    rule.name = text("name");
    where += " (" + rule.name + ")";
    if (!names.insert(rule.name).second) fail(where + ": duplicate rule name");

    const std::string aggregate = text("aggregate");
    if (aggregate == "sum") rule.aggregate = Aggregate::Sum;
    else if (aggregate == "max") rule.aggregate = Aggregate::Max;
    else fail(where + ": aggregate must be 'sum' or 'max', got '" + aggregate + "'");

    rule.metric = text("metric");

    if (const YAML::Node params = r["params"]) {
      if (!params.IsMap()) fail(where + ": params must be a map");
      for (const auto& kv : params) {
        const std::string key = kv.first.as<std::string>();
        try {
          rule.params[key] = kv.second.as<double>();
        } catch (const YAML::Exception&) {
          fail(where + ": params." + key + " must be a number");
        }
      }
    }
    spec.rules.push_back(rule);
  }

  auto ruleName = [&](const YAML::Node& n, const std::string& where) {
    if (!n.IsScalar()) fail(where + ": expected a rule name");
    const std::string name = n.as<std::string>();
    if (!names.count(name)) fail(where + ": unknown rule '" + name + "'");
    return name;
  };

  if (const YAML::Node eq = root["equivalent"]) {
    if (!eq.IsSequence()) fail("'equivalent' must be a list of lists of rule names");
    for (std::size_t i = 0; i < eq.size(); i++) {
      const std::string where = "equivalent[" + std::to_string(i) + "]";
      if (!eq[i].IsSequence() || eq[i].size() < 2) fail(where + " must list at least two rule names");
      std::vector<std::string> cls;
      for (const auto& n : eq[i]) cls.push_back(ruleName(n, where));
      spec.equivalent.push_back(cls);
    }
  }

  if (const YAML::Node pr = root["priority"]) {
    if (!pr.IsSequence()) fail("'priority' must be a list of [higher, lower] pairs");
    for (std::size_t i = 0; i < pr.size(); i++) {
      const std::string where = "priority[" + std::to_string(i) + "]";
      if (!pr[i].IsSequence() || pr[i].size() != 2) fail(where + " must be a [higher, lower] pair");
      const std::string hi = ruleName(pr[i][0], where);
      const std::string lo = ruleName(pr[i][1], where);
      if (hi == lo) fail(where + ": a rule can't outrank itself");
      spec.priority.emplace_back(hi, lo);
    }
  }
  return spec;
}

void populateRulebook(
    Rulebook& rulebook,
    const RulebookSpec& spec
  ) {
  for (const auto& rule : spec.rules) {
    if (rule.aggregate == Aggregate::Sum) rulebook.addRule(RuleSum(rule.name));
    else rulebook.addRule(RuleMax(rule.name));
  }
  const auto ids = ruleIds(spec);

  if (!spec.equivalent.empty()) {
    std::vector<std::unordered_set<std::size_t>> classes;
    for (const auto& cls : spec.equivalent) {
      std::unordered_set<std::size_t> set;
      for (const auto& name : cls) set.insert(ids.at(name));
      classes.push_back(set);
    }
    try {
      rulebook.setEquivalentClasses(classes);
    } catch (const std::exception& e) {
      throw std::runtime_error(spec.source + ": equivalent: " + e.what());
    }
  }

  for (const auto& [hi, lo] : spec.priority) {
    try {
      rulebook.addGTRelation(ids.at(hi), ids.at(lo));
    } catch (const std::exception& e) {
      throw std::runtime_error(spec.source + ": priority [" + hi + ", " + lo + "]: " + e.what());
    }
  }
}

bool LoadedRulebook::needsClearance() const {
  return std::any_of(metrics.begin(), metrics.end(), [](const Metric& m) { return m.needs_clearance; });
}

std::shared_ptr<const LoadedRulebook> loadRulebook(const std::string& path) {
  auto lr = std::make_shared<LoadedRulebook>();
  lr->spec = parseRulebookYaml(path);
  populateRulebook(lr->rulebook, lr->spec);

  bool consistent = false;
  try {
    consistent = lr->rulebook.build();
  } catch (const std::exception& e) {
    throw std::runtime_error(path + ": priority relations form a cycle (" + e.what() + ")");
  }
  if (!consistent) throw std::runtime_error(path + ": priority relations are inconsistent");

  for (std::size_t i = 0; i < lr->spec.rules.size(); i++) {
    const RuleSpec& rule = lr->spec.rules[i];
    try {
      lr->metrics.push_back(createMetric(rule.metric, rule.params));
    } catch (const std::exception& e) {
      throw std::runtime_error(path + ": rules[" + std::to_string(i) + "] (" + rule.name + "): " + e.what());
    }
  }

  lr->description = describe(*lr);
  RulebookCost::setRulebook(lr->rulebook);
  return lr;
}

}  // namespace planner_adapter
