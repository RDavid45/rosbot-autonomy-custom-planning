// Runs planners on a saved map without ROS, through the same pipeline the Nav2 plugin uses.
//
//   compare_planners --map maps/map.yaml --start 0 0 --goal 3.5 1.2
//                    [--planners dijkstra,rulebook] [--csv results.csv] [--label office]
//                    [--set grid_spacing=0.2] [--set rulebook_file=rulebooks/office.yaml] ...
//
// --set takes the same parameter names as the Nav2 YAML. Without --planners, every registered
// planner runs.

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "planner_adapter/pipeline.hpp"
#include "planner_adapter/planner_registry.hpp"

using namespace planner_adapter;

namespace {

class MapParameterSource : public ParameterSource {
 public:
  std::map<std::string, std::string> values;

  double getDouble(const std::string& n, double d) override {
    const auto it = values.find(n);
    return it == values.end() ? d : std::stod(it->second);
  }
  int getInt(const std::string& n, int d) override {
    const auto it = values.find(n);
    return it == values.end() ? d : std::stoi(it->second);
  }
  bool getBool(const std::string& n, bool d) override {
    const auto it = values.find(n);
    if (it == values.end()) return d;
    return it->second == "true" || it->second == "1" || it->second == "True";
  }
  std::string getString(const std::string& n, const std::string& d) override {
    const auto it = values.find(n);
    return it == values.end() ? d : it->second;
  }
};

// Reads the handful of keys map_saver writes. Not a general YAML parser.
grid_graph::OccupancyMap loadMapYaml(const std::string& yaml_path) {
  std::ifstream in(yaml_path);
  if (!in) throw std::runtime_error("cannot open " + yaml_path);
  std::string image;
  double resolution = 0.05, origin_x = 0, origin_y = 0, occupied = 0.65, free_thresh = 0.196;
  std::string line;
  while (std::getline(in, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = line.substr(0, colon);
    std::string value = line.substr(colon + 1);
    key.erase(0, key.find_first_not_of(" \t"));
    value.erase(0, value.find_first_not_of(" \t"));
    if (key == "image") image = value;
    else if (key == "resolution") resolution = std::stod(value);
    else if (key == "occupied_thresh") occupied = std::stod(value);
    else if (key == "free_thresh") free_thresh = std::stod(value);
    else if (key == "origin") {
      for (char& c : value)
        if (c == '[' || c == ']' || c == ',') c = ' ';
      std::istringstream(value) >> origin_x >> origin_y;
    }
  }
  if (image.empty()) throw std::runtime_error(yaml_path + " has no image: entry");
  if (image.front() != '/') {
    const auto slash = yaml_path.find_last_of("/\\");
    if (slash != std::string::npos) image = yaml_path.substr(0, slash + 1) + image;
  }
  return grid_graph::loadPgm(image, resolution, origin_x, origin_y, occupied, free_thresh);
}

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> parts;
  std::stringstream ss(s);
  std::string part;
  while (std::getline(ss, part, sep))
    if (!part.empty()) parts.push_back(part);
  return parts;
}

int usage() {
  std::cerr << "usage: compare_planners --map MAP.yaml --start X Y --goal X Y\n"
               "         [--planners a,b] [--csv FILE] [--label TEXT] [--set key=value]...\n"
               "planners:";
  for (const auto& name : plannerNames()) std::cerr << ' ' << name;
  std::cerr << '\n';
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  std::string map_path, csv_path, label;
  std::vector<std::string> planners;
  double sx = 0, sy = 0, gx = 0, gy = 0;
  bool have_start = false, have_goal = false;
  MapParameterSource params;

  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    auto need = [&](int count) {
      if (i + count >= argc) {
        std::cerr << arg << " needs " << count << " value(s)\n";
        std::exit(usage());
      }
    };
    if (arg == "--map") { need(1); map_path = argv[++i]; }
    else if (arg == "--start") { need(2); sx = std::stod(argv[++i]); sy = std::stod(argv[++i]); have_start = true; }
    else if (arg == "--goal") { need(2); gx = std::stod(argv[++i]); gy = std::stod(argv[++i]); have_goal = true; }
    else if (arg == "--planners") { need(1); planners = split(argv[++i], ','); }
    else if (arg == "--csv") { need(1); csv_path = argv[++i]; }
    else if (arg == "--label") { need(1); label = argv[++i]; }
    else if (arg == "--set") {
      need(1);
      const std::string kv = argv[++i];
      const auto eq = kv.find('=');
      if (eq == std::string::npos) return usage();
      params.values[kv.substr(0, eq)] = kv.substr(eq + 1);
    } else {
      std::cerr << "unknown argument " << arg << '\n';
      return usage();
    }
  }
  if (map_path.empty() || !have_start || !have_goal) return usage();
  if (planners.empty()) planners = plannerNames();

  grid_graph::OccupancyMap map;
  try {
    map = loadMapYaml(map_path);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  PipelineParams pipeline;
  try {
    pipeline = readPipelineParams(params);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  if (pipeline.rulebook) std::cout << "rulebook " << pipeline.rulebook->description << '\n';

  int failures = 0;
  for (const auto& name : planners) {
    auto planner = createPlanner(name);
    if (!planner) {
      std::cerr << "unknown planner '" << name << "'\n";
      return usage();
    }
    PrefixedParameterSource own(params, name);
    planner->configure(own);

    PlanOutcome out = runPlanner(*planner, name, map, pipeline, sx, sy, gx, gy);
    out.record.label = label;
    const PlanRecord& r = out.record;
    std::cout << name << ": " << (r.success ? "ok" : "FAILED") << "  nodes=" << r.graph_nodes
              << " edges=" << r.graph_edges << " build=" << r.build_ms << "ms plan=" << r.plan_ms
              << "ms path=" << r.path_nodes << " nodes / " << r.path_length_m << "m";
    for (const auto& [rule, value] : r.rule_costs) std::cout << "  " << rule << '=' << value;
    if (!r.message.empty()) std::cout << "  (" << r.message << ")";
    std::cout << '\n';
    if (!r.success) failures++;
    if (!csv_path.empty() && !appendCsv(csv_path, r)) {
      std::cerr << "cannot write " << csv_path << '\n';
      return 1;
    }
  }
  return failures == 0 ? 0 : 1;
}
