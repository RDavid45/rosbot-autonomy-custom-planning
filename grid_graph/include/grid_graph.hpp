#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace grid_graph {

// Same layout as nav_msgs/OccupancyGrid: row-major, row 0 is the bottom of the map
// (closest to origin), values are -1 = unknown, 0..100 = occupancy probability.
struct OccupancyMap {
  // map size
  int width = 0;
  int height = 0;

  // meters per cell
  double resolution = 0.05;

  // world position of the bottom-left corner of cell (0, 0)
  double origin_x = 0.0;
  double origin_y = 0.0;

  // flattened 2D vector of the map
  std::vector<int8_t> data;

  int8_t at(int col, int row) const { return data[row * width + col]; }
};

struct GraphParams {
  double grid_spacing = 0.25;       // m between neighbouring nodes; must be >= map resolution
  double robot_radius = 0.16;       // m
  double safety_margin = 0.05;      // m of extra clearance beyond robot_radius
  int occupied_threshold = 50;      // cells with value >= this are obstacles
  bool unknown_is_blocked = true;   // never route through unexplored space
  bool eight_connected = true;      // add diagonal edges
  bool keep_largest_component = true;  // drop islands the robot can't reach
  int num_costs = 0;                // size of Edge::costs, e.g. the number of rules in the rulebook
};

struct Node {
  int id;
  // World grid index: the node sits at (lattice_col * grid_spacing, lattice_row * grid_spacing).
  // Stays the same across rebuilds of a growing map, unlike id. Can be negative.
  int lattice_row;
  int lattice_col;
  double x;  // world coordinates (m)
  double y;
};

struct Edge {
  int to;
  double distance;  // Euclidean length (m), filled in by buildGraph
  // One entry per rule, indexed by a fixed rule id (not by priority, which the planner can reorder).
  // buildGraph sizes it to GraphParams::num_costs and leaves every entry at 0 for the planners to fill.
  std::vector<double> costs;
};

struct Graph {
  std::vector<Node> nodes;
  std::vector<std::vector<Edge>> adjacency;  // adjacency[node id]
  int lattice_rows = 0;  // number of lattice points that fit on the map, blocked ones included
  int lattice_cols = 0;

  std::size_t edgeCount() const;
  // Closest node to a world position, or -1 if the graph is empty.
  int nearestNode(
      double x,
      double y
    ) const;
};

// Loads a map_saver .pgm (trinary mode, negate = 0) using the resolution/origin from its .yaml.
OccupancyMap loadPgm(
    const std::string& path,
    double resolution,
    double origin_x,
    double origin_y,
    double occupied_thresh = 0.65,
    double free_thresh = 0.196
  );

// Distance (m) from every cell to the nearest blocked cell; 0 for blocked cells, infinity if
// the map has none. Computed by rippling outward from every blocked cell at once.
std::vector<double> clearanceMap(
    const OccupancyMap& map,
    const GraphParams& params
  );

// Overlays a lattice on the map; a lattice point becomes a node if the robot fits there,
// and two neighbouring nodes are connected if the straight line between them is clear.
// Lattice points sit at whole multiples of grid_spacing in world coordinates, so they stay put
// when SLAM grows the map or moves its origin. Throws std::invalid_argument on a bad map/params.
Graph buildGraph(
    const OccupancyMap& map,
    const GraphParams& params = {}
  );

}  // namespace grid_graph
