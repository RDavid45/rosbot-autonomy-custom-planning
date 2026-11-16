#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "grid_graph.hpp"

namespace planner_adapter {

using Waypoints = std::vector<std::pair<double, double>>;  // world (x, y)

// True if every point along the polyline (sampled every half map cell) is on the map and has more
// than `required` m of clearance: the same test buildGraph uses to place nodes.
bool pathClear(
    const grid_graph::OccupancyMap& map,
    const std::vector<double>& clearance,
    double required,
    const Waypoints& path
  );

struct SmoothedPath {
  Waypoints waypoints;
  // Segments whose curve failed pathClear and were replaced by the straight line between their nodes.
  std::size_t straight_segments = 0;
};

// Cubic Bezier spline through every point, with continuous first and second derivatives where
// segments meet, sampled about `step` m apart (at most 3 * step) and passing exactly through every
// point. Each segment (between two consecutive points) is checked with pathClear on its own; a
// segment that fails is replaced by the straight line between its two points, which is safe when
// the points are joined by a graph edge. Fewer than 3 points are returned unchanged.
SmoothedPath smoothPath(
    const grid_graph::OccupancyMap& map,
    const std::vector<double>& clearance,
    double required,
    const Waypoints& points,
    double step
  );

}  // namespace planner_adapter
