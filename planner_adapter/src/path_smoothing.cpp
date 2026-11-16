// The Bezier control-point solve is adapted from:
// Source - https://stackoverflow.com/a/23258882
// Posted by FuzzyBunnySlippers
// Retrieved 2026-11-16, License - CC BY-SA 3.0
// which is based on http://www.particleincell.com/blog/2012/bezier-splines/

#include "planner_adapter/path_smoothing.hpp"

#include <algorithm>
#include <cmath>

namespace planner_adapter {

namespace {

// For one coordinate of the points p[0..N], the first and second control points of each of the N
// cubic segments. The tridiagonal system is the same for x and y, so it is solved per coordinate.
void controlPoints(
    const std::vector<double>& p,
    std::vector<double>& c1,
    std::vector<double>& c2
  ) {
  const int n = static_cast<int>(p.size()) - 1;
  c1.assign(n, 0.0);
  c2.assign(n, 0.0);
  if (n == 1) {
    c1[0] = (2.0 * p[0] + p[1]) / 3.0;
    c2[0] = 2.0 * c1[0] - p[0];
    return;
  }

  std::vector<double> a(n), b(n), c(n), r(n);
  a[0] = 0;
  b[0] = 2;
  c[0] = 1;
  r[0] = p[0] + 2 * p[1];
  for (int i = 1; i < n - 1; i++) {
    a[i] = 1;
    b[i] = 4;
    c[i] = 1;
    r[i] = 4 * p[i] + 2 * p[i + 1];
  }
  a[n - 1] = 2;
  b[n - 1] = 7;
  c[n - 1] = 0;
  r[n - 1] = 8 * p[n - 1] + p[n];

  // Thomas algorithm.
  for (int i = 1; i < n; i++) {
    const double m = a[i] / b[i - 1];
    b[i] -= m * c[i - 1];
    r[i] -= m * r[i - 1];
  }
  c1[n - 1] = r[n - 1] / b[n - 1];
  for (int i = n - 2; i >= 0; i--) c1[i] = (r[i] - c[i] * c1[i + 1]) / b[i];

  for (int i = 0; i < n - 1; i++) c2[i] = 2 * p[i + 1] - c1[i + 1];
  c2[n - 1] = 0.5 * (p[n] + c1[n - 1]);
}

double bezier(
    double p0,
    double p1,
    double p2,
    double p3,
    double t
  ) {
  const double u = 1.0 - t;
  return u * u * u * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 + t * t * t * p3;
}

}  // namespace

SmoothedPath smoothPath(
    const grid_graph::OccupancyMap& map,
    const std::vector<double>& clearance,
    double required,
    const Waypoints& points,
    double step
  ) {
  if (points.size() < 3) return {points, 0};

  std::vector<double> xs, ys;
  for (const auto& [x, y] : points) {
    xs.push_back(x);
    ys.push_back(y);
  }
  std::vector<double> x1, x2, y1, y2;
  controlPoints(xs, x1, x2);
  controlPoints(ys, y1, y2);

  SmoothedPath out{{points.front()}, 0};
  for (std::size_t s = 0; s + 1 < points.size(); s++) {
    Waypoints segment{points[s]};
    // The control polygon is never shorter than the curve, so this takes at least curve length / step
    // samples. Even steps in t are not even steps along the curve, so neighbouring samples can be up
    // to 3 * step apart when one leg of the control polygon dominates.
    const double polygon = std::hypot(x1[s] - xs[s], y1[s] - ys[s]) +
                           std::hypot(x2[s] - x1[s], y2[s] - y1[s]) +
                           std::hypot(xs[s + 1] - x2[s], ys[s + 1] - y2[s]);
    const int samples = std::max(1, static_cast<int>(std::ceil(polygon / step)));
    for (int k = 1; k <= samples; k++) {
      const double t = static_cast<double>(k) / samples;
      segment.emplace_back(bezier(xs[s], x1[s], x2[s], xs[s + 1], t), bezier(ys[s], y1[s], y2[s], ys[s + 1], t));
    }

    if (!pathClear(map, clearance, required, segment)) {
      out.straight_segments++;
      const double length = std::hypot(xs[s + 1] - xs[s], ys[s + 1] - ys[s]);
      const int pieces = std::max(1, static_cast<int>(std::ceil(length / step)));
      segment.resize(1);
      for (int k = 1; k <= pieces; k++) {
        const double t = static_cast<double>(k) / pieces;
        segment.emplace_back(xs[s] + t * (xs[s + 1] - xs[s]), ys[s] + t * (ys[s + 1] - ys[s]));
      }
      segment.back() = points[s + 1];
    }
    out.waypoints.insert(out.waypoints.end(), segment.begin() + 1, segment.end());
  }
  return out;
}

bool pathClear(
    const grid_graph::OccupancyMap& map,
    const std::vector<double>& clearance,
    double required,
    const Waypoints& path
  ) {
  auto clear = [&](double x, double y) {
    const int col = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
    const int row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
    if (col < 0 || col >= map.width || row < 0 || row >= map.height) return false;
    return clearance[static_cast<std::size_t>(row) * map.width + col] > required;
  };

  if (path.empty()) return true;
  if (!clear(path.front().first, path.front().second)) return false;
  for (std::size_t k = 0; k + 1 < path.size(); k++) {
    const auto [ax, ay] = path[k];
    const auto [bx, by] = path[k + 1];
    const double length = std::hypot(bx - ax, by - ay);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / (0.5 * map.resolution))));
    for (int i = 1; i <= steps; i++) {
      const double t = static_cast<double>(i) / steps;
      if (!clear(ax + t * (bx - ax), ay + t * (by - ay))) return false;
    }
  }
  return true;
}

}  // namespace planner_adapter
