#include "grid_graph.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace grid_graph {

namespace {

constexpr long long kUnreached = std::numeric_limits<long long>::max();  // no ripple has reached this cell

bool isBlocked(
    int8_t value,
    const GraphParams& params
  ) {
  if (value < 0) return params.unknown_is_blocked;
  return value >= params.occupied_threshold;
}

// True if every cell sampled along the segment has enough clearance for the robot.
bool segmentSafe(
    const std::vector<char>& safe,
    int width,
    int r0,
    int c0,
    int r1,
    int c1
  ) {
  const int n = std::max(std::abs(r1 - r0), std::abs(c1 - c0)) + 1;
  for (int k = 0; k < n; k++) {
    const double t = n == 1 ? 0.0 : double(k) / (n - 1);
    const int r = static_cast<int>(std::lround(r0 + t * (r1 - r0)));
    const int c = static_cast<int>(std::lround(c0 + t * (c1 - c0)));
    if (!safe[r * width + c]) return false;
  }
  return true;
}

}  // namespace

std::size_t Graph::edgeCount() const {
  std::size_t total = 0;
  for (const auto& edges : adjacency) total += edges.size();
  return total / 2;  // undirected: every edge is stored in both directions
}

int Graph::nearestNode(
    double x,
    double y
  ) const {
  int best = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (const auto& node : nodes) {
    const double d2 = (node.x - x) * (node.x - x) + (node.y - y) * (node.y - y);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = node.id;
    }
  }
  return best;
}

OccupancyMap loadPgm(
    const std::string& path,
    double resolution,
    double origin_x,
    double origin_y,
    double occupied_thresh,
    double free_thresh
  ) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open " + path);

  // Header: magic, width, height, maxval, each separated by whitespace, '#' starts a comment.
  auto nextToken = [&in]() {
    std::string token;
    while (in >> token) {
      if (token[0] != '#') return token;
      std::string rest;
      std::getline(in, rest);
    }
    throw std::runtime_error("truncated PGM header");
  };
  if (nextToken() != "P5") throw std::runtime_error(path + " is not a binary PGM (P5)");
  const int width = std::stoi(nextToken());
  const int height = std::stoi(nextToken());
  const int maxval = std::stoi(nextToken());
  if (maxval > 255) throw std::runtime_error("16-bit PGM not supported");
  in.get();  // single whitespace byte before the pixel data

  std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height);
  if (!in.read(reinterpret_cast<char*>(pixels.data()), pixels.size()))
    throw std::runtime_error("truncated PGM data");

  OccupancyMap map;
  map.width = width;
  map.height = height;
  map.resolution = resolution;
  map.origin_x = origin_x;
  map.origin_y = origin_y;
  map.data.resize(pixels.size());
  for (int row = 0; row < height; row++) {
    // Image row 0 is the top of the map, OccupancyGrid row 0 is the bottom.
    const int img_row = height - 1 - row;
    for (int col = 0; col < width; col++) {
      const double p = (maxval - pixels[img_row * width + col]) / double(maxval);
      int8_t value = -1;
      if (p > occupied_thresh) value = 100;
      else if (p < free_thresh) value = 0;
      map.data[row * width + col] = value;
    }
  }
  return map;
}

std::vector<double> clearanceMap(
    const OccupancyMap& map,
    const GraphParams& params
  ) {
  const int w = map.width;
  const int h = map.height;
  const std::size_t n = static_cast<std::size_t>(w) * h;

  // For every cell: the nearest wall found so far, and the squared distance to it in cells².
  // Integers keep the distances exact, so comparisons never suffer from rounding.
  std::vector<int> nearest_wall(n, -1);
  std::vector<long long> best_d2(n, kUnreached);

  // Min-heap of (squared distance, cell): the closest cell is always processed next,
  // so the ripple spreads outward evenly from every wall at once.
  using Entry = std::pair<long long, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;

  // Every wall starts its own ripple at distance 0, as its own nearest wall.
  for (std::size_t i = 0; i < n; i++) {
    if (!isBlocked(map.data[i], params)) continue;
    nearest_wall[i] = static_cast<int>(i);
    best_d2[i] = 0;
    queue.push({0, static_cast<int>(i)});
  }

  while (!queue.empty()) {
    const auto [d2, cell] = queue.top();
    queue.pop();
    if (d2 > best_d2[cell]) continue;  // outdated entry: a closer wall reached this cell since it was queued

    const int wall = nearest_wall[cell];
    const int wall_col = wall % w, wall_row = wall / w;
    const int col = cell % w, row = cell / w;

    // Pass this cell's wall on to its 8 neighbours, measuring the true straight-line
    // distance from each neighbour back to that wall (not a step count).
    for (int dr = -1; dr <= 1; dr++) {
      for (int dc = -1; dc <= 1; dc++) {
        const int nr = row + dr, nc = col + dc;
        if ((dr == 0 && dc == 0) || nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
        const long long dx = nc - wall_col, dy = nr - wall_row;
        const long long nd2 = dx * dx + dy * dy;
        const int neighbour = nr * w + nc;
        if (nd2 >= best_d2[neighbour]) continue;  // it already knows a wall at least as close
        best_d2[neighbour] = nd2;
        nearest_wall[neighbour] = wall;
        queue.push({nd2, neighbour});
      }
    }
  }

  std::vector<double> clearance(n);
  for (std::size_t i = 0; i < n; i++) {
    clearance[i] = best_d2[i] == kUnreached ? std::numeric_limits<double>::infinity()
                                            : std::sqrt(static_cast<double>(best_d2[i])) * map.resolution;
  }
  return clearance;
}

Graph buildGraph(
    const OccupancyMap& map,
    const GraphParams& params
  ) {
  if (map.width <= 0 || map.height <= 0 || map.data.size() != std::size_t(map.width) * map.height)
    throw std::invalid_argument("map size does not match its data");
  if (map.resolution <= 0.0) throw std::invalid_argument("map resolution must be positive");
  if (params.grid_spacing < map.resolution)
    throw std::invalid_argument("grid_spacing must be at least the map resolution");
  if (params.num_costs < 0) throw std::invalid_argument("num_costs must not be negative");

  const std::vector<double> clearance = clearanceMap(map, params);
  const double required = params.robot_radius + params.safety_margin;
  std::vector<char> safe(clearance.size());
  for (std::size_t i = 0; i < clearance.size(); i++) safe[i] = clearance[i] > required;

  // Lattice points are anchored to the world, not to the map corner: world grid index k sits at
  // k * grid_spacing. first_row/first_col are the first indices at or past the map's origin.
  // Lattice point (i, j) is world index (first_row + i, first_col + j) and sits in map cell
  // (cell_rows[i], cell_cols[j]).
  const double spacing = params.grid_spacing;
  const int first_row = static_cast<int>(std::ceil(map.origin_y / spacing));
  const int first_col = static_cast<int>(std::ceil(map.origin_x / spacing));
  // The small epsilon stops floating-point error from rounding a point on a cell edge into the cell below.
  auto worldToCell = [&map](double world, double origin) {
    return static_cast<int>(std::floor((world - origin) / map.resolution + 1e-9));
  };
  std::vector<int> cell_rows, cell_cols;
  for (int k = first_row; worldToCell(k * spacing, map.origin_y) < map.height; k++)
    cell_rows.push_back(worldToCell(k * spacing, map.origin_y));
  for (int k = first_col; worldToCell(k * spacing, map.origin_x) < map.width; k++)
    cell_cols.push_back(worldToCell(k * spacing, map.origin_x));
  const int rows = static_cast<int>(cell_rows.size());
  const int cols = static_cast<int>(cell_cols.size());
  auto latticeIndex = [cols](int i, int j) { return i * cols + j; };

  std::vector<char> candidate(static_cast<std::size_t>(rows) * cols, 0);
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++) candidate[latticeIndex(i, j)] = safe[cell_rows[i] * map.width + cell_cols[j]];

  // Lattice-level edges; each undirected edge is generated once by looking "forward".
  struct LatticeEdge {
    int a, b;
    double distance;
  };
  std::vector<std::pair<int, int>> offsets = {{0, 1}, {1, 0}};
  if (params.eight_connected) {
    offsets.push_back({1, 1});
    offsets.push_back({1, -1});
  }
  std::vector<LatticeEdge> lattice_edges;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      if (!candidate[latticeIndex(i, j)]) continue;
      for (const auto& [di, dj] : offsets) {
        const int ni = i + di, nj = j + dj;
        if (ni < 0 || ni >= rows || nj < 0 || nj >= cols || !candidate[latticeIndex(ni, nj)]) continue;
        if (!segmentSafe(safe, map.width, cell_rows[i], cell_cols[j], cell_rows[ni], cell_cols[nj])) continue;
        lattice_edges.push_back({latticeIndex(i, j), latticeIndex(ni, nj), params.grid_spacing * std::hypot(di, dj)});
      }
    }
  }

  // Optionally keep only the largest connected component.
  std::vector<char> keep = candidate;
  if (params.keep_largest_component) {
    std::vector<std::vector<int>> neighbours(candidate.size());
    for (const auto& e : lattice_edges) {
      neighbours[e.a].push_back(e.b);
      neighbours[e.b].push_back(e.a);
    }
    std::vector<int> component(candidate.size(), -1);
    std::vector<int> sizes;
    for (std::size_t start = 0; start < candidate.size(); start++) {
      if (!candidate[start] || component[start] != -1) continue;
      const int label = static_cast<int>(sizes.size());
      sizes.push_back(0);
      std::queue<int> frontier;
      frontier.push(static_cast<int>(start));
      component[start] = label;
      while (!frontier.empty()) {
        const int cur = frontier.front();
        frontier.pop();
        sizes[label]++;
        for (int nb : neighbours[cur]) {
          if (component[nb] != -1) continue;
          component[nb] = label;
          frontier.push(nb);
        }
      }
    }
    const int largest = sizes.empty() ? -1 : int(std::max_element(sizes.begin(), sizes.end()) - sizes.begin());
    for (std::size_t k = 0; k < keep.size(); k++) keep[k] = candidate[k] && component[k] == largest;
  }

  Graph graph;
  graph.lattice_rows = rows;
  graph.lattice_cols = cols;
  std::vector<int> node_id(candidate.size(), -1);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      if (!keep[latticeIndex(i, j)]) continue;
      const int id = static_cast<int>(graph.nodes.size());
      node_id[latticeIndex(i, j)] = id;
      const int world_row = first_row + i, world_col = first_col + j;
      graph.nodes.push_back({id, world_row, world_col, world_col * spacing, world_row * spacing});
    }
  }
  graph.adjacency.resize(graph.nodes.size());
  for (const auto& e : lattice_edges) {
    const int a = node_id[e.a], b = node_id[e.b];
    if (a < 0 || b < 0) continue;
    const std::vector<double> costs(params.num_costs, 0.0);
    graph.adjacency[a].push_back({b, e.distance, costs});
    graph.adjacency[b].push_back({a, e.distance, costs});
  }
  return graph;
}

}  // namespace grid_graph
