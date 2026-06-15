// SPDX-License-Identifier: Apache-2.0
#include "amr_planning/astar.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace amr_planning {

namespace {

const double kSqrt2 = std::sqrt(2.0);

// 8-connected neighbour offsets with their step lengths (cell units).
struct Neighbor {
  int dr;
  int dc;
  double step;
};
const Neighbor kNeighbors[8] = {
    {-1, 0, 1.0},      {1, 0, 1.0},      {0, -1, 1.0},     {0, 1, 1.0},
    {-1, -1, kSqrt2},  {-1, 1, kSqrt2},  {1, -1, kSqrt2},  {1, 1, kSqrt2},
};

double octile(int r0, int c0, int r1, int c1) {
  const int dr = std::abs(r1 - r0);
  const int dc = std::abs(c1 - c0);
  const int lo = std::min(dr, dc);
  const int hi = std::max(dr, dc);
  return static_cast<double>(hi - lo) + kSqrt2 * static_cast<double>(lo);
}

// Pack a (row, col) cell into a single key for hashing/sets.
inline long long key(int r, int c) {
  return (static_cast<long long>(r) << 32) ^ (static_cast<unsigned int>(c));
}

// BFS outward from (row, col) for the nearest non-lethal cell within max_dist_m
// metres. Returns the cell, or sets found=false.
std::pair<int, int> nearest_non_lethal(const Costmap& costmap, int row, int col,
                                       double max_dist_m, bool& found) {
  found = false;
  if (costmap.in_bounds(row, col) && !costmap.is_lethal(row, col)) {
    found = true;
    return {row, col};
  }
  const int max_cells =
      static_cast<int>(std::floor(max_dist_m / costmap.resolution()));
  std::set<long long> seen;
  seen.insert(key(row, col));
  std::deque<std::tuple<int, int, int>> q;
  q.emplace_back(row, col, 0);
  while (!q.empty()) {
    auto [r, c, d] = q.front();
    q.pop_front();
    if (d >= max_cells) {
      continue;
    }
    for (const auto& nb : kNeighbors) {
      const int nr = r + nb.dr;
      const int nc = c + nb.dc;
      const long long k = key(nr, nc);
      if (seen.count(k)) {
        continue;
      }
      seen.insert(k);
      if (!costmap.in_bounds(nr, nc)) {
        continue;
      }
      if (!costmap.is_lethal(nr, nc)) {
        found = true;
        return {nr, nc};
      }
      q.emplace_back(nr, nc, d + 1);
    }
  }
  return {row, col};
}

// Greedy shortcutting: drop a point when there is line of sight from the
// previous kept point to the point after it.
std::vector<std::array<double, 2>> simplify(
    const Costmap& costmap, const std::vector<std::array<double, 2>>& pts) {
  if (pts.size() <= 2) {
    return pts;
  }
  std::vector<std::array<double, 2>> out;
  out.push_back(pts[0]);
  std::size_t i = 0;
  while (i + 1 < pts.size()) {
    std::size_t j = i + 1;
    while (j + 1 < pts.size() &&
           has_line_of_sight(costmap, out.back()[0], out.back()[1],
                             pts[j + 1][0], pts[j + 1][1])) {
      ++j;
    }
    out.push_back(pts[j]);
    i = j;
  }
  return out;
}

}  // namespace

bool has_line_of_sight(const Costmap& costmap, double px, double py, double qx,
                       double qy) {
  const double dist = std::hypot(qx - px, qy - py);
  const double step = costmap.resolution() * 0.5;
  const int n = dist > 0.0 ? static_cast<int>(std::ceil(dist / step)) : 0;
  for (int i = 0; i <= n; ++i) {
    const double t = n > 0 ? static_cast<double>(i) / n : 0.0;
    const double x = px + t * (qx - px);
    const double y = py + t * (qy - py);
    if (costmap.cost_at_world(x, y) >= 0.99) {
      return false;
    }
  }
  return true;
}

std::optional<std::vector<std::array<double, 2>>> plan_path(
    const Costmap& costmap, const std::array<double, 2>& start_xy,
    const std::array<double, 2>& goal_xy, const amr_core::AstarConfig& cfg) {
  const double start_x = start_xy[0];
  const double start_y = start_xy[1];
  const double goal_x = goal_xy[0];
  const double goal_y = goal_xy[1];

  int sr, sc, gr, gc;
  costmap.world_to_grid(start_x, start_y, sr, sc);
  costmap.world_to_grid(goal_x, goal_y, gr, gc);

  if (!costmap.in_bounds(gr, gc) || costmap.is_lethal(gr, gc)) {
    return std::nullopt;
  }

  // If the start cell is lethal, search outward (<= 0.3 m) for a free cell.
  bool start_found = false;
  auto start_cell = nearest_non_lethal(costmap, sr, sc, 0.3, start_found);
  if (!start_found) {
    return std::nullopt;
  }
  sr = start_cell.first;
  sc = start_cell.second;

  if (sr == gr && sc == gc) {
    return std::vector<std::array<double, 2>>{{start_x, start_y},
                                              {goal_x, goal_y}};
  }

  const double w_cost = cfg.w_cost;

  // Open priority queue: (f, g, r, c). Min-heap on f.
  struct Node {
    double f;
    double g;
    int r;
    int c;
  };
  struct NodeCmp {
    bool operator()(const Node& a, const Node& b) const { return a.f > b.f; }
  };
  std::priority_queue<Node, std::vector<Node>, NodeCmp> open_heap;
  std::unordered_map<long long, double> g_score;
  std::unordered_map<long long, long long> came_from;
  std::set<long long> closed;

  open_heap.push({octile(sr, sc, gr, gc), 0.0, sr, sc});
  g_score[key(sr, sc)] = 0.0;

  bool found = false;
  while (!open_heap.empty()) {
    const Node cur = open_heap.top();
    open_heap.pop();
    const long long ck = key(cur.r, cur.c);
    if (closed.count(ck)) {
      continue;
    }
    if (cur.r == gr && cur.c == gc) {
      found = true;
      break;
    }
    closed.insert(ck);
    for (const auto& nb : kNeighbors) {
      const int nr = cur.r + nb.dr;
      const int nc = cur.c + nb.dc;
      if (!costmap.in_bounds(nr, nc)) {
        continue;
      }
      const long long nk = key(nr, nc);
      if (closed.count(nk)) {
        continue;
      }
      const double cell_cost = static_cast<double>(costmap.cost_at(nr, nc));
      if (cell_cost >= 0.99) {  // lethal cell excluded
        continue;
      }
      const double tentative =
          cur.g + nb.step * (1.0 + w_cost * cell_cost);
      auto it = g_score.find(nk);
      if (it == g_score.end() || tentative < it->second) {
        g_score[nk] = tentative;
        came_from[nk] = ck;
        const double f = tentative + octile(nr, nc, gr, gc);
        open_heap.push({f, tentative, nr, nc});
      }
    }
  }

  if (!found) {
    return std::nullopt;
  }

  // Reconstruct cell path goal -> start, then reverse to start -> goal.
  std::vector<std::pair<int, int>> cells;
  cells.emplace_back(gr, gc);
  long long node = key(gr, gc);
  auto it = came_from.find(node);
  while (it != came_from.end()) {
    node = it->second;
    const int r = static_cast<int>(node >> 32);
    const int c = static_cast<int>(static_cast<unsigned int>(node & 0xffffffff));
    cells.emplace_back(r, c);
    it = came_from.find(node);
  }
  std::reverse(cells.begin(), cells.end());

  // World points at cell centers, with exact start prepended and exact goal
  // appended.
  std::vector<std::array<double, 2>> pts;
  pts.push_back({start_x, start_y});
  for (const auto& rc : cells) {
    double wx, wy;
    costmap.grid_to_world(rc.first, rc.second, wx, wy);
    pts.push_back({wx, wy});
  }
  pts.push_back({goal_x, goal_y});

  if (cfg.simplify) {
    pts = simplify(costmap, pts);
  }

  return pts;
}

}  // namespace amr_planning
