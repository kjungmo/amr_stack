// SPDX-License-Identifier: Apache-2.0
#include "amr_planning/costmap.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "amr_core/geometry.hpp"

namespace amr_planning {

Costmap::Costmap(const amr_core::OccupancyGrid& grid,
                 const amr_core::CostmapConfig& cfg, double robot_radius)
    : resolution_(grid.resolution),
      origin_x_(grid.origin_x),
      origin_y_(grid.origin_y),
      rows_(grid.rows),
      cols_(grid.cols),
      robot_radius_(robot_radius) {
  const std::size_t n = static_cast<std::size_t>(rows_) * cols_;

  // Obstacle mask: occupied iff data >= occupied_thresh, plus unknown (<0) when
  // unknown_is_lethal.
  std::vector<std::uint8_t> mask(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    const int8_t v = grid.data[i];
    bool occ = v >= cfg.occupied_thresh;
    if (cfg.unknown_is_lethal && v < 0) {
      occ = true;
    }
    mask[i] = occ ? 1u : 0u;
  }

  // Use max_dist slightly beyond the inflation radius so genuinely-distant cells
  // land strictly outside the band (cost 0) instead of saturating at exactly
  // inflation_radius (indistinguishable from a cell truly on the band edge).
  std::vector<float> d = amr_core::distance_field(
      mask, rows_, cols_, grid.resolution,
      cfg.inflation_radius + grid.resolution);

  // distance_field reports the distance to the nearest occupied *cell center*;
  // the true distance from an arbitrary point to the obstacle is up to half a
  // cell smaller. Shift by half a cell so the bands hug the obstacle boundary.
  const double half_cell = 0.5 * grid.resolution;
  const double rr = robot_radius;
  const double infl = cfg.inflation_radius;

  cost_.assign(n, 0.0f);
  for (std::size_t i = 0; i < n; ++i) {
    double dist = static_cast<double>(d[i]) - half_cell;
    if (dist < 0.0) {
      dist = 0.0;
    }
    if (dist <= rr) {
      // Lethal core: within (inflated) robot radius of an obstacle.
      cost_[i] = 1.0f;
    } else if (dist < infl) {
      // Decay band: robot_radius < d < inflation_radius.
      cost_[i] = static_cast<float>(std::exp(-cfg.cost_decay * (dist - rr)));
    }
    // Beyond the inflation radius cost stays 0.0.
  }
}

void Costmap::world_to_grid(double x, double y, int& row, int& col) const {
  row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
}

void Costmap::grid_to_world(int row, int col, double& x, double& y) const {
  x = origin_x_ + (static_cast<double>(col) + 0.5) * resolution_;
  y = origin_y_ + (static_cast<double>(row) + 0.5) * resolution_;
}

bool Costmap::in_bounds(int row, int col) const {
  return row >= 0 && row < rows_ && col >= 0 && col < cols_;
}

bool Costmap::is_lethal(int row, int col) const {
  if (!in_bounds(row, col)) {
    return true;
  }
  return cost_at(row, col) >= 0.99f;
}

double Costmap::cost_at_world(double x, double y) const {
  int row, col;
  world_to_grid(x, y, row, col);
  if (!in_bounds(row, col)) {
    return 1.0;
  }
  return static_cast<double>(cost_at(row, col));
}

}  // namespace amr_planning
