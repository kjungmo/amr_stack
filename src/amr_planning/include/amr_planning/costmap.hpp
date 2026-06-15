// SPDX-License-Identifier: Apache-2.0
// Inflated costmap for global/local planning. Ports amr/planning/costmap.py.
//
// Builds a float cost grid in [0, 1] from an OccupancyGrid. Obstacle cells (and,
// optionally, unknown cells) are lethal; the cost decays outward over an
// inflation band. Because the inflation radius already accounts for the robot
// radius, all downstream collision checks treat the robot as a point.
#pragma once

#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"

namespace amr_planning {

class Costmap {
 public:
  static constexpr float LETHAL = 1.0f;

  Costmap(const amr_core::OccupancyGrid& grid,
          const amr_core::CostmapConfig& cfg, double robot_radius);

  // --- grid <-> world helpers (same semantics as OccupancyGrid) ---
  void world_to_grid(double x, double y, int& row, int& col) const;
  void grid_to_world(int row, int col, double& x, double& y) const;
  bool in_bounds(int row, int col) const;

  int rows() const { return rows_; }
  int cols() const { return cols_; }
  double resolution() const { return resolution_; }

  // Flat row-major cost grid (rows*cols).
  const std::vector<float>& cost() const { return cost_; }
  float cost_at(int row, int col) const {
    return cost_[static_cast<std::size_t>(row) * cols_ + col];
  }

  // --- cost queries ---
  bool is_lethal(int row, int col) const;
  double cost_at_world(double x, double y) const;

 private:
  double resolution_;
  double origin_x_;
  double origin_y_;
  int rows_;
  int cols_;
  double robot_radius_;
  std::vector<float> cost_;
};

}  // namespace amr_planning
