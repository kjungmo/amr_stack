// SPDX-License-Identifier: Apache-2.0
// Log-odds occupancy-grid mapper. Faithful C++ port of
// amr/mapping/occupancy_grid_mapper.py.
//
// Update rule per scan:
//   For each beam in scan[::beam_subsample]:
//     * endpoint = pose (+) (r*cos(angle), r*sin(angle))
//     * Bresenham from robot cell to endpoint cell (amr_core::bresenham)
//     * every cell except the last gets log_odds += l_free
//     * the endpoint cell gets log_odds += l_occ only for a valid return
//     * no-return beams trace free to range_max*0.99, skipping the endpoint
//     * clamp log_odds to +/- l_clamp
//
// to_occupancy_grid (CONTRACT §2 item 5):
//   p = 1 - 1/(1 + exp(log_odds))
//   p > occupied_thresh -> 100 ; p < free_thresh -> 0 ; else -> -1
#pragma once

#include <utility>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"

namespace amr_mapping {

class OccupancyGridMapper {
 public:
  /// size_m = (width_x, height_y) in metres; origin_xy = world (x,y) of the
  /// outer corner of cell (row=0, col=0).
  OccupancyGridMapper(const amr_core::MappingConfig& cfg,
                      std::pair<double, double> size_m,
                      std::pair<double, double> origin_xy = {0.0, 0.0});

  /// Integrate one laser scan taken at `pose` into the log-odds map.
  void update(const amr_core::Pose2D& pose, const amr_core::LaserScan& scan);

  /// Convert the log-odds map to a ROS-style OccupancyGrid.
  amr_core::OccupancyGrid to_occupancy_grid() const;

  int rows() const { return rows_; }
  int cols() const { return cols_; }
  /// Raw log-odds accessor (row-major), for tests/inspection.
  float log_odds_at(int row, int col) const {
    return log_odds_[static_cast<std::size_t>(row) * cols_ + col];
  }

 private:
  bool in_bounds(int row, int col) const {
    return row >= 0 && row < rows_ && col >= 0 && col < cols_;
  }
  void world_to_cell(double x, double y, int& row, int& col) const;

  amr_core::MappingConfig cfg_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  int rows_;
  int cols_;
  std::vector<float> log_odds_;  // row-major, 0 = unobserved
};

}  // namespace amr_mapping
