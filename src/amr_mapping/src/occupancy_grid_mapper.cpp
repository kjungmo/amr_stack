// SPDX-License-Identifier: Apache-2.0
// Log-odds occupancy-grid mapper. Faithful C++ port of
// amr/mapping/occupancy_grid_mapper.py.
#include "amr_mapping/occupancy_grid_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "amr_core/geometry.hpp"

namespace amr_mapping {

OccupancyGridMapper::OccupancyGridMapper(const amr_core::MappingConfig& cfg,
                                         std::pair<double, double> size_m,
                                         std::pair<double, double> origin_xy)
    : cfg_(cfg),
      resolution_(cfg.resolution),
      origin_x_(origin_xy.first),
      origin_y_(origin_xy.second) {
  cols_ = static_cast<int>(std::lround(size_m.first / cfg.resolution));
  rows_ = static_cast<int>(std::lround(size_m.second / cfg.resolution));
  log_odds_.assign(static_cast<std::size_t>(rows_) * cols_, 0.0f);
}

void OccupancyGridMapper::world_to_cell(double x, double y, int& row,
                                        int& col) const {
  row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
}

void OccupancyGridMapper::update(const amr_core::Pose2D& pose,
                                 const amr_core::LaserScan& scan) {
  int robot_r = 0;
  int robot_c = 0;
  world_to_cell(pose.x, pose.y, robot_r, robot_c);

  const std::vector<double> angles = scan.angles();
  const std::vector<char> valid = scan.valid_mask();
  const int num_beams = scan.num_beams();

  const float l_free = static_cast<float>(cfg_.l_free);
  const float l_occ = static_cast<float>(cfg_.l_occ);
  const float l_clamp = static_cast<float>(cfg_.l_clamp);

  for (int i = 0; i < num_beams; i += cfg_.beam_subsample) {
    const double angle_b = angles[static_cast<std::size_t>(i)];
    const bool is_valid = valid[static_cast<std::size_t>(i)] != 0;

    double r_dist;
    if (is_valid) {
      r_dist = scan.ranges[static_cast<std::size_t>(i)];
    } else {
      // No-return beam: trace free space up to range_max * 0.99.
      r_dist = scan.range_max * 0.99;
    }

    // Beam direction in the world frame.
    const double beam_cos = std::cos(pose.theta + angle_b);
    const double beam_sin = std::sin(pose.theta + angle_b);

    const double end_x = pose.x + r_dist * beam_cos;
    const double end_y = pose.y + r_dist * beam_sin;
    int end_r = 0;
    int end_c = 0;
    world_to_cell(end_x, end_y, end_r, end_c);

    // Bresenham ray from robot cell to endpoint cell ((row, col) pairs).
    const std::vector<std::pair<int, int>> cells =
        amr_core::bresenham(robot_r, robot_c, end_r, end_c);

    // Mark every cell except the last as free.
    if (cells.size() > 1) {
      for (std::size_t k = 0; k + 1 < cells.size(); ++k) {
        const int cr = cells[k].first;
        const int cc = cells[k].second;
        if (in_bounds(cr, cc)) {
          float& lo = log_odds_[static_cast<std::size_t>(cr) * cols_ + cc];
          lo = std::min(std::max(lo + l_free, -l_clamp), l_clamp);
        }
      }
    }

    // Mark the endpoint cell as occupied (only for valid returns).
    if (is_valid && in_bounds(end_r, end_c)) {
      float& lo = log_odds_[static_cast<std::size_t>(end_r) * cols_ + end_c];
      lo = std::min(std::max(lo + l_occ, -l_clamp), l_clamp);
    }
  }
}

amr_core::OccupancyGrid OccupancyGridMapper::to_occupancy_grid() const {
  amr_core::OccupancyGrid grid;
  grid.resolution = resolution_;
  grid.origin_x = origin_x_;
  grid.origin_y = origin_y_;
  grid.rows = rows_;
  grid.cols = cols_;
  grid.data.assign(log_odds_.size(), static_cast<int8_t>(-1));

  for (std::size_t i = 0; i < log_odds_.size(); ++i) {
    const double lo = static_cast<double>(log_odds_[i]);
    const double p = 1.0 - 1.0 / (1.0 + std::exp(lo));
    if (p > cfg_.occupied_thresh) {
      grid.data[i] = 100;
    } else if (p < cfg_.free_thresh) {
      grid.data[i] = 0;
    }
    // else stays -1 (unknown; log_odds == 0 -> p == 0.5).
  }
  return grid;
}

}  // namespace amr_mapping
