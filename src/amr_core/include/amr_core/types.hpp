// SPDX-License-Identifier: Apache-2.0
// Shared AMR data types (C++/ROS 2 port). Mirrors amr/core/types.py.
// See the implementation plan §2 for the frozen coordinate/data conventions.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace amr_core {

/// 2-D rigid pose. theta in rad, CCW, wrapped to (-pi, pi].
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};

  std::array<double, 3> to_array() const { return {x, y, theta}; }
  static Pose2D from_array(const std::array<double, 3>& a) {
    return Pose2D{a[0], a[1], a[2]};
  }
};

/// Body twist: forward velocity v (m/s) and yaw rate omega (rad/s, CCW).
struct Twist2D {
  double v{0.0};
  double omega{0.0};
};

/// A single planar laser scan. A no-return beam is encoded as range_max.
struct LaserScan {
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
  std::vector<double> ranges;
  double stamp{0.0};  // sim time, s

  int num_beams() const { return static_cast<int>(ranges.size()); }

  std::vector<double> angles() const {
    std::vector<double> a(ranges.size());
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      a[i] = angle_min + angle_increment * static_cast<double>(i);
    }
    return a;
  }

  /// Beams strictly inside (range_min, 0.999*range_max).
  std::vector<char> valid_mask() const {
    std::vector<char> m(ranges.size());
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      m[i] = static_cast<char>((ranges[i] > range_min) &&
                               (ranges[i] < range_max * 0.999));
    }
    return m;
  }
};

/// Row-major occupancy grid. data values: -1 unknown, 0 free, 100 occupied.
/// origin_(x,y) is the world position of the outer corner of cell (row=0,col=0).
struct OccupancyGrid {
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  int rows{0};
  int cols{0};
  std::vector<int8_t> data;

  int8_t at(int r, int c) const {
    return data[static_cast<std::size_t>(r) * cols + c];
  }
  int8_t& at(int r, int c) {
    return data[static_cast<std::size_t>(r) * cols + c];
  }

  void world_to_grid(double x, double y, int& row, int& col) const {
    row = static_cast<int>(std::floor((y - origin_y) / resolution));
    col = static_cast<int>(std::floor((x - origin_x) / resolution));
  }
  void grid_to_world(int row, int col, double& x, double& y) const {
    x = origin_x + (static_cast<double>(col) + 0.5) * resolution;
    y = origin_y + (static_cast<double>(row) + 0.5) * resolution;
  }
  bool in_bounds(int row, int col) const {
    return row >= 0 && row < rows && col >= 0 && col < cols;
  }
};

}  // namespace amr_core
