// SPDX-License-Identifier: Apache-2.0
// 2-D ground-truth world: rasterized occupancy grid built from obstacle
// geometry. Port of amr/sim/world.py.
//
// The world grid is the simulator's ground truth (0 free, 100 occupied, never
// -1). Origin is fixed at (0, 0); cell (row, col) center is at
// ((col + 0.5) * res, (row + 0.5) * res).
#pragma once

#include <string>
#include <utility>

#include "amr_core/types.hpp"

namespace amr_sim {

/// Ground-truth world: occupancy grid + spawn pose + world size.
class World {
 public:
  World(amr_core::OccupancyGrid grid, amr_core::Pose2D spawn,
        std::pair<double, double> size)
      : grid_(std::move(grid)), spawn_(spawn), size_(size) {}

  /// Load a world YAML file (size, resolution, spawn, obstacles).
  static World from_yaml(const std::string& path);

  const amr_core::OccupancyGrid& grid() const { return grid_; }
  const amr_core::Pose2D& spawn() const { return spawn_; }
  std::pair<double, double> size() const { return size_; }

  /// True if (x, y) is occupied; out of bounds counts as occupied.
  bool is_occupied_world(double x, double y) const;

 private:
  amr_core::OccupancyGrid grid_;
  amr_core::Pose2D spawn_;
  std::pair<double, double> size_;
};

}  // namespace amr_sim
