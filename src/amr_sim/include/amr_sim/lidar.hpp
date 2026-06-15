// SPDX-License-Identifier: Apache-2.0
// Vectorized 2-D lidar that ray-casts against the world occupancy grid.
// Port of amr/sim/lidar.py.
//
// The whole (B, S) sample grid (B beams x S range samples) is built and
// converted to integer cells; the first occupied sample per beam is the range.
#pragma once

#include <random>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_sim/world.hpp"

namespace amr_sim {

class Lidar {
 public:
  Lidar(const amr_core::LidarConfig& cfg, std::mt19937& rng)
      : cfg_(cfg), rng_(rng) {}

  /// Cast all beams against `world` from `pose`, returning a LaserScan stamped
  /// at `stamp`. A no-return beam is encoded as range_max.
  amr_core::LaserScan scan(const World& world, const amr_core::Pose2D& pose,
                           double stamp);

 private:
  amr_core::LidarConfig cfg_;
  std::mt19937& rng_;
};

}  // namespace amr_sim
