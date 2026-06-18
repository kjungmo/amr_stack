// SPDX-License-Identifier: Apache-2.0
// Pure-data aggregates shared across the amr_api interfaces. Reuses amr_core
// types verbatim and adds the few aggregates the interfaces exchange.
#pragma once

#include <array>
#include <vector>

#include "amr_core/types.hpp"

namespace amr_api {

using amr_core::LaserScan;
using amr_core::OccupancyGrid;
using amr_core::Pose2D;
using amr_core::Twist2D;

/// World (x, y) waypoints (identical shape to amr_planning::Path).
using Path = std::vector<std::array<double, 2>>;

/// Particle-filter snapshot for visualization / diagnostics.
struct ParticleCloud {
  std::vector<Pose2D> poses;
  std::vector<double> weights;
};

}  // namespace amr_api
