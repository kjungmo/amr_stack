// SPDX-License-Identifier: Apache-2.0
// ISlam: standardized SLAM front-end interface. Fuses an odometry increment and
// an optional laser scan into a map-frame pose, and owns the live map.
#pragma once

#include <optional>

#include "amr_api/types.hpp"

namespace amr_api {

struct SlamInput {
  Pose2D odom_delta;              // robot-frame increment since last tick
  std::optional<LaserScan> scan;  // present only on scan ticks
};

class ISlam {
 public:
  virtual ~ISlam() = default;

  /// Fuse one tick; return the updated map-frame pose.
  virtual Pose2D update(const SlamInput& in) = 0;

  /// Current best map-frame pose.
  virtual Pose2D pose() const = 0;

  /// Snapshot the live occupancy map.
  virtual OccupancyGrid map() const = 0;
};

}  // namespace amr_api
