// SPDX-License-Identifier: Apache-2.0
// ILocalizer: standardized localization interface over a known map. Bridges a
// predict/correct/estimate filter to a single tick.
#pragma once

#include <optional>

#include "amr_api/types.hpp"

namespace amr_api {

struct LocalizerInput {
  Pose2D odom_delta;
  std::optional<LaserScan> scan;  // correction runs only when present
};

struct LocalizerOutput {
  Pose2D pose;
  ParticleCloud cloud;
};

class ILocalizer {
 public:
  virtual ~ILocalizer() = default;

  /// Predict with odom_delta, correct if a scan is present, return the estimate.
  virtual LocalizerOutput update(const LocalizerInput& in) = 0;

  /// Current weighted-mean pose estimate.
  virtual Pose2D estimate() const = 0;

  /// Re-seed the belief about `pose`.
  virtual void set_pose(const Pose2D& pose) = 0;
};

}  // namespace amr_api
