// SPDX-License-Identifier: Apache-2.0
// Likelihood-field range-finder sensor model.
// Port of amr/localization/sensor_model.py. Precomputes a chamfer distance
// field (metres to nearest occupied cell) over the static map and scores each
// particle with the per-beam likelihood
//   q = z_hit * exp(-d^2 / (2 sigma^2)) + z_rand / range_max
// The particle weight is exp(sum_b ln q). See Probabilistic Robotics
// (Thrun et al.) Table 6.3.
#pragma once

#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"

namespace amr_localization {

class LikelihoodField {
 public:
  LikelihoodField(const amr_core::OccupancyGrid& grid,
                  const amr_core::LikelihoodConfig& cfg);

  /// Unnormalized weights for each particle, one per pose in `particles`.
  std::vector<double> weigh(const std::vector<amr_core::Pose2D>& particles,
                            const amr_core::LaserScan& scan) const;

  /// Distance field (m to nearest occupied cell), row-major [row*cols + col].
  const std::vector<float>& field() const { return field_; }

 private:
  amr_core::OccupancyGrid grid_;
  amr_core::LikelihoodConfig cfg_;
  std::vector<float> field_;     // row-major distances (m)
  std::vector<char> unknown_;    // row-major: cell was unknown in source map
};

}  // namespace amr_localization
