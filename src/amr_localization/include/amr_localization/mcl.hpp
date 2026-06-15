// SPDX-License-Identifier: Apache-2.0
// Monte-Carlo localization (adaptive particle filter / AMCL-lite).
// Port of amr/localization/mcl.py. Maintains a cloud of weighted pose
// hypotheses: predict() pushes every particle through the Thrun odometry
// motion model; correct() reweights with the likelihood-field sensor model,
// renormalizes (with an underflow guard), and low-variance resamples whenever
// the effective sample size drops below resample_neff_frac * N; estimate()
// returns the weighted mean pose (circular mean for theta). See Probabilistic
// Robotics (Thrun et al.) Table 8.2 / 4.4.
#pragma once

#include <memory>
#include <optional>
#include <random>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_localization/sensor_model.hpp"

namespace amr_localization {

class MonteCarloLocalizer {
 public:
  /// Construct over a static map. If `initial_pose` is given the cloud is
  /// Gaussian-seeded around it (init_std); otherwise it is globally initialized
  /// uniformly over free cells. `rng` is the owned, seeded generator.
  MonteCarloLocalizer(
      const amr_core::OccupancyGrid& grid,
      const amr_core::LocalizationConfig& cfg, std::mt19937& rng,
      std::optional<amr_core::Pose2D> initial_pose = std::nullopt);

  /// Push every particle through the motion model (robot-frame odom delta).
  void predict(const amr_core::Pose2D& odom_delta);

  /// Reweight with the sensor model, renormalize, resample if needed.
  void correct(const amr_core::LaserScan& scan);

  /// Weighted mean pose (circular mean for theta).
  amr_core::Pose2D estimate() const;

  /// Re-seed the cloud (Gaussian about `pose`) with uniform weights.
  void set_pose(const amr_core::Pose2D& pose);

  const std::vector<amr_core::Pose2D>& particles() const { return particles_; }
  const std::vector<double>& weights() const { return weights_; }
  int size() const { return n_; }

 private:
  std::vector<amr_core::Pose2D> global_init();
  std::vector<amr_core::Pose2D> gaussian_init(const amr_core::Pose2D& pose);

  amr_core::OccupancyGrid grid_;
  amr_core::LocalizationConfig cfg_;
  std::mt19937& rng_;
  int n_;
  std::unique_ptr<LikelihoodField> field_;
  std::vector<amr_core::Pose2D> particles_;
  std::vector<double> weights_;
};

}  // namespace amr_localization
