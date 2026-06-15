// SPDX-License-Identifier: Apache-2.0
// Thrun odometry motion model (sample_motion_model_odometry).
// Port of amr/localization/motion_model.py. Decomposes the robot-frame
// relative odometry into rot1 / trans / rot2 and applies independent
// per-particle Gaussian noise. See Probabilistic Robotics (Thrun et al.)
// Table 5.6.
#pragma once

#include <random>
#include <vector>

#include "amr_core/types.hpp"

namespace amr_localization {

/// Apply the Thrun odometry motion model to a particle cloud.
///
/// `particles` are world poses (Pose2D). `odom_delta` is the robot-frame
/// relative motion (dx, dy, dtheta) since the last update. `alphas` are the
/// four noise coefficients (a1..a4). `rng` supplies seeded Gaussian noise.
/// Returns a new cloud of the same size.
std::vector<amr_core::Pose2D> sample_motion(
    const std::vector<amr_core::Pose2D>& particles,
    const amr_core::Pose2D& odom_delta, const std::vector<double>& alphas,
    std::mt19937& rng);

}  // namespace amr_localization
