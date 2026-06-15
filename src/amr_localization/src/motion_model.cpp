// SPDX-License-Identifier: Apache-2.0
#include "amr_localization/motion_model.hpp"

#include <cmath>

#include "amr_core/geometry.hpp"

namespace amr_localization {

std::vector<amr_core::Pose2D> sample_motion(
    const std::vector<amr_core::Pose2D>& particles,
    const amr_core::Pose2D& odom_delta, const std::vector<double>& alphas,
    std::mt19937& rng) {
  const double a1 = alphas[0];
  const double a2 = alphas[1];
  const double a3 = alphas[2];
  const double a4 = alphas[3];

  const double dx = odom_delta.x;
  const double dy = odom_delta.y;
  const double dth = odom_delta.theta;

  const double trans = std::hypot(dx, dy);
  const double rot1 = (trans > 1e-4) ? std::atan2(dy, dx) : 0.0;
  const double rot2 = amr_core::wrap_angle(dth - rot1);

  const double abs_rot1 = std::abs(rot1);
  const double abs_rot2 = std::abs(rot2);

  const double std_rot1 = a1 * abs_rot1 + a2 * trans;
  const double std_trans = a3 * trans + a4 * (abs_rot1 + abs_rot2);
  const double std_rot2 = a1 * abs_rot2 + a2 * trans;

  // Per-particle independent draws (one rot1/trans/rot2 sample each), matching
  // numpy's rng.normal(mean, std, n). A zero std yields the mean exactly.
  std::normal_distribution<double> n_rot1(0.0, std_rot1);
  std::normal_distribution<double> n_trans(0.0, std_trans);
  std::normal_distribution<double> n_rot2(0.0, std_rot2);

  std::vector<amr_core::Pose2D> out(particles.size());
  for (std::size_t i = 0; i < particles.size(); ++i) {
    const double rot1_s = rot1 + n_rot1(rng);
    const double trans_s = trans + n_trans(rng);
    const double rot2_s = rot2 + n_rot2(rng);

    const double th = particles[i].theta;
    out[i].x = particles[i].x + trans_s * std::cos(th + rot1_s);
    out[i].y = particles[i].y + trans_s * std::sin(th + rot1_s);
    out[i].theta = amr_core::wrap_angle(th + rot1_s + rot2_s);
  }
  return out;
}

}  // namespace amr_localization
