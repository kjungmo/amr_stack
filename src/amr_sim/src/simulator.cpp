// SPDX-License-Identifier: Apache-2.0
#include "amr_sim/simulator.hpp"

#include <cmath>
#include <utility>

#include "amr_core/geometry.hpp"

namespace amr_sim {

amr_core::Pose2D arc_robot_frame(double v, double w, double dt) {
  if (std::abs(w) < 1e-9) {
    return amr_core::Pose2D{v * dt, 0.0, 0.0};
  }
  const double R = v / w;
  const double dth = w * dt;
  const double dx = R * std::sin(dth);
  const double dy = R * (1.0 - std::cos(dth));
  return amr_core::Pose2D{dx, dy, amr_core::wrap_angle(dth)};
}

Simulator::Simulator(World world, const amr_core::AmrConfig& cfg,
                     std::mt19937& rng)
    : world_(std::move(world)),
      cfg_(cfg),
      rng_(rng),
      time_(0.0),
      steps_(0),
      robot_(cfg_.robot, world_.spawn()),
      lidar_(cfg_.lidar, rng_),
      odom_pose_(world_.spawn()) {}

SimStepResult Simulator::step(const amr_core::Twist2D& cmd) {
  const double dt = cfg_.sim.dt;

  // Advance ground truth.
  robot_.step(cmd, dt, world_);
  ++steps_;
  time_ += dt;

  // Noisy odometry from the actual (clamped) velocity.
  const double v = robot_.vel().v;
  const double w = robot_.vel().omega;
  const amr_core::OdomNoiseConfig& nz = cfg_.sim.odom_noise;
  const double v_std = nz.alpha_v * std::abs(v) + nz.floor;
  const double w_std = nz.alpha_w * std::abs(w) + nz.floor;
  std::normal_distribution<double> v_dist(0.0, v_std);
  std::normal_distribution<double> w_dist(0.0, w_std);
  const double v_meas = v + v_dist(rng_);
  const double w_meas = w + w_dist(rng_);

  const amr_core::Pose2D odom_delta = arc_robot_frame(v_meas, w_meas, dt);
  odom_pose_ = amr_core::pose_compose(odom_pose_, odom_delta);

  SimStepResult result;
  result.ground_truth = robot_.pose();
  result.odom_pose = odom_pose_;
  result.odom_delta = odom_delta;
  result.collided = robot_.collided();
  result.sim_time = time_;

  // Scan every scan_every steps.
  const int scan_every = cfg_.lidar.scan_every;
  if (scan_every > 0 && (steps_ % scan_every == 0)) {
    result.scan = lidar_.scan(world_, robot_.pose(), time_);
    result.scan_ready = true;
  } else {
    result.scan_ready = false;
  }

  return result;
}

}  // namespace amr_sim
