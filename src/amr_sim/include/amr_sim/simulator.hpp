// SPDX-License-Identifier: Apache-2.0
// Top-level 2-D simulator: advances the robot, emits noisy odometry and scans.
// Port of amr/sim/simulator.py.
//
// Each step integrates the ground-truth robot, derives a noisy odometry
// increment (in the robot frame) and accumulates it into a drifting odom pose,
// and emits a lidar scan every cfg.lidar.scan_every steps.
#pragma once

#include <random>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_sim/lidar.hpp"
#include "amr_sim/robot.hpp"
#include "amr_sim/world.hpp"

namespace amr_sim {

/// Result of a single Simulator::step.
struct SimStepResult {
  amr_core::Pose2D ground_truth;   // exact GT pose
  amr_core::Pose2D odom_pose;      // accumulated noisy odom pose
  amr_core::Pose2D odom_delta;     // noisy increment this step (robot frame)
  amr_core::LaserScan scan;        // valid iff scan_ready
  bool scan_ready{false};          // a scan was emitted this step
  bool collided{false};            // robot collided this step
  double sim_time{0.0};            // sim clock after this step
};

/// Exact-arc integration of (v, w) over dt, expressed in the robot frame
/// (start pose at origin, heading +x). Returns Pose2D(dx_fwd, dy_left, dth).
amr_core::Pose2D arc_robot_frame(double v, double w, double dt);

class Simulator {
 public:
  Simulator(World world, const amr_core::AmrConfig& cfg, std::mt19937& rng);

  /// Advance one step under command `cmd`.
  SimStepResult step(const amr_core::Twist2D& cmd);

  const World& world() const { return world_; }
  double time() const { return time_; }
  const amr_core::Pose2D& odom_pose() const { return odom_pose_; }

 private:
  World world_;
  amr_core::AmrConfig cfg_;
  std::mt19937& rng_;
  double time_;
  long steps_;
  DiffDriveRobot robot_;
  Lidar lidar_;
  amr_core::Pose2D odom_pose_;
};

}  // namespace amr_sim
