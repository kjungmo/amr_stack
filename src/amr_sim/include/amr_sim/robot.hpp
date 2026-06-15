// SPDX-License-Identifier: Apache-2.0
// Differential-drive robot with exact-arc kinematics and footprint collision.
// Port of amr/sim/robot.py.
//
// Ground-truth state only; odometry noise lives in the Simulator. Motion is the
// exact arc integration of a constant (v, omega) over dt, with both velocity
// and acceleration clamped to the robot's limits.
#pragma once

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_sim/world.hpp"

namespace amr_sim {

class DiffDriveRobot {
 public:
  DiffDriveRobot(const amr_core::RobotConfig& cfg, const amr_core::Pose2D& spawn)
      : cfg_(cfg), pose_(spawn), vel_{0.0, 0.0}, collided_(false) {}

  /// Advance the ground-truth state by one step under command `cmd`.
  void step(const amr_core::Twist2D& cmd, double dt, const World& world);

  const amr_core::Pose2D& pose() const { return pose_; }
  const amr_core::Twist2D& vel() const { return vel_; }
  bool collided() const { return collided_; }

 private:
  bool collides(double x, double y, const World& world) const;

  amr_core::RobotConfig cfg_;
  amr_core::Pose2D pose_;
  amr_core::Twist2D vel_;
  bool collided_;
};

}  // namespace amr_sim
