// SPDX-License-Identifier: Apache-2.0
#include "amr_sim/robot.hpp"

#include <cmath>

#include "amr_core/geometry.hpp"

namespace amr_sim {

namespace {
double clamp(double value, double lo, double hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}
}  // namespace

void DiffDriveRobot::step(const amr_core::Twist2D& cmd, double dt,
                          const World& world) {
  const amr_core::RobotConfig& cfg = cfg_;

  // 1. Clamp command to velocity limits.
  const double v_cmd = clamp(cmd.v, -cfg.max_lin_vel, cfg.max_lin_vel);
  const double w_cmd = clamp(cmd.omega, -cfg.max_ang_vel, cfg.max_ang_vel);

  // 2. Clamp the change from the current velocity by max accel * dt.
  const double dv_max = cfg.max_lin_acc * dt;
  const double dw_max = cfg.max_ang_acc * dt;
  const double v = vel_.v + clamp(v_cmd - vel_.v, -dv_max, dv_max);
  const double w = vel_.omega + clamp(w_cmd - vel_.omega, -dw_max, dw_max);

  // 3. Integrate the exact arc to a candidate pose.
  const double th = pose_.theta;
  double nx, ny, nth;
  if (std::abs(w) < 1e-9) {
    nx = pose_.x + v * std::cos(th) * dt;
    ny = pose_.y + v * std::sin(th) * dt;
    nth = amr_core::wrap_angle(th + w * dt);
  } else {
    const double R = v / w;
    const double nth_raw = th + w * dt;
    nx = pose_.x + R * (std::sin(nth_raw) - std::sin(th));
    ny = pose_.y - R * (std::cos(nth_raw) - std::cos(th));
    nth = amr_core::wrap_angle(nth_raw);
  }

  // 4. Collision: test the center plus 8 footprint-circle points.
  if (collides(nx, ny, world)) {
    // Keep the old pose, zero velocity, latch collided for this step.
    vel_ = amr_core::Twist2D{0.0, 0.0};
    collided_ = true;
  } else {
    pose_ = amr_core::Pose2D{nx, ny, nth};
    vel_ = amr_core::Twist2D{v, w};
    collided_ = false;
  }
}

bool DiffDriveRobot::collides(double x, double y, const World& world) const {
  const double r = cfg_.radius;
  if (world.is_occupied_world(x, y)) {
    return true;
  }
  for (int k = 0; k < 8; ++k) {
    const double ang = 2.0 * M_PI * static_cast<double>(k) / 8.0;
    if (world.is_occupied_world(x + r * std::cos(ang),
                                y + r * std::sin(ang))) {
      return true;
    }
  }
  return false;
}

}  // namespace amr_sim
