// SPDX-License-Identifier: Apache-2.0
// Navigation FSM implementation. Faithful port of amr/navigation/navigator.py.
#include "amr_navigation/navigator.hpp"

#include "amr_planning/astar.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

namespace amr_navigation {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;
}  // namespace

const char* to_string(NavState s) {
  switch (s) {
    case NavState::IDLE:      return "IDLE";
    case NavState::PLANNING:  return "PLANNING";
    case NavState::FOLLOWING: return "FOLLOWING";
    case NavState::RECOVERY:  return "RECOVERY";
    case NavState::SUCCEEDED: return "SUCCEEDED";
    case NavState::FAILED:    return "FAILED";
  }
  return "IDLE";  // unreachable; defensive
}

Navigator::Navigator(const amr_core::NavConfig& cfg, PlanFn plan_fn,
                     ControlFn control_fn)
    : cfg_(cfg), plan_fn_(std::move(plan_fn)),
      control_fn_(std::move(control_fn)) {}

Navigator Navigator::make_default(const amr_core::NavConfig& cfg,
                                  const amr_core::AstarConfig& astar_cfg,
                                  const amr_core::DwaConfig& dwa_cfg,
                                  const amr_core::RobotConfig& robot_cfg) {
  // Bind the astar config into the planner callback.
  PlanFn plan_fn =
      [astar_cfg](const std::array<double, 2>& start_xy,
                  const std::array<double, 2>& goal_xy,
                  const amr_planning::Costmap& costmap) -> std::optional<Path> {
    return amr_planning::plan_path(costmap, start_xy, goal_xy, astar_cfg);
  };
  // DwaPlanner is stateless; capture it by value (shared_ptr for copyability).
  auto dwa = std::make_shared<amr_planning::DwaPlanner>(dwa_cfg, robot_cfg);
  ControlFn control_fn =
      [dwa](const amr_core::Pose2D& pose, const amr_core::Twist2D& vel,
            const Path& path,
            const amr_planning::Costmap& costmap) -> amr_planning::DwaResult {
    return dwa->compute(pose, vel, path, costmap);
  };
  return Navigator(cfg, std::move(plan_fn), std::move(control_fn));
}

// ----------------------------------------------------------------------- API
void Navigator::set_goal(const amr_core::Pose2D& goal) {
  goal_ = amr_core::Pose2D{goal.x, goal.y, goal.theta};
  path_.reset();
  recovery_count_ = 0;
  recovery_behavior_ = 0;
  reset_recovery_progress();
  transition(NavState::PLANNING);
}

void Navigator::cancel() {
  goal_.reset();
  path_.reset();
  reset_recovery_progress();
  transition(NavState::IDLE);
}

amr_core::Twist2D Navigator::update(const amr_core::Pose2D& pose,
                                    const amr_core::Twist2D& vel,
                                    const amr_planning::Costmap& costmap,
                                    double now) {
  if (state_ == NavState::IDLE || state_ == NavState::SUCCEEDED ||
      state_ == NavState::FAILED) {
    return amr_core::Twist2D{0.0, 0.0};
  }
  if (state_ == NavState::PLANNING) {
    return do_planning(pose, vel, costmap, now);
  }
  if (state_ == NavState::FOLLOWING) {
    return do_following(pose, vel, costmap, now);
  }
  if (state_ == NavState::RECOVERY) {
    return do_recovery(pose, vel, costmap, now);
  }
  // Unreachable; defensive zero.
  return amr_core::Twist2D{0.0, 0.0};
}

// ------------------------------------------------------------------- PLANNING
amr_core::Twist2D Navigator::do_planning(const amr_core::Pose2D& pose,
                                         const amr_core::Twist2D& vel,
                                         const amr_planning::Costmap& costmap,
                                         double now) {
  auto path = plan(pose, costmap);
  if (path.has_value()) {
    path_ = std::move(path);
    plan_time_ = now;
    transition(NavState::FOLLOWING);
    // Compute and return the first command this same tick.
    return follow_cmd(pose, vel, costmap, now);
  }
  // Planning failed: escalate to recovery / failure.
  return on_plan_failure(now);
}

// ------------------------------------------------------------------ FOLLOWING
amr_core::Twist2D Navigator::do_following(const amr_core::Pose2D& pose,
                                          const amr_core::Twist2D& vel,
                                          const amr_planning::Costmap& costmap,
                                          double now) {
  // Goal reached?
  if (at_goal(pose)) {
    transition(NavState::SUCCEEDED);
    return amr_core::Twist2D{0.0, 0.0};
  }

  // Replan if the timer fired or the path ahead is blocked.
  if ((now - plan_time_ > cfg_.replan_period) ||
      path_blocked_ahead(pose, costmap)) {
    auto path = plan(pose, costmap);
    if (path.has_value()) {
      path_ = std::move(path);
      plan_time_ = now;
      // nav: replanned (state=FOLLOWING)
    } else {
      return on_plan_failure(now);
    }
  }

  return follow_cmd(pose, vel, costmap, now);
}

amr_core::Twist2D Navigator::follow_cmd(const amr_core::Pose2D& pose,
                                        const amr_core::Twist2D& vel,
                                        const amr_planning::Costmap& costmap,
                                        double now) {
  // Run DWA against the current path; route blocked rollouts to recovery.
  const Path& path = path_.value();
  amr_planning::DwaResult result = control_fn_(pose, vel, path, costmap);
  if (result.blocked) {
    return enter_recovery(now);
  }
  return result.cmd;
}

// -------------------------------------------------------------------- RECOVERY
amr_core::Twist2D Navigator::do_recovery(const amr_core::Pose2D& /*pose*/,
                                         const amr_core::Twist2D& /*vel*/,
                                         const amr_planning::Costmap& /*costmap*/,
                                         double now) {
  // Integrate progress from the caller's clock delta.
  double dt;
  if (!has_recovery_last_now_) {
    dt = 0.0;
  } else {
    dt = now - recovery_last_now_;
    if (dt < 0.0) {
      dt = 0.0;
    }
  }
  recovery_last_now_ = now;
  has_recovery_last_now_ = true;

  if (recovery_behavior_ == 0) {
    // Rotate in place until accumulated rotation reaches 2*pi.
    recovery_progress_ += cfg_.recovery_rotate_speed * dt;
    if (recovery_progress_ >= kTwoPi) {
      return finish_recovery();
    }
    return amr_core::Twist2D{0.0, cfg_.recovery_rotate_speed};
  }
  // Back up until accumulated distance reaches recovery_backup_dist.
  recovery_progress_ += cfg_.recovery_backup_speed * dt;
  if (recovery_progress_ >= cfg_.recovery_backup_dist) {
    return finish_recovery();
  }
  return amr_core::Twist2D{-cfg_.recovery_backup_speed, 0.0};
}

amr_core::Twist2D Navigator::enter_recovery(double now) {
  // Charge a recovery against the budget; transition to RECOVERY or FAILED.
  (void)now;
  recovery_count_ += 1;
  if (recovery_count_ > cfg_.max_recoveries) {
    transition(NavState::FAILED);
    return amr_core::Twist2D{0.0, 0.0};
  }
  reset_recovery_progress();
  transition(NavState::RECOVERY);
  // Emit the first recovery command this same tick.
  if (recovery_behavior_ == 0) {
    return amr_core::Twist2D{0.0, cfg_.recovery_rotate_speed};
  }
  return amr_core::Twist2D{-cfg_.recovery_backup_speed, 0.0};
}

amr_core::Twist2D Navigator::finish_recovery() {
  // A recovery behaviour completed: alternate behaviour and replan.
  recovery_behavior_ = 1 - recovery_behavior_;
  reset_recovery_progress();
  transition(NavState::PLANNING);
  return amr_core::Twist2D{0.0, 0.0};
}

amr_core::Twist2D Navigator::on_plan_failure(double now) {
  // Shared PLANNING-failure handling: bump count, recover or fail.
  (void)now;
  recovery_count_ += 1;
  if (recovery_count_ > cfg_.max_recoveries) {
    transition(NavState::FAILED);
    return amr_core::Twist2D{0.0, 0.0};
  }
  reset_recovery_progress();
  transition(NavState::RECOVERY);
  if (recovery_behavior_ == 0) {
    return amr_core::Twist2D{0.0, cfg_.recovery_rotate_speed};
  }
  return amr_core::Twist2D{-cfg_.recovery_backup_speed, 0.0};
}

// --------------------------------------------------------------------- helpers
std::optional<Path> Navigator::plan(const amr_core::Pose2D& pose,
                                    const amr_planning::Costmap& costmap) const {
  if (!goal_.has_value()) {
    return std::nullopt;
  }
  return plan_fn_({pose.x, pose.y}, {goal_->x, goal_->y}, costmap);
}

bool Navigator::at_goal(const amr_core::Pose2D& pose) const {
  if (!goal_.has_value()) {
    return false;
  }
  return std::hypot(pose.x - goal_->x, pose.y - goal_->y) < cfg_.goal_tol_xy;
}

bool Navigator::path_blocked_ahead(const amr_core::Pose2D& pose,
                                   const amr_planning::Costmap& costmap) const {
  // True if any path waypoint within `path_block_check_dist` arc length ahead of
  // the robot's projection onto the path is lethal.
  if (!path_.has_value() || path_->empty()) {
    return false;
  }
  const Path& path = path_.value();
  const double check_dist = cfg_.path_block_check_dist;

  // Find the index of the path vertex nearest the robot.
  std::size_t i0 = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double dx = path[i][0] - pose.x;
    const double dy = path[i][1] - pose.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      i0 = i;
    }
  }

  // Check forward along the polyline accumulating arc length up to check_dist.
  double acc = 0.0;
  for (std::size_t i = i0; i < path.size(); ++i) {
    const double x = path[i][0];
    const double y = path[i][1];
    if (costmap.cost_at_world(x, y) >= 0.99) {
      return true;
    }
    if (i + 1 < path.size()) {
      const double sx = path[i + 1][0] - path[i][0];
      const double sy = path[i + 1][1] - path[i][1];
      acc += std::hypot(sx, sy);
      if (acc > check_dist) {
        break;
      }
    }
  }
  return false;
}

double Navigator::distance_remaining(const amr_core::Pose2D& pose) const {
  if (!goal_.has_value()) {
    return 0.0;
  }
  return std::hypot(pose.x - goal_->x, pose.y - goal_->y);
}

void Navigator::reset_recovery_progress() {
  recovery_progress_ = 0.0;
  has_recovery_last_now_ = false;
  recovery_last_now_ = 0.0;
}

void Navigator::transition(NavState new_state) {
  // (state change logging is handled at the node layer via rclcpp)
  state_ = new_state;
}

}  // namespace amr_navigation
