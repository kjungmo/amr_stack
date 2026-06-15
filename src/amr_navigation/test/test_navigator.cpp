// SPDX-License-Identifier: Apache-2.0
// Unit tests for the navigation FSM. Mirrors tests/test_navigator.py: the FSM
// transitions are exercised with a stubbed planner/controller, with NO running
// ROS node (the Navigator library is node-free by design).
#include <gtest/gtest.h>

#include <array>
#include <optional>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/costmap.hpp"
#include "amr_planning/dwa.hpp"

#include "amr_navigation/navigator.hpp"

using amr_core::Pose2D;
using amr_core::Twist2D;
using amr_navigation::NavState;
using amr_navigation::Navigator;
using amr_navigation::Path;

namespace {

// A small all-free costmap so cost_at_world() is 0 everywhere (never blocked).
amr_planning::Costmap make_free_costmap() {
  amr_core::OccupancyGrid grid;
  grid.resolution = 0.05;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.rows = 200;  // 10 m x 10 m
  grid.cols = 200;
  grid.data.assign(static_cast<std::size_t>(grid.rows) * grid.cols, 0);  // free
  return amr_planning::Costmap(grid, amr_core::CostmapConfig(), 0.18);
}

// A straight path from (1,1) towards (5,5).
Path straight_path() {
  return Path{{1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}, {4.0, 4.0}, {5.0, 5.0}};
}

// Planner stub that always succeeds, returning a fixed path.
amr_navigation::PlanFn ok_planner(const Path& p) {
  return [p](const std::array<double, 2>&, const std::array<double, 2>&,
             const amr_planning::Costmap&) -> std::optional<Path> { return p; };
}

// Planner stub that always fails.
amr_navigation::PlanFn fail_planner() {
  return [](const std::array<double, 2>&, const std::array<double, 2>&,
            const amr_planning::Costmap&) -> std::optional<Path> {
    return std::nullopt;
  };
}

// Controller stub that always returns an unblocked command.
amr_navigation::ControlFn ok_controller(double v, double w) {
  return [v, w](const Pose2D&, const Twist2D&, const Path&,
                const amr_planning::Costmap&) -> amr_planning::DwaResult {
    amr_planning::DwaResult r;
    r.cmd = Twist2D{v, w};
    r.blocked = false;
    return r;
  };
}

// Controller stub that always reports blocked.
amr_navigation::ControlFn blocked_controller() {
  return [](const Pose2D&, const Twist2D&, const Path&,
            const amr_planning::Costmap&) -> amr_planning::DwaResult {
    amr_planning::DwaResult r;
    r.blocked = true;
    return r;
  };
}

}  // namespace

// Mirrors test_idle_and_cancel: IDLE returns zero twist, cancel clears goal.
TEST(NavigatorFsm, IdleAndCancel) {
  auto cm = make_free_costmap();
  Navigator nav(amr_core::NavConfig(), ok_planner(straight_path()),
                ok_controller(0.2, 0.0));

  EXPECT_EQ(nav.state(), NavState::IDLE);
  Twist2D out = nav.update(Pose2D{1, 1, 0}, Twist2D{}, cm, 0.0);
  EXPECT_DOUBLE_EQ(out.v, 0.0);
  EXPECT_DOUBLE_EQ(out.omega, 0.0);

  nav.set_goal(Pose2D{5, 5, 0});
  EXPECT_EQ(nav.state(), NavState::PLANNING);
  nav.cancel();
  EXPECT_EQ(nav.state(), NavState::IDLE);
  EXPECT_FALSE(nav.has_goal());
}

// IDLE -> PLANNING -> FOLLOWING in a single set_goal + update.
// set_goal enters PLANNING; the first update plans (success) -> FOLLOWING and
// returns the DWA command this same tick.
TEST(NavigatorFsm, IdleToPlanningToFollowing) {
  auto cm = make_free_costmap();
  Navigator nav(amr_core::NavConfig(), ok_planner(straight_path()),
                ok_controller(0.25, 0.0));

  nav.set_goal(Pose2D{5.0, 5.0, 0.0});
  EXPECT_EQ(nav.state(), NavState::PLANNING);

  // Robot far from goal, on the path: planning succeeds, follow this tick.
  Twist2D cmd = nav.update(Pose2D{1.0, 1.0, 0.0}, Twist2D{}, cm, 0.0);
  EXPECT_EQ(nav.state(), NavState::FOLLOWING);
  EXPECT_DOUBLE_EQ(cmd.v, 0.25);  // DWA stub command emitted
  EXPECT_TRUE(nav.path().has_value());
}

// FOLLOWING -> SUCCEEDED once within goal_tol_xy of the goal.
TEST(NavigatorFsm, FollowingToSucceeded) {
  auto cm = make_free_costmap();
  amr_core::NavConfig cfg;
  Navigator nav(cfg, ok_planner(straight_path()), ok_controller(0.25, 0.0));

  nav.set_goal(Pose2D{5.0, 5.0, 0.0});
  // First tick: PLANNING -> FOLLOWING.
  nav.update(Pose2D{1.0, 1.0, 0.0}, Twist2D{}, cm, 0.0);
  ASSERT_EQ(nav.state(), NavState::FOLLOWING);

  // Now within goal_tol_xy: should transition to SUCCEEDED, zero twist.
  Pose2D at_goal{5.0 + cfg.goal_tol_xy * 0.5, 5.0, 0.0};
  Twist2D cmd = nav.update(at_goal, Twist2D{}, cm, 0.1);
  EXPECT_EQ(nav.state(), NavState::SUCCEEDED);
  EXPECT_DOUBLE_EQ(cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(cmd.omega, 0.0);

  // SUCCEEDED is terminal: further updates return zero, stay SUCCEEDED.
  Twist2D again = nav.update(at_goal, Twist2D{}, cm, 0.2);
  EXPECT_EQ(nav.state(), NavState::SUCCEEDED);
  EXPECT_DOUBLE_EQ(again.v, 0.0);
}

// Planning failure escalates to RECOVERY, then FAILED after max_recoveries.
// Mirrors test_unreachable_goal_fails_after_recoveries.
TEST(NavigatorFsm, PlanningFailureEscalatesToRecoveryThenFailed) {
  auto cm = make_free_costmap();
  amr_core::NavConfig cfg;  // max_recoveries = 3 by default
  Navigator nav(cfg, fail_planner(), ok_controller(0.0, 0.0));

  nav.set_goal(Pose2D{3.0, 3.0, 0.0});
  EXPECT_EQ(nav.state(), NavState::PLANNING);

  bool saw_recovery = false;
  double t = 0.0;
  // Drive the FSM forward; planning always fails so it cycles
  // PLANNING -> RECOVERY -> (rotate full 2pi) -> PLANNING ... until budget out.
  for (int i = 0; i < 100000 && nav.state() != NavState::FAILED; ++i) {
    nav.update(Pose2D{1.0, 1.0, 0.0}, Twist2D{}, cm, t);
    if (nav.state() == NavState::RECOVERY) {
      saw_recovery = true;
    }
    t += 0.05;
  }
  EXPECT_TRUE(saw_recovery);
  EXPECT_EQ(nav.state(), NavState::FAILED);
}

// Recovery rotate-in-place: emits the rotate command and completes after the
// integrated rotation reaches 2*pi (via the caller clock), then re-enters
// PLANNING with the next behaviour (backup).
TEST(NavigatorFsm, RecoveryRotateThenAlternatesToBackup) {
  auto cm = make_free_costmap();
  amr_core::NavConfig cfg;
  Navigator nav(cfg, fail_planner(), ok_controller(0.0, 0.0));

  nav.set_goal(Pose2D{3.0, 3.0, 0.0});
  // First update: PLANNING fails -> RECOVERY, emits the rotate command.
  Twist2D first = nav.update(Pose2D{1, 1, 0}, Twist2D{}, cm, 0.0);
  ASSERT_EQ(nav.state(), NavState::RECOVERY);
  EXPECT_DOUBLE_EQ(first.omega, cfg.recovery_rotate_speed);
  EXPECT_DOUBLE_EQ(first.v, 0.0);

  // Integrate rotation until 2*pi accumulates. dt is from the caller clock.
  double t = 0.0;
  const double dt = 0.05;
  int guard = 0;
  while (nav.state() == NavState::RECOVERY && guard < 100000) {
    t += dt;
    nav.update(Pose2D{1, 1, 0}, Twist2D{}, cm, t);
    ++guard;
  }
  // Completing the rotate behaviour re-enters PLANNING; planning fails again so
  // the next recovery uses the backup behaviour (negative v).
  Twist2D after = nav.update(Pose2D{1, 1, 0}, Twist2D{}, cm, t);
  ASSERT_EQ(nav.state(), NavState::RECOVERY);
  EXPECT_LT(after.v, 0.0);  // backing up
  EXPECT_DOUBLE_EQ(after.v, -cfg.recovery_backup_speed);
}

// DWA-blocked rollout during FOLLOWING routes to RECOVERY.
TEST(NavigatorFsm, FollowingDwaBlockedEntersRecovery) {
  auto cm = make_free_costmap();
  amr_core::NavConfig cfg;
  Navigator nav(cfg, ok_planner(straight_path()), blocked_controller());

  nav.set_goal(Pose2D{5.0, 5.0, 0.0});
  // First update: PLANNING -> FOLLOWING, but DWA reports blocked -> RECOVERY.
  Twist2D cmd = nav.update(Pose2D{1.0, 1.0, 0.0}, Twist2D{}, cm, 0.0);
  EXPECT_EQ(nav.state(), NavState::RECOVERY);
  // First recovery command is the rotate.
  EXPECT_DOUBLE_EQ(cmd.omega, cfg.recovery_rotate_speed);
}

// distance_remaining reflects straight-line distance to the goal.
TEST(NavigatorFsm, DistanceRemaining) {
  Navigator nav(amr_core::NavConfig(), ok_planner(straight_path()),
                ok_controller(0.0, 0.0));
  EXPECT_DOUBLE_EQ(nav.distance_remaining(Pose2D{0, 0, 0}), 0.0);  // no goal
  nav.set_goal(Pose2D{3.0, 4.0, 0.0});
  EXPECT_DOUBLE_EQ(nav.distance_remaining(Pose2D{0.0, 0.0, 0.0}), 5.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
