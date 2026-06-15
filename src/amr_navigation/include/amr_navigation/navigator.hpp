// SPDX-License-Identifier: Apache-2.0
// Goal-driven navigation finite-state machine. Ports amr/navigation/navigator.py.
//
// Drives a differential-drive robot to a goal pose by combining the planning
// stack: a global A* path over an inflated Costmap, a Dynamic Window Approach
// local planner that tracks the path, periodic / event-triggered replanning, and
// a pair of alternating recovery behaviours (rotate-in-place and back-up) when
// the robot becomes blocked or planning fails.
//
// The algorithm lives entirely in this library and is unit-testable without a
// running ROS node. The planner and controller are injected as std::function
// callbacks so gtests can stub them (the navigator_node wires the real
// amr_planning::plan_path / DwaPlanner). State transitions follow the frozen
// spec exactly:
//
//   IDLE / SUCCEEDED / FAILED  -> emit a zero twist, do nothing.
//   PLANNING                   -> plan a path; success -> FOLLOWING,
//                                 failure -> bump recovery_count and either
//                                 RECOVERY or (if exhausted) FAILED.
//   FOLLOWING                  -> if at goal -> SUCCEEDED; if the replan timer
//                                 fired or the path ahead is blocked, replan
//                                 inline (same tick); if DWA reports blocked
//                                 -> recovery; otherwise emit the DWA command.
//   RECOVERY                   -> run the current behaviour (rotate 2*pi, then
//                                 back up, alternating per entry), integrating
//                                 progress from successive caller-clock deltas;
//                                 when the behaviour completes -> PLANNING.
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/costmap.hpp"
#include "amr_planning/dwa.hpp"

namespace amr_navigation {

enum class NavState { IDLE, PLANNING, FOLLOWING, RECOVERY, SUCCEEDED, FAILED };

/// Stable string label (matches the Python NavState.value strings and the
/// amr_interfaces feedback/result `state` field values).
const char* to_string(NavState s);

/// A path is a sequence of world (x, y) waypoints (same as amr_planning::Path).
using Path = std::vector<std::array<double, 2>>;

/// Planner callback: world start_xy, goal_xy, costmap -> path or nullopt.
/// Mirrors amr_planning::plan_path with the navigator's astar config bound in.
using PlanFn = std::function<std::optional<Path>(
    const std::array<double, 2>& start_xy,
    const std::array<double, 2>& goal_xy,
    const amr_planning::Costmap& costmap)>;

/// Controller callback: pose, vel, path, costmap -> DWA result.
/// Mirrors amr_planning::DwaPlanner::compute.
using ControlFn = std::function<amr_planning::DwaResult(
    const amr_core::Pose2D& pose, const amr_core::Twist2D& vel,
    const Path& path, const amr_planning::Costmap& costmap)>;

class Navigator {
 public:
  /// Construct with the nav config and injected planner/controller callbacks.
  Navigator(const amr_core::NavConfig& cfg, PlanFn plan_fn, ControlFn control_fn);

  /// Convenience: build the default planner/controller from the planning config
  /// and robot config (wires amr_planning::plan_path + DwaPlanner). Used by the
  /// node; gtests use the injecting constructor with stubs.
  static Navigator make_default(const amr_core::NavConfig& cfg,
                                const amr_core::AstarConfig& astar_cfg,
                                const amr_core::DwaConfig& dwa_cfg,
                                const amr_core::RobotConfig& robot_cfg);

  // -------------------------------------------------------------------- API
  /// Accept a new goal: reset recovery state and (re)enter PLANNING.
  void set_goal(const amr_core::Pose2D& goal);

  /// Abandon the current goal: go IDLE, clear goal and path.
  void cancel();

  /// Advance the FSM by one tick and return the commanded twist.
  amr_core::Twist2D update(const amr_core::Pose2D& pose,
                           const amr_core::Twist2D& vel,
                           const amr_planning::Costmap& costmap, double now);

  // --------------------------------------------------------------- accessors
  NavState state() const { return state_; }
  bool has_goal() const { return goal_.has_value(); }
  const std::optional<amr_core::Pose2D>& goal() const { return goal_; }
  const std::optional<Path>& path() const { return path_; }

  /// Straight-line distance from `pose` to the goal, or 0 if no goal.
  double distance_remaining(const amr_core::Pose2D& pose) const;

 private:
  // --- state handlers (mirror the Python private methods) ---
  amr_core::Twist2D do_planning(const amr_core::Pose2D& pose,
                                const amr_core::Twist2D& vel,
                                const amr_planning::Costmap& costmap, double now);
  amr_core::Twist2D do_following(const amr_core::Pose2D& pose,
                                 const amr_core::Twist2D& vel,
                                 const amr_planning::Costmap& costmap,
                                 double now);
  amr_core::Twist2D do_recovery(const amr_core::Pose2D& pose,
                                const amr_core::Twist2D& vel,
                                const amr_planning::Costmap& costmap, double now);

  amr_core::Twist2D follow_cmd(const amr_core::Pose2D& pose,
                               const amr_core::Twist2D& vel,
                               const amr_planning::Costmap& costmap, double now);
  amr_core::Twist2D enter_recovery(double now);
  amr_core::Twist2D finish_recovery();
  amr_core::Twist2D on_plan_failure(double now);

  // --- helpers ---
  std::optional<Path> plan(const amr_core::Pose2D& pose,
                           const amr_planning::Costmap& costmap) const;
  bool at_goal(const amr_core::Pose2D& pose) const;
  bool path_blocked_ahead(const amr_core::Pose2D& pose,
                          const amr_planning::Costmap& costmap) const;
  void reset_recovery_progress();
  void transition(NavState new_state);

  amr_core::NavConfig cfg_;
  PlanFn plan_fn_;
  ControlFn control_fn_;

  NavState state_{NavState::IDLE};
  std::optional<amr_core::Pose2D> goal_{};
  std::optional<Path> path_{};

  // Replan bookkeeping.
  double plan_time_{0.0};
  int recovery_count_{0};

  // Recovery-behaviour bookkeeping.
  int recovery_behavior_{0};        // 0 = rotate, 1 = back up; alternates
  double recovery_progress_{0.0};   // integrated rotation (rad) / distance (m)
  bool has_recovery_last_now_{false};
  double recovery_last_now_{0.0};
};

}  // namespace amr_navigation
