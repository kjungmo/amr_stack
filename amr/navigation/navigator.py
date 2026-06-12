"""Goal-driven navigation finite-state machine.

Drives a differential-drive robot to a goal pose by combining the Phase-1
planning stack: a global A* path over an inflated :class:`Costmap`, a Dynamic
Window Approach local planner that tracks the path, periodic / event-triggered
replanning, and a pair of alternating recovery behaviours (rotate-in-place and
back-up) when the robot becomes blocked or planning fails.

The FSM states and transitions follow the frozen specification:

    IDLE / SUCCEEDED / FAILED  -> emit a zero twist, do nothing.
    PLANNING                   -> plan a path; success -> FOLLOWING,
                                  failure -> bump recovery_count and either
                                  RECOVERY or (if exhausted) FAILED.
    FOLLOWING                  -> if at goal -> SUCCEEDED; if the replan timer
                                  fired or the path ahead is blocked, replan
                                  inline (same tick); if DWA reports blocked
                                  -> recovery; otherwise emit the DWA command.
    RECOVERY                   -> run the current behaviour (rotate 2*pi, then
                                  back up, alternating per entry), integrating
                                  progress from successive caller-clock deltas;
                                  when the behaviour completes -> PLANNING.

All stochastic-free; the only state carried between ticks lives on the
``Navigator`` instance. Every state transition is logged at INFO on the
``amr.nav`` logger.
"""
from __future__ import annotations

import math
from enum import Enum
from typing import Optional

import numpy as np

from amr.core.config import AstarConfig, NavConfig
from amr.core.log import get_logger
from amr.core.types import Pose2D, Twist2D
from amr.planning.astar import plan_path
from amr.planning.costmap import Costmap
from amr.planning.dwa import DwaPlanner

_TWO_PI = 2.0 * math.pi


class NavState(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    FOLLOWING = "FOLLOWING"
    RECOVERY = "RECOVERY"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


class Navigator:
    """Goal-driven navigation FSM. See module docstring for the transition table."""

    state: NavState
    goal: Optional[Pose2D]
    path: Optional[np.ndarray]

    def __init__(self, cfg: NavConfig, astar_cfg: AstarConfig, dwa: DwaPlanner):
        self.cfg = cfg
        self.astar_cfg = astar_cfg
        self.dwa = dwa
        self._log = get_logger("nav")

        self.state = NavState.IDLE
        self.goal = None
        self.path = None

        # Replan bookkeeping.
        self.plan_time = 0.0
        self.recovery_count = 0

        # Recovery-behaviour bookkeeping.
        self._recovery_behavior = 0       # 0 = rotate, 1 = back up; alternates
        self._recovery_progress = 0.0     # integrated rotation (rad) / distance (m)
        self._recovery_last_now: Optional[float] = None

    # ------------------------------------------------------------------ API
    def set_goal(self, goal: Pose2D) -> None:
        """Accept a new goal: reset recovery state and (re)enter PLANNING."""
        self.goal = Pose2D(goal.x, goal.y, goal.theta)
        self.path = None
        self.recovery_count = 0
        self._recovery_behavior = 0
        self._reset_recovery_progress()
        self._transition(NavState.PLANNING)

    def cancel(self) -> None:
        """Abandon the current goal: go IDLE, clear goal and path."""
        self.goal = None
        self.path = None
        self._reset_recovery_progress()
        self._transition(NavState.IDLE)

    def update(self, pose: Pose2D, vel: Twist2D, costmap: Costmap,
               now: float) -> Twist2D:
        """Advance the FSM by one tick and return the commanded twist."""
        if self.state in (NavState.IDLE, NavState.SUCCEEDED, NavState.FAILED):
            return Twist2D(0.0, 0.0)

        if self.state == NavState.PLANNING:
            return self._do_planning(pose, vel, costmap, now)

        if self.state == NavState.FOLLOWING:
            return self._do_following(pose, vel, costmap, now)

        if self.state == NavState.RECOVERY:
            return self._do_recovery(pose, vel, costmap, now)

        # Unreachable; defensive zero.
        return Twist2D(0.0, 0.0)

    # --------------------------------------------------------------- PLANNING
    def _do_planning(self, pose: Pose2D, vel: Twist2D, costmap: Costmap,
                     now: float) -> Twist2D:
        path = self._plan(pose, costmap)
        if path is not None:
            self.path = path
            self.plan_time = now
            self._transition(NavState.FOLLOWING)
            # Compute and return the first command this same tick.
            return self._follow_cmd(pose, vel, costmap, now)
        # Planning failed: escalate to recovery / failure.
        return self._on_plan_failure(now)

    # -------------------------------------------------------------- FOLLOWING
    def _do_following(self, pose: Pose2D, vel: Twist2D, costmap: Costmap,
                      now: float) -> Twist2D:
        # Goal reached?
        if self._at_goal(pose):
            self._transition(NavState.SUCCEEDED)
            return Twist2D(0.0, 0.0)

        # Replan if the timer fired or the path ahead is blocked.
        if (now - self.plan_time > self.cfg.replan_period
                or self._path_blocked_ahead(pose, costmap)):
            path = self._plan(pose, costmap)
            if path is not None:
                self.path = path
                self.plan_time = now
                self._log.info("nav: replanned (state=FOLLOWING)")
            else:
                return self._on_plan_failure(now)

        return self._follow_cmd(pose, vel, costmap, now)

    def _follow_cmd(self, pose: Pose2D, vel: Twist2D, costmap: Costmap,
                    now: float) -> Twist2D:
        """Run DWA against the current path; route blocked rollouts to recovery."""
        result = self.dwa.compute(pose, vel, self.path, costmap)
        if result.blocked:
            return self._enter_recovery(now)
        return result.cmd

    # ---------------------------------------------------------------- RECOVERY
    def _do_recovery(self, pose: Pose2D, vel: Twist2D, costmap: Costmap,
                     now: float) -> Twist2D:
        # Integrate progress from the caller's clock delta.
        if self._recovery_last_now is None:
            dt = 0.0
        else:
            dt = now - self._recovery_last_now
            if dt < 0.0:
                dt = 0.0
        self._recovery_last_now = now

        if self._recovery_behavior == 0:
            # Rotate in place until accumulated rotation reaches 2*pi.
            self._recovery_progress += self.cfg.recovery_rotate_speed * dt
            if self._recovery_progress >= _TWO_PI:
                return self._finish_recovery()
            return Twist2D(0.0, self.cfg.recovery_rotate_speed)
        else:
            # Back up until accumulated distance reaches recovery_backup_dist.
            self._recovery_progress += self.cfg.recovery_backup_speed * dt
            if self._recovery_progress >= self.cfg.recovery_backup_dist:
                return self._finish_recovery()
            return Twist2D(-self.cfg.recovery_backup_speed, 0.0)

    def _enter_recovery(self, now: float) -> Twist2D:
        """Charge a recovery against the budget; transition to RECOVERY or FAILED."""
        self.recovery_count += 1
        if self.recovery_count > self.cfg.max_recoveries:
            self._transition(NavState.FAILED)
            return Twist2D(0.0, 0.0)
        self._reset_recovery_progress()
        self._transition(NavState.RECOVERY)
        # Emit the first recovery command this same tick.
        if self._recovery_behavior == 0:
            return Twist2D(0.0, self.cfg.recovery_rotate_speed)
        return Twist2D(-self.cfg.recovery_backup_speed, 0.0)

    def _finish_recovery(self) -> Twist2D:
        """A recovery behaviour completed: alternate behaviour and replan."""
        self._recovery_behavior = 1 - self._recovery_behavior
        self._reset_recovery_progress()
        self._transition(NavState.PLANNING)
        return Twist2D(0.0, 0.0)

    def _on_plan_failure(self, now: float) -> Twist2D:
        """Shared PLANNING-failure handling: bump count, recover or fail."""
        self.recovery_count += 1
        if self.recovery_count > self.cfg.max_recoveries:
            self._transition(NavState.FAILED)
            return Twist2D(0.0, 0.0)
        self._reset_recovery_progress()
        self._transition(NavState.RECOVERY)
        if self._recovery_behavior == 0:
            return Twist2D(0.0, self.cfg.recovery_rotate_speed)
        return Twist2D(-self.cfg.recovery_backup_speed, 0.0)

    # ------------------------------------------------------------- helpers
    def _plan(self, pose: Pose2D, costmap: Costmap) -> Optional[np.ndarray]:
        if self.goal is None:
            return None
        return plan_path(costmap, (pose.x, pose.y),
                         (self.goal.x, self.goal.y), self.astar_cfg)

    def _at_goal(self, pose: Pose2D) -> bool:
        if self.goal is None:
            return False
        return math.hypot(pose.x - self.goal.x,
                          pose.y - self.goal.y) < self.cfg.goal_tol_xy

    def _path_blocked_ahead(self, pose: Pose2D, costmap: Costmap) -> bool:
        """True if any path waypoint within ``path_block_check_dist`` arc length
        ahead of the robot's projection onto the path is lethal."""
        path = self.path
        if path is None or len(path) == 0:
            return False
        check_dist = self.cfg.path_block_check_dist

        # Find the index of the path vertex nearest the robot; check forward
        # along the polyline accumulating arc length up to check_dist.
        p = np.array([pose.x, pose.y], dtype=float)
        d2 = np.sum((path - p) ** 2, axis=1)
        i0 = int(np.argmin(d2))

        acc = 0.0
        for i in range(i0, len(path)):
            x, y = float(path[i, 0]), float(path[i, 1])
            if costmap.cost_at_world(x, y) >= 0.99:
                return True
            if i + 1 < len(path):
                seg = path[i + 1] - path[i]
                acc += math.hypot(float(seg[0]), float(seg[1]))
                if acc > check_dist:
                    break
        return False

    def _reset_recovery_progress(self) -> None:
        self._recovery_progress = 0.0
        self._recovery_last_now = None

    def _transition(self, new_state: NavState) -> None:
        if new_state != self.state:
            self._log.info("nav: %s -> %s", self.state.value, new_state.value)
        self.state = new_state
