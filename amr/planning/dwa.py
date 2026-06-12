"""Dynamic Window Approach local planner.

Samples a grid of (v, omega) commands within the velocity/acceleration-limited
dynamic window, rolls each out with a unicycle model, rejects rollouts that hit a
lethal costmap cell, and scores the survivors against a carrot point taken a fixed
arc length ahead on the global path.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from amr.core.config import DwaConfig, RobotConfig
from amr.core.geometry import wrap_angle
from amr.core.types import Pose2D, Twist2D
from amr.planning.costmap import Costmap


@dataclass
class DwaResult:
    cmd: Twist2D
    trajectory: np.ndarray    # (K, 3) poses of the chosen rollout
    blocked: bool             # True if every sampled rollout collides


def carrot_point(path: np.ndarray, pose: Pose2D, lookahead: float) -> np.ndarray:
    """Point ``lookahead`` metres of arc length beyond the projection of the
    pose onto the path polyline; clamps to the final point (the goal)."""
    path = np.asarray(path, dtype=float)
    if len(path) == 1:
        return path[0].copy()
    p = np.array([pose.x, pose.y])

    # Find the closest point on any segment; record its cumulative arc length.
    best_d2 = float("inf")
    best_arc = 0.0
    arc = 0.0
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        seg = b - a
        seg_len = math.hypot(seg[0], seg[1])
        if seg_len < 1e-12:
            continue
        t = float(np.dot(p - a, seg) / (seg_len * seg_len))
        t = min(1.0, max(0.0, t))
        proj = a + t * seg
        d2 = float((p[0] - proj[0]) ** 2 + (p[1] - proj[1]) ** 2)
        if d2 < best_d2:
            best_d2 = d2
            best_arc = arc + t * seg_len
        arc += seg_len

    target_arc = best_arc + lookahead

    # Walk forward along the polyline to the target arc length.
    arc = 0.0
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        seg = b - a
        seg_len = math.hypot(seg[0], seg[1])
        if seg_len < 1e-12:
            continue
        if arc + seg_len >= target_arc:
            t = (target_arc - arc) / seg_len
            t = min(1.0, max(0.0, t))
            return a + t * seg
        arc += seg_len
    # Past the end of the path -> clamp to the final point.
    return path[-1].copy()


class DwaPlanner:
    def __init__(self, cfg: DwaConfig, robot: RobotConfig):
        self.cfg = cfg
        self.robot = robot

    def _rollout(self, pose: Pose2D, v: float, omega: float
                 ) -> np.ndarray:
        cfg = self.cfg
        n = int(round(cfg.sim_time / cfg.sim_dt))
        x, y, th = pose.x, pose.y, pose.theta
        out = np.empty((n, 3), dtype=float)
        dt = cfg.sim_dt
        for k in range(n):
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th = wrap_angle(th + omega * dt)
            out[k, 0] = x
            out[k, 1] = y
            out[k, 2] = th
        return out

    def compute(self, pose: Pose2D, vel: Twist2D, path: np.ndarray,
                costmap: Costmap) -> DwaResult:
        cfg = self.cfg
        robot = self.robot
        T = cfg.sim_time

        v_max = robot.max_lin_vel
        w_max = robot.max_ang_vel

        # Dynamic window (sim_time used as the reachability horizon).
        v_lo = max(0.0, vel.v - robot.max_lin_acc * T)
        v_hi = min(v_max, vel.v + robot.max_lin_acc * T)
        w_lo = max(-w_max, vel.omega - robot.max_ang_acc * T)
        w_hi = min(w_max, vel.omega + robot.max_ang_acc * T)

        carrot = carrot_point(path, pose, cfg.lookahead)
        goal = np.asarray(path, dtype=float)[-1]
        dist_to_goal = math.hypot(goal[0] - pose.x, goal[1] - pose.y)

        # Near-goal slowdown: cap sampled v.
        if dist_to_goal < cfg.lookahead:
            v_hi = min(v_hi, max(0.1, 0.7 * dist_to_goal))
            v_hi = max(v_hi, v_lo)

        v_grid = np.linspace(v_lo, v_hi, cfg.v_samples)
        w_grid = np.linspace(w_lo, w_hi, cfg.w_samples)

        # Collect surviving rollouts and their raw score components.
        trajs = []
        vels = []
        progress = []
        heading = []
        clearance = []
        velocity = []

        for v in v_grid:
            for w in w_grid:
                traj = self._rollout(pose, float(v), float(w))
                # Per-pose cost; reject if any pose is lethal.
                costs = np.empty(traj.shape[0], dtype=float)
                lethal = False
                for k in range(traj.shape[0]):
                    cck = costmap.cost_at_world(traj[k, 0], traj[k, 1])
                    costs[k] = cck
                    if cck >= 0.99:
                        lethal = True
                        break
                if lethal:
                    continue

                end = traj[-1]
                d_end = math.hypot(end[0] - carrot[0], end[1] - carrot[1])
                bearing = math.atan2(carrot[1] - end[1], carrot[0] - end[0])
                head_err = abs(wrap_angle(bearing - end[2]))
                clear = float(np.min(1.0 - costs))

                trajs.append(traj)
                vels.append((float(v), float(w)))
                progress.append(-d_end)
                heading.append(-head_err)
                clearance.append(clear)
                velocity.append(float(v) / v_max if v_max > 0 else 0.0)

        if not trajs:
            return DwaResult(Twist2D(0.0, 0.0),
                             np.empty((0, 3), dtype=float), blocked=True)

        progress = np.asarray(progress)
        heading = np.asarray(heading)
        clearance = np.asarray(clearance)
        velocity = np.asarray(velocity)

        def _norm(a: np.ndarray) -> np.ndarray:
            lo = float(a.min())
            hi = float(a.max())
            if hi - lo < 1e-12:
                return np.ones_like(a)
            return (a - lo) / (hi - lo)

        total = (cfg.w_progress * _norm(progress)
                 + cfg.w_heading * _norm(heading)
                 + cfg.w_clearance * _norm(clearance)
                 + cfg.w_velocity * _norm(velocity))

        best = int(np.argmax(total))
        bv, bw = vels[best]
        return DwaResult(Twist2D(bv, bw), trajs[best], blocked=False)
