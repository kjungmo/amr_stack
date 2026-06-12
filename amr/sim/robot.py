"""Differential-drive robot with exact-arc kinematics and footprint collision.

Ground-truth state only; odometry noise lives in the Simulator. Motion is the
exact arc integration of a constant (v, omega) over dt, with both velocity and
acceleration clamped to the robot's limits.
"""
from __future__ import annotations

import math

from amr.core.config import RobotConfig
from amr.core.geometry import wrap_angle
from amr.core.types import Pose2D, Twist2D
from amr.sim.world import World


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else (hi if value > hi else value)


class DiffDriveRobot:
    def __init__(self, cfg: RobotConfig, spawn: Pose2D):
        self.cfg = cfg
        self.pose = Pose2D(spawn.x, spawn.y, spawn.theta)
        self.vel = Twist2D(0.0, 0.0)
        self.collided = False

    def step(self, cmd: Twist2D, dt: float, world: World) -> None:
        cfg = self.cfg

        # 1. Clamp command to velocity limits.
        v_cmd = _clamp(cmd.v, -cfg.max_lin_vel, cfg.max_lin_vel)
        w_cmd = _clamp(cmd.omega, -cfg.max_ang_vel, cfg.max_ang_vel)

        # 2. Clamp the change from the current velocity by max accel * dt.
        dv_max = cfg.max_lin_acc * dt
        dw_max = cfg.max_ang_acc * dt
        v = self.vel.v + _clamp(v_cmd - self.vel.v, -dv_max, dv_max)
        w = self.vel.omega + _clamp(w_cmd - self.vel.omega, -dw_max, dw_max)

        # 3. Integrate the exact arc to a candidate pose.
        th = self.pose.theta
        if abs(w) < 1e-9:
            nx = self.pose.x + v * math.cos(th) * dt
            ny = self.pose.y + v * math.sin(th) * dt
            nth = wrap_angle(th + w * dt)
        else:
            R = v / w
            nth_raw = th + w * dt
            nx = self.pose.x + R * (math.sin(nth_raw) - math.sin(th))
            ny = self.pose.y - R * (math.cos(nth_raw) - math.cos(th))
            nth = wrap_angle(nth_raw)

        # 4. Collision: test the center plus 8 footprint-circle points.
        if self._collides(nx, ny, world):
            # Keep the old pose, zero velocity, latch collided for this step.
            self.vel = Twist2D(0.0, 0.0)
            self.collided = True
        else:
            self.pose = Pose2D(nx, ny, nth)
            self.vel = Twist2D(v, w)
            self.collided = False

    def _collides(self, x: float, y: float, world: World) -> bool:
        r = self.cfg.radius
        if world.is_occupied_world(x, y):
            return True
        for k in range(8):
            ang = 2.0 * math.pi * k / 8.0
            if world.is_occupied_world(x + r * math.cos(ang),
                                       y + r * math.sin(ang)):
                return True
        return False
