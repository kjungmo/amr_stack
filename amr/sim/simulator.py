"""Top-level 2D simulator: advances the robot, emits noisy odometry and scans.

Each step integrates the ground-truth robot, derives a noisy odometry increment
(in the robot frame) and accumulates it into a drifting odom pose, and emits a
lidar scan every cfg.lidar.scan_every steps.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from amr.core.config import AmrConfig
from amr.core.geometry import pose_compose, wrap_angle
from amr.core.types import LaserScan, Pose2D, Twist2D
from amr.sim.lidar import Lidar
from amr.sim.robot import DiffDriveRobot
from amr.sim.world import World


@dataclass
class SimStepResult:
    ground_truth: Pose2D
    odom_pose: Pose2D
    odom_delta: Pose2D
    scan: Optional[LaserScan]
    collided: bool
    sim_time: float


def _arc_robot_frame(v: float, w: float, dt: float) -> Pose2D:
    """Exact-arc integration of (v, w) over dt, expressed in the robot frame
    (start pose at the origin, heading +x). Returns Pose2D(dx_fwd, dy_left, dth).
    """
    if abs(w) < 1e-9:
        return Pose2D(v * dt, 0.0, 0.0)
    R = v / w
    dth = w * dt
    dx = R * math.sin(dth)
    dy = R * (1.0 - math.cos(dth))
    return Pose2D(dx, dy, wrap_angle(dth))


class Simulator:
    def __init__(self, world: World, cfg: AmrConfig, rng: np.random.Generator):
        self.world = world
        self.cfg = cfg
        self.rng = rng
        self.time = 0.0
        self._steps = 0
        self.robot = DiffDriveRobot(cfg.robot, world.spawn)
        self.lidar = Lidar(cfg.lidar, rng)
        self.odom_pose = Pose2D(world.spawn.x, world.spawn.y, world.spawn.theta)

    def step(self, cmd: Twist2D) -> SimStepResult:
        dt = self.cfg.sim.dt

        # Advance ground truth.
        self.robot.step(cmd, dt, self.world)
        self._steps += 1
        self.time += dt

        # Noisy odometry from the actual (clamped) velocity.
        v = self.robot.vel.v
        w = self.robot.vel.omega
        nz = self.cfg.sim.odom_noise
        v_std = nz.alpha_v * abs(v) + nz.floor
        w_std = nz.alpha_w * abs(w) + nz.floor
        v_meas = v + self.rng.normal(0.0, v_std)
        w_meas = w + self.rng.normal(0.0, w_std)

        odom_delta = _arc_robot_frame(v_meas, w_meas, dt)
        self.odom_pose = pose_compose(self.odom_pose, odom_delta)

        # Scan every scan_every steps.
        scan = None
        scan_every = self.cfg.lidar.scan_every
        if scan_every > 0 and (self._steps % scan_every == 0):
            scan = self.lidar.scan(self.world, self.robot.pose, self.time)

        return SimStepResult(
            ground_truth=Pose2D(self.robot.pose.x, self.robot.pose.y,
                                self.robot.pose.theta),
            odom_pose=Pose2D(self.odom_pose.x, self.odom_pose.y,
                             self.odom_pose.theta),
            odom_delta=odom_delta,
            scan=scan,
            collided=self.robot.collided,
            sim_time=self.time,
        )
