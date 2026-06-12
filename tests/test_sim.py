import math

import numpy as np
import pytest

from amr.core.config import RobotConfig
from amr.core.types import Pose2D, Twist2D
from amr.sim.lidar import Lidar
from amr.sim.robot import DiffDriveRobot
from amr.sim.simulator import Simulator


def test_world_raster(box_world):
    assert box_world.grid.data.shape == (120, 120)
    assert box_world.is_occupied_world(3.0, 3.0)          # center box
    assert not box_world.is_occupied_world(1.0, 1.0)
    assert box_world.is_occupied_world(-1.0, 3.0)         # out of bounds


def test_drive_straight(box_world, cfg):
    r = DiffDriveRobot(cfg.robot, Pose2D(1.0, 1.0, 0.0))
    for _ in range(40):                                   # 2.0 s
        r.step(Twist2D(0.5, 0.0), 0.05, box_world)
    # ramp-limited distance ≈ 0.5*2 - 0.5²/(2*0.8) = 0.844
    assert r.pose.x == pytest.approx(1.844, abs=0.03)
    assert r.pose.y == pytest.approx(1.0, abs=1e-6)


def test_arc_curvature(box_world):
    fast = RobotConfig(max_lin_acc=50.0, max_ang_acc=50.0)  # no ramp: exact circle
    r = DiffDriveRobot(fast, Pose2D(3.0, 1.0, 0.0))
    for _ in range(60):
        r.step(Twist2D(0.4, 0.8), 0.05, box_world)        # R = 0.5, center (3, 1.5)
    d = math.hypot(r.pose.x - 3.0, r.pose.y - 1.5)
    assert d == pytest.approx(0.5, abs=0.02)


def test_collision_stops_robot(box_world, cfg):
    r = DiffDriveRobot(cfg.robot, Pose2D(5.0, 3.9, 0.0))  # heading at east wall
    hit = False
    for _ in range(100):
        r.step(Twist2D(0.5, 0.0), 0.05, box_world)
        hit = hit or r.collided
    assert hit
    assert r.pose.x < 5.9 - cfg.robot.radius + 0.02       # never penetrates


def test_lidar_exact_ranges(box_world, cfg, rng):
    lidar = Lidar(cfg.lidar, rng)                          # noise_std = 0 via fixture
    scan = lidar.scan(box_world, Pose2D(1.0, 1.0, 0.0), 0.0)
    i_fwd = int(round((0.0 - scan.angle_min) / scan.angle_increment)) % scan.num_beams
    # forward beam (+x): wall inner face at x=5.9 → expected 4.9
    assert scan.ranges[i_fwd] == pytest.approx(4.9, abs=0.08)


def test_sim_odometry_drifts_but_tracks(box_world, cfg, rng):
    sim = Simulator(box_world, cfg, rng)
    for _ in range(100):
        res = sim.step(Twist2D(0.4, 0.3))
    gt, od = res.ground_truth, res.odom_pose
    err = math.hypot(gt.x - od.x, gt.y - od.y)
    assert 0.0 < err < 0.8
    assert res.sim_time == pytest.approx(5.0)
