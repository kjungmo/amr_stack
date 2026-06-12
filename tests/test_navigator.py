import math

import pytest

from amr.core.config import AstarConfig, CostmapConfig, DwaConfig, RobotConfig, NavConfig
from amr.core.types import Pose2D, Twist2D
from amr.navigation.navigator import Navigator, NavState
from amr.planning.costmap import Costmap
from amr.planning.dwa import DwaPlanner
from amr.sim.robot import DiffDriveRobot


def _nav():
    return Navigator(NavConfig(), AstarConfig(), DwaPlanner(DwaConfig(), RobotConfig()))


@pytest.mark.slow
def test_reaches_goal_in_box_world(box_world, cfg):
    cm = Costmap(box_world.grid, CostmapConfig(), cfg.robot.radius)
    nav = _nav()
    robot = DiffDriveRobot(cfg.robot, box_world.spawn)
    nav.set_goal(Pose2D(5.0, 5.0, 0.0))
    t = 0.0
    while t < 60.0 and nav.state not in (NavState.SUCCEEDED, NavState.FAILED):
        cmd = nav.update(robot.pose, robot.vel, cm, t)
        robot.step(cmd, 0.05, box_world)
        t += 0.05
    assert nav.state == NavState.SUCCEEDED
    assert math.hypot(robot.pose.x - 5.0, robot.pose.y - 5.0) < NavConfig().goal_tol_xy + 0.05
    assert not robot.collided


def test_unreachable_goal_fails_after_recoveries(box_world, cfg):
    cm = Costmap(box_world.grid, CostmapConfig(), cfg.robot.radius)
    nav = _nav()
    nav.set_goal(Pose2D(3.0, 3.0, 0.0))                   # inside the center box
    states = set()
    t = 0.0
    while t < 120.0 and nav.state != NavState.FAILED:
        nav.update(Pose2D(1.0, 1.0, 0.0), Twist2D(), cm, t)
        states.add(nav.state)
        t += 0.05
    assert nav.state == NavState.FAILED
    assert NavState.RECOVERY in states


def test_idle_and_cancel(box_world, cfg):
    cm = Costmap(box_world.grid, CostmapConfig(), cfg.robot.radius)
    nav = _nav()
    assert nav.update(Pose2D(1, 1, 0), Twist2D(), cm, 0.0) == Twist2D(0.0, 0.0)
    nav.set_goal(Pose2D(5, 5, 0))
    nav.cancel()
    assert nav.state == NavState.IDLE and nav.goal is None
