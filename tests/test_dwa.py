import numpy as np

from amr.core.config import CostmapConfig, DwaConfig, RobotConfig
from amr.core.types import Pose2D, Twist2D
from amr.planning.costmap import Costmap
from amr.planning.dwa import DwaPlanner, carrot_point


def _cm(box_world):
    return Costmap(box_world.grid, CostmapConfig(), 0.18)


def test_carrot():
    path = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    c = carrot_point(path, Pose2D(0.2, 0.1, 0.0), 0.8)
    assert c[0] >= 0.8 and abs(c[1]) < 1e-6
    assert np.allclose(carrot_point(path, Pose2D(1.9, 0, 0), 0.8), [2.0, 0.0])


def test_drives_forward_when_clear(box_world):
    dwa = DwaPlanner(DwaConfig(), RobotConfig())
    path = np.array([[1.0, 1.0], [2.0, 1.0], [3.0, 1.0]])
    res = dwa.compute(Pose2D(1.0, 1.0, 0.0), Twist2D(0.3, 0.0), path, _cm(box_world))
    assert not res.blocked and res.cmd.v > 0.15
    for x, y, _ in res.trajectory:
        assert _cm(box_world).cost_at_world(x, y) < 0.99


def test_avoids_wall_ahead(box_world):
    dwa = DwaPlanner(DwaConfig(), RobotConfig())
    pose = Pose2D(5.0, 3.0, 0.0)                          # 0.9 m from east wall
    path = np.array([[5.0, 3.0], [5.0, 4.5]])             # path bends north
    res = dwa.compute(pose, Twist2D(0.4, 0.0), path, _cm(box_world))
    assert not res.blocked
    assert res.cmd.omega > 0.1                            # turns left toward the path
    for x, y, _ in res.trajectory:
        assert _cm(box_world).cost_at_world(x, y) < 0.99
