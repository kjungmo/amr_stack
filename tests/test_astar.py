import math

import numpy as np

from amr.core.config import AstarConfig, CostmapConfig
from amr.planning.astar import has_line_of_sight, plan_path
from amr.planning.costmap import Costmap
from amr.sim.world import World


def test_path_around_box(box_world):
    cm = Costmap(box_world.grid, CostmapConfig(), 0.18)
    path = plan_path(cm, (1.0, 1.0), (5.0, 5.0), AstarConfig())
    assert path is not None
    assert np.allclose(path[0], [1.0, 1.0]) and np.allclose(path[-1], [5.0, 5.0])
    for x, y in path:
        assert cm.cost_at_world(x, y) < 0.99
    length = np.sum(np.hypot(*np.diff(path, axis=0).T))
    assert length >= math.hypot(4, 4) - 0.01


def test_no_path_when_sealed():
    d = dict(size=[4.0, 4.0], resolution=0.05, spawn={"x": 1, "y": 1, "theta": 0},
             obstacles=[{"type": "rect", "x": 1.9, "y": 0.0, "w": 0.2, "h": 4.0}])
    w = World.from_dict(d)
    cm = Costmap(w.grid, CostmapConfig(unknown_is_lethal=False), 0.18)
    assert plan_path(cm, (1.0, 2.0), (3.0, 2.0), AstarConfig()) is None


def test_line_of_sight(box_world):
    cm = Costmap(box_world.grid, CostmapConfig(), 0.18)
    assert has_line_of_sight(cm, (1.0, 1.0), (1.0, 5.0))
    assert not has_line_of_sight(cm, (1.0, 3.0), (5.0, 3.0))   # crosses the box
