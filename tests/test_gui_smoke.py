import os

import numpy as np
import pytest

tk = pytest.importorskip("tkinter")

from amr.core.config import load_config
from amr.core.types import OccupancyGrid, Pose2D
from amr.gui.map_canvas import grid_to_photoimage
from amr.runtime.app import Mode, Snapshot

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"),
                                reason="no display available")


def _fake_snapshot(grid):
    return Snapshot(mode=Mode.SLAM, nav_state="MAPPING", pose=Pose2D(1, 1, 0),
                    gt_pose=Pose2D(1, 1, 0),
                    scan_points=np.array([[2.0, 2.0], [1.5, 0.5]]),
                    particles=None, path=None, goal=None, grid=grid,
                    map_version=1, sim_time=0.0, collided=False, estop=False,
                    status="smoke")


def test_render_pipeline_headless_window():
    from amr.gui.app import AmrGuiApp
    cfg = load_config()
    grid = OccupancyGrid(0.05, 0, 0,
                         np.zeros((60, 60), dtype=np.int8))
    grid.data[0, :] = 100
    gui = AmrGuiApp(cfg, start_worker=False)              # constructor must allow this
    gui.root.withdraw()
    img = grid_to_photoimage(grid, 2)
    assert img.width() == 120 and img.height() == 120     # 60 cells × zoom 2
    gui.render_snapshot(_fake_snapshot(grid))             # public for testability
    items = gui.canvas.find_all()
    assert len(items) >= 3                                # map image + robot + scan
    ps = gui.canvas.postscript()                          # proves real drawing happened
    assert len(ps) > 500
    gui.root.destroy()
