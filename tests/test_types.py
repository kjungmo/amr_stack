import numpy as np
import pytest

from amr.core.types import LaserScan, OccupancyGrid


def test_grid_world_roundtrip():
    g = OccupancyGrid(resolution=0.5, origin_x=-1.0, origin_y=2.0,
                      data=np.zeros((4, 6), dtype=np.int8))
    assert g.rows == 4 and g.cols == 6
    r, c = g.world_to_grid(0.6, 3.2)
    assert (r, c) == (2, 3)
    x, y = g.grid_to_world(2, 3)
    assert (x, y) == pytest.approx((0.75, 3.25))          # cell center
    assert g.world_to_grid(x, y) == (2, 3)
    assert g.in_bounds(0, 0) and not g.in_bounds(4, 0) and not g.in_bounds(-1, 2)


def test_scan_helpers():
    s = LaserScan(angle_min=-1.0, angle_increment=0.5, range_min=0.1,
                  range_max=5.0, ranges=np.array([0.05, 2.0, 5.0, 4.9]))
    assert s.num_beams == 4
    assert s.angles() == pytest.approx([-1.0, -0.5, 0.0, 0.5])
    assert list(s.valid_mask()) == [False, True, False, True]
