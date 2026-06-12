import math

import numpy as np
import pytest

from amr.core.geometry import (bresenham, distance_field, pose_between,
                               pose_compose, transform_points, wrap_angle)
from amr.core.types import Pose2D


def test_wrap_angle():
    assert wrap_angle(0.0) == 0.0
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)      # (-pi, pi]
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3.5 * math.pi) == pytest.approx(-0.5 * math.pi)


def test_compose_between_roundtrip():
    a = Pose2D(1.0, 2.0, 0.7)
    b = Pose2D(-0.5, 3.0, -2.0)
    d = pose_between(a, b)
    c = pose_compose(a, d)
    assert (c.x, c.y) == pytest.approx((b.x, b.y), abs=1e-9)
    assert wrap_angle(c.theta - b.theta) == pytest.approx(0.0, abs=1e-9)


def test_transform_points():
    pts = np.array([[1.0, 0.0], [0.0, 1.0]])
    out = transform_points(Pose2D(2.0, 1.0, math.pi / 2), pts)
    assert out[0] == pytest.approx([2.0, 2.0])   # forward point ends up +y
    assert out[1] == pytest.approx([1.0, 1.0])


def test_bresenham():
    assert bresenham(0, 0, 0, 3) == [(0, 0), (0, 1), (0, 2), (0, 3)]
    assert bresenham(0, 0, 3, 3) == [(0, 0), (1, 1), (2, 2), (3, 3)]
    cells = bresenham(0, 0, 1, 3)
    assert cells[0] == (0, 0) and cells[-1] == (1, 3) and len(cells) == 4
    assert bresenham(2, 5, 2, 5) == [(2, 5)]


def test_distance_field():
    occ = np.zeros((11, 11), dtype=bool)
    occ[5, 5] = True
    d = distance_field(occ, resolution=0.1, max_dist=0.5)
    assert d[5, 5] == 0.0
    assert d[5, 8] == pytest.approx(0.3, abs=0.01)        # 3 cells straight
    assert d[8, 8] == pytest.approx(0.3 * math.sqrt(2), abs=0.05)
    assert d[0, 0] == pytest.approx(0.5)                  # capped at max_dist
