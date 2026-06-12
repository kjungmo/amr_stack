"""Geometry primitives shared by all subsystems."""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from amr.core.types import Pose2D


def wrap_angle(a: float) -> float:
    """Wrap a scalar angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def wrap_angles(a: np.ndarray) -> np.ndarray:
    """Vector wrap to [-pi, pi) — note the half-open end differs from wrap_angle."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def pose_compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """a ⊕ b: pose of frame b (expressed in a) in a's parent frame."""
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose2D(a.x + c * b.x - s * b.y,
                  a.y + s * b.x + c * b.y,
                  wrap_angle(a.theta + b.theta))


def pose_between(a: Pose2D, b: Pose2D) -> Pose2D:
    """a ⊖ b: pose of b expressed in frame a, so pose_compose(a, result) == b."""
    dx, dy = b.x - a.x, b.y - a.y
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose2D(c * dx + s * dy, -s * dx + c * dy, wrap_angle(b.theta - a.theta))


def transform_points(pose: Pose2D, pts: np.ndarray) -> np.ndarray:
    """Transform (N, 2) points from pose's frame into the world frame."""
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T + np.array([pose.x, pose.y])


def bresenham(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
    """All integer grid cells on the line from (r0,c0) to (r1,c1), inclusive."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells


def distance_field(occupied: np.ndarray, resolution: float,
                   max_dist: float) -> np.ndarray:
    """Chamfer distance (m) from each cell to the nearest occupied cell.

    Iterative 8-neighbour relaxation (Bellman-Ford style), fully vectorized;
    converges in <= ceil(max_dist/resolution) sweeps. No scipy required.
    """
    big = max_dist / resolution + 2.0
    d = np.where(occupied, 0.0, big)
    sq2 = math.sqrt(2.0)
    for _ in range(int(math.ceil(max_dist / resolution)) + 1):
        nd = d.copy()
        nd[1:, :] = np.minimum(nd[1:, :], d[:-1, :] + 1.0)
        nd[:-1, :] = np.minimum(nd[:-1, :], d[1:, :] + 1.0)
        nd[:, 1:] = np.minimum(nd[:, 1:], d[:, :-1] + 1.0)
        nd[:, :-1] = np.minimum(nd[:, :-1], d[:, 1:] + 1.0)
        nd[1:, 1:] = np.minimum(nd[1:, 1:], d[:-1, :-1] + sq2)
        nd[1:, :-1] = np.minimum(nd[1:, :-1], d[:-1, 1:] + sq2)
        nd[:-1, 1:] = np.minimum(nd[:-1, 1:], d[1:, :-1] + sq2)
        nd[:-1, :-1] = np.minimum(nd[:-1, :-1], d[1:, 1:] + sq2)
        if np.array_equal(nd, d):
            break
        d = nd
    return np.minimum(d * resolution, max_dist).astype(np.float32)
