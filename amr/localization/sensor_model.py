"""Likelihood-field range-finder sensor model.

Precomputes a distance field (metres to the nearest occupied cell) over the
map. For each particle, the valid beam endpoints are transformed into the world
frame, looked up in the distance field, and scored with the per-beam likelihood
``q = z_hit * exp(-d^2 / (2 sigma^2)) + z_rand / range_max``. The particle
weight is ``exp(sum_b ln q)``. Fully vectorized over particles and beams.
See Probabilistic Robotics (Thrun et al.) Table 6.3.
"""
from __future__ import annotations

import numpy as np

from amr.core.config import LikelihoodConfig
from amr.core.geometry import distance_field
from amr.core.types import LaserScan, OccupancyGrid


class LikelihoodField:
    def __init__(self, grid: OccupancyGrid, cfg: LikelihoodConfig):
        self.grid = grid
        self.cfg = cfg
        occupied = grid.data >= 65
        self.field = distance_field(occupied, grid.resolution, cfg.max_dist)
        # Cells that are unknown in the source map are treated as max_dist so
        # they contribute no informative hit likelihood.
        self.unknown = grid.data < 0

    def weigh(self, particles: np.ndarray, scan: LaserScan) -> np.ndarray:
        """(N,) unnormalized weights."""
        particles = np.asarray(particles, dtype=float)
        n = particles.shape[0]
        cfg = self.cfg
        grid = self.grid
        res = grid.resolution
        max_dist = cfg.max_dist
        sigma = cfg.sigma_hit

        valid = scan.valid_mask()
        sub = max(int(cfg.beam_subsample), 1)
        angles = scan.angles()
        ranges = scan.ranges

        a = angles[valid][::sub]
        r = ranges[valid][::sub]

        if a.size == 0:
            return np.ones(n, dtype=float)

        # Beam endpoints in the robot frame: (B, 2).
        bx = r * np.cos(a)
        by = r * np.sin(a)

        th = particles[:, 2]                       # (N,)
        cos_t = np.cos(th)[:, None]                # (N, 1)
        sin_t = np.sin(th)[:, None]
        px = particles[:, 0][:, None]              # (N, 1)
        py = particles[:, 1][:, None]

        # Broadcast-transform every beam endpoint by every particle: (N, B).
        wx = px + cos_t * bx[None, :] - sin_t * by[None, :]
        wy = py + sin_t * bx[None, :] + cos_t * by[None, :]

        cols = np.floor((wx - grid.origin_x) / res).astype(np.int64)
        rows = np.floor((wy - grid.origin_y) / res).astype(np.int64)

        in_bounds = ((rows >= 0) & (rows < grid.rows) &
                     (cols >= 0) & (cols < grid.cols))

        rc = np.clip(rows, 0, grid.rows - 1)
        cc = np.clip(cols, 0, grid.cols - 1)

        d = self.field[rc, cc].astype(float)       # (N, B)
        unknown = self.unknown[rc, cc]
        # Out-of-bounds or unknown cells contribute the max distance.
        d = np.where(in_bounds & ~unknown, d, max_dist)

        q = (cfg.z_hit * np.exp(-(d * d) / (2.0 * sigma * sigma)) +
             cfg.z_rand / scan.range_max)
        q = np.maximum(q, 1e-300)

        w = np.exp(np.sum(np.log(q), axis=1))
        return w
