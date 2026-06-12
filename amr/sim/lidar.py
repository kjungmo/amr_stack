"""Vectorized 2D lidar that ray-casts against the world occupancy grid.

The whole (B, S) sample grid (B beams x S range samples) is built and converted
to integer cells in one shot; the first occupied sample per beam is the range.
A per-beam Python loop would be too slow for the performance budget.
"""
from __future__ import annotations

import numpy as np

from amr.core.config import LidarConfig
from amr.core.types import LaserScan, Pose2D
from amr.sim.world import World


class Lidar:
    def __init__(self, cfg: LidarConfig, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng

    def scan(self, world: World, pose: Pose2D, stamp: float) -> LaserScan:
        cfg = self.cfg
        grid = world.grid
        res = grid.resolution
        B = int(cfg.num_beams)
        increment = (cfg.angle_max - cfg.angle_min) / B

        # Beam angles in the world frame.
        beam_angles = cfg.angle_min + increment * np.arange(B)          # (B,)
        world_angles = pose.theta + beam_angles                        # (B,)
        cosw = np.cos(world_angles)                                    # (B,)
        sinw = np.sin(world_angles)

        # Range samples along each beam.
        S = np.arange(cfg.range_min, cfg.range_max, res * 0.5)         # (Snum,)
        if S.size == 0:
            S = np.array([cfg.range_min])

        # (B, Snum) sample world coordinates via broadcasting.
        sx = pose.x + cosw[:, None] * S[None, :]                      # (B, Snum)
        sy = pose.y + sinw[:, None] * S[None, :]

        # Convert all samples to integer grid cells at once.
        rows = np.floor((sy - grid.origin_y) / res).astype(np.int64)
        cols = np.floor((sx - grid.origin_x) / res).astype(np.int64)

        in_bounds = ((rows >= 0) & (rows < grid.rows) &
                     (cols >= 0) & (cols < grid.cols))

        occ = grid.data >= 50
        # Clip indices so out-of-bounds lookups don't crash; mark OOB as a hit.
        rc = np.clip(rows, 0, grid.rows - 1)
        cc = np.clip(cols, 0, grid.cols - 1)
        hit = occ[rc, cc] & in_bounds        # (B, Snum) occupied & in bounds
        hit |= ~in_bounds                     # out-of-bounds counts as a hit

        any_hit = hit.any(axis=1)             # (B,)
        first = np.argmax(hit, axis=1)        # (B,) index of first True per beam

        ranges = np.where(any_hit, S[first], cfg.range_max).astype(float)

        if cfg.noise_std > 0.0:
            ranges = ranges + self.rng.normal(0.0, cfg.noise_std, size=B)

        ranges = np.clip(ranges, cfg.range_min, cfg.range_max)

        return LaserScan(
            angle_min=cfg.angle_min,
            angle_increment=increment,
            range_min=cfg.range_min,
            range_max=cfg.range_max,
            ranges=ranges,
            stamp=stamp,
        )
