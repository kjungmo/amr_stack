"""Inflated costmap for global/local planning.

Builds a float cost grid in [0, 1] from an OccupancyGrid. Obstacle cells (and,
optionally, unknown cells) are lethal; the cost decays outward over an inflation
band. Because the inflation radius already accounts for the robot radius, all
downstream collision checks treat the robot as a point.
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from amr.core.config import CostmapConfig
from amr.core.geometry import distance_field
from amr.core.types import OccupancyGrid


class Costmap:
    LETHAL = 1.0

    def __init__(self, grid: OccupancyGrid, cfg: CostmapConfig,
                 robot_radius: float):
        self.resolution = grid.resolution
        self.origin_x = grid.origin_x
        self.origin_y = grid.origin_y
        self._rows = grid.rows
        self._cols = grid.cols
        self.cfg = cfg
        self.robot_radius = float(robot_radius)

        data = grid.data
        mask = data >= cfg.occupied_thresh
        if cfg.unknown_is_lethal:
            mask = mask | (data < 0)

        # Use max_dist slightly beyond the inflation radius so genuinely-distant
        # cells land strictly outside the band (cost 0) instead of saturating at
        # exactly inflation_radius, which would be indistinguishable from a cell
        # truly on the band's outer edge.
        d = distance_field(mask, grid.resolution,
                           cfg.inflation_radius + grid.resolution)
        # distance_field reports the distance to the nearest occupied *cell
        # center*; the true distance from an arbitrary point to the obstacle is
        # up to half a cell smaller. Shift by half a cell so the lethal/decay
        # bands hug the actual obstacle boundary.
        d = np.maximum(d - 0.5 * grid.resolution, 0.0)

        rr = float(robot_radius)
        infl = float(cfg.inflation_radius)
        cost = np.zeros(d.shape, dtype=np.float32)
        # Lethal core: within (inflated) robot radius of an obstacle.
        cost[d <= rr] = 1.0
        # Decay band: robot_radius < d < inflation_radius.
        band = (d > rr) & (d < infl)
        cost[band] = np.exp(-cfg.cost_decay * (d[band] - rr)).astype(np.float32)
        # Beyond the inflation radius cost stays 0.0.
        self.cost = cost.astype(np.float32)

    # --- grid <-> world helpers (same semantics as OccupancyGrid) ---
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self._rows and 0 <= col < self._cols

    @property
    def rows(self) -> int:
        return self._rows

    @property
    def cols(self) -> int:
        return self._cols

    # --- cost queries ---
    def is_lethal(self, row: int, col: int) -> bool:
        if not self.in_bounds(row, col):
            return True
        return bool(self.cost[row, col] >= 0.99)

    def cost_at_world(self, x: float, y: float) -> float:
        row, col = self.world_to_grid(x, y)
        if not self.in_bounds(row, col):
            return 1.0
        return float(self.cost[row, col])
