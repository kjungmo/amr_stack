"""2D ground-truth world: rasterized occupancy grid built from obstacle geometry.

The world grid is the simulator's ground truth (0 free, 100 occupied, never -1).
Origin is fixed at (0, 0); cell (row, col) center is at
((col + 0.5) * res, (row + 0.5) * res).
"""
from __future__ import annotations

from typing import Tuple

import numpy as np
import yaml

from amr.core.types import OccupancyGrid, Pose2D


class World:
    def __init__(self, grid: OccupancyGrid, spawn: Pose2D,
                 size: Tuple[float, float]):
        self.grid = grid
        self.spawn = spawn
        self.size = size

    @staticmethod
    def from_yaml(path: str) -> "World":
        with open(path) as f:
            d = yaml.safe_load(f)
        return World.from_dict(d)

    @staticmethod
    def from_dict(d: dict) -> "World":
        size_x, size_y = float(d["size"][0]), float(d["size"][1])
        res = float(d["resolution"])
        rows = int(round(size_y / res))
        cols = int(round(size_x / res))

        data = np.zeros((rows, cols), dtype=np.int8)

        # Cell-center world coordinates, vectorized via meshgrid.
        xs = (np.arange(cols) + 0.5) * res            # (cols,) world x per column
        ys = (np.arange(rows) + 0.5) * res            # (rows,) world y per row
        cx, cy = np.meshgrid(xs, ys)                  # (rows, cols)

        occ = np.zeros((rows, cols), dtype=bool)
        for obs in d.get("obstacles", []) or []:
            otype = obs["type"]
            if otype == "rect":
                ox, oy = float(obs["x"]), float(obs["y"])
                ow, oh = float(obs["w"]), float(obs["h"])
                mask = ((cx >= ox) & (cx < ox + ow) &
                        (cy >= oy) & (cy < oy + oh))
                occ |= mask
            elif otype == "circle":
                ox, oy = float(obs["x"]), float(obs["y"])
                r = float(obs["r"])
                mask = ((cx - ox) ** 2 + (cy - oy) ** 2) <= r * r
                occ |= mask
            else:
                raise ValueError("unknown obstacle type %r" % otype)

        data[occ] = 100
        grid = OccupancyGrid(res, 0.0, 0.0, data)

        sp = d["spawn"]
        spawn = Pose2D(float(sp["x"]), float(sp["y"]), float(sp.get("theta", 0.0)))
        return World(grid, spawn, (size_x, size_y))

    def is_occupied_world(self, x: float, y: float) -> bool:
        """True if (x, y) is occupied; out of bounds counts as occupied."""
        row, col = self.grid.world_to_grid(x, y)
        if not self.grid.in_bounds(row, col):
            return True
        return bool(self.grid.data[row, col] >= 50)
