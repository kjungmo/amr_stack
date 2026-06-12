"""Shared AMR data types. See plan §2 for the frozen conventions."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # rad, CCW, wrapped to (-pi, pi]

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)

    @staticmethod
    def from_array(a) -> "Pose2D":
        return Pose2D(float(a[0]), float(a[1]), float(a[2]))


@dataclass
class Twist2D:
    v: float = 0.0      # m/s forward
    omega: float = 0.0  # rad/s CCW


@dataclass
class LaserScan:
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float            # no-return is encoded as range_max
    ranges: np.ndarray          # (N,) float
    stamp: float = 0.0          # sim time, s

    @property
    def num_beams(self) -> int:
        return int(self.ranges.shape[0])

    def angles(self) -> np.ndarray:
        return self.angle_min + self.angle_increment * np.arange(self.num_beams)

    def valid_mask(self) -> np.ndarray:
        return (self.ranges > self.range_min) & (self.ranges < self.range_max * 0.999)


@dataclass
class OccupancyGrid:
    resolution: float   # m/cell
    origin_x: float     # world x of the outer corner of cell (row=0, col=0)
    origin_y: float
    data: np.ndarray    # int8 (rows, cols): -1 unknown, 0 free, 100 occupied

    @property
    def rows(self) -> int:
        return int(self.data.shape[0])

    @property
    def cols(self) -> int:
        return int(self.data.shape[1])

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    def occupied_mask(self, thresh: int = 65) -> np.ndarray:
        return self.data >= thresh

    def copy(self) -> "OccupancyGrid":
        return OccupancyGrid(self.resolution, self.origin_x, self.origin_y,
                             self.data.copy())
