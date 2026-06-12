"""Log-odds occupancy grid mapper.

Update rule per plan §Task 1.2:
- For each beam in scan[::beam_subsample]:
  * Compute the endpoint in world coords via pose ⊕ (r·cosθ, r·sinθ, 0)
  * Run Bresenham from robot cell to endpoint cell
  * All cells on the ray (except the last) get log_odds += l_free
  * The endpoint cell gets log_odds += l_occ  only if the beam is a valid return
  * For no-return beams: trace free to range_max * 0.99, skip endpoint update
  * Clamp log_odds to ±l_clamp

to_occupancy_grid thresholds (plan §2 item 5):
  p = 1 - 1/(1 + exp(log_odds))
  p > occupied_thresh → 100
  p < free_thresh     → 0
  else                → -1  (unknown, or unobserved cells where log_odds == 0)
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from amr.core.config import MappingConfig
from amr.core.geometry import bresenham
from amr.core.types import LaserScan, OccupancyGrid, Pose2D


class OccupancyGridMapper:
    """Log-odds occupancy grid mapper."""

    log_odds: np.ndarray  # float32 (rows, cols), 0 = unobserved

    def __init__(self, cfg: MappingConfig,
                 size_m: Tuple[float, float],
                 origin_xy: Tuple[float, float] = (0.0, 0.0)) -> None:
        self._cfg = cfg
        self._resolution = cfg.resolution
        self._origin_x = float(origin_xy[0])
        self._origin_y = float(origin_xy[1])

        cols = int(round(size_m[0] / cfg.resolution))
        rows = int(round(size_m[1] / cfg.resolution))
        self.log_odds = np.zeros((rows, cols), dtype=np.float32)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        row = int(math.floor((y - self._origin_y) / self._resolution))
        col = int(math.floor((x - self._origin_x) / self._resolution))
        return row, col

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.log_odds.shape[0] and 0 <= col < self.log_odds.shape[1]

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def update(self, pose: Pose2D, scan: LaserScan) -> None:
        """Integrate one laser scan at the given pose into the log-odds map."""
        cfg = self._cfg
        robot_r, robot_c = self._world_to_cell(pose.x, pose.y)

        angles = scan.angles()
        valid = scan.valid_mask()

        # Subsample beams
        indices = range(0, scan.num_beams, cfg.beam_subsample)

        cos_th = math.cos(pose.theta)
        sin_th = math.sin(pose.theta)

        for i in indices:
            angle_b = angles[i]
            is_valid = bool(valid[i])

            if is_valid:
                r_dist = float(scan.ranges[i])
            else:
                # Trace free space up to range_max * 0.99
                r_dist = scan.range_max * 0.99

            # Beam direction in world frame
            beam_cos = math.cos(pose.theta + angle_b)
            beam_sin = math.sin(pose.theta + angle_b)

            # Endpoint in world coords
            end_x = pose.x + r_dist * beam_cos
            end_y = pose.y + r_dist * beam_sin
            end_r, end_c = self._world_to_cell(end_x, end_y)

            # Bresenham ray from robot cell to endpoint cell
            cells = bresenham(robot_r, robot_c, end_r, end_c)

            # Mark all cells except the last as free
            for cell_r, cell_c in cells[:-1]:
                if self._in_bounds(cell_r, cell_c):
                    self.log_odds[cell_r, cell_c] = np.clip(
                        self.log_odds[cell_r, cell_c] + cfg.l_free,
                        -cfg.l_clamp, cfg.l_clamp
                    )

            # Mark the endpoint cell as occupied (only for valid returns)
            if is_valid and self._in_bounds(end_r, end_c):
                self.log_odds[end_r, end_c] = np.clip(
                    self.log_odds[end_r, end_c] + cfg.l_occ,
                    -cfg.l_clamp, cfg.l_clamp
                )

    def to_occupancy_grid(self) -> OccupancyGrid:
        """Convert log-odds map to ROS-style OccupancyGrid.

        p = 1 - 1/(1 + exp(log_odds))
        p > occupied_thresh → 100
        p < free_thresh     → 0
        else                → -1
        """
        cfg = self._cfg
        # p_occ = 1 - 1/(1+exp(log_odds)) = exp(log_odds)/(1+exp(log_odds))
        # Equivalent: p = sigmoid(log_odds)
        # p = 0.5 when log_odds = 0 (unobserved)
        lo = self.log_odds.astype(np.float64)
        p = 1.0 - 1.0 / (1.0 + np.exp(lo))

        data = np.full(self.log_odds.shape, -1, dtype=np.int8)
        data[p > cfg.occupied_thresh] = 100
        data[p < cfg.free_thresh] = 0
        # Cells where log_odds == 0 exactly → p == 0.5 → unknown (-1)

        return OccupancyGrid(
            resolution=self._resolution,
            origin_x=self._origin_x,
            origin_y=self._origin_y,
            data=data,
        )
