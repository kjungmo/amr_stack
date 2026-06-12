"""Monte-Carlo localization (adaptive particle filter / AMCL-lite).

Maintains a cloud of weighted pose hypotheses. ``predict`` pushes every
particle through the Thrun odometry motion model; ``correct`` reweights the
cloud with the likelihood-field sensor model, renormalizes (with an underflow
guard), and resamples via a low-variance sampler whenever the effective sample
size drops below ``resample_neff_frac * N``. ``estimate`` returns the weighted
mean pose (circular mean for theta). See Probabilistic Robotics (Thrun et al.)
Table 8.2 / 4.4.
"""
from __future__ import annotations

import numpy as np

from amr.core.config import LocalizationConfig
from amr.core.types import LaserScan, OccupancyGrid, Pose2D
from amr.localization.motion_model import sample_motion
from amr.localization.sensor_model import LikelihoodField


def _low_variance_resample(particles, weights, rng):
    n = len(weights)
    positions = (rng.random() + np.arange(n)) / n
    idx = np.searchsorted(np.cumsum(weights), positions)
    return particles[np.minimum(idx, n - 1)].copy()


class MonteCarloLocalizer:
    def __init__(self, grid: OccupancyGrid, cfg: LocalizationConfig,
                 rng: np.random.Generator,
                 initial_pose=None):
        self.grid = grid
        self.cfg = cfg
        self.rng = rng
        self.n = int(cfg.num_particles)
        self.field = LikelihoodField(grid, cfg.likelihood)

        if initial_pose is None:
            self.particles = self._global_init()
        else:
            self.particles = self._gaussian_init(initial_pose)
        self.weights = np.full(self.n, 1.0 / self.n, dtype=float)

    def _global_init(self) -> np.ndarray:
        """Sample uniformly over free cells (data == 0); theta uniform."""
        free_rows, free_cols = np.nonzero(self.grid.data == 0)
        if free_rows.size == 0:
            # Degenerate map with no free cells: fall back to map extent.
            free_rows = np.array([self.grid.rows // 2])
            free_cols = np.array([self.grid.cols // 2])
        res = self.grid.resolution
        pick = self.rng.integers(0, free_rows.size, self.n)
        rows = free_rows[pick]
        cols = free_cols[pick]
        # Random offset within the chosen cell.
        x = self.grid.origin_x + (cols + self.rng.random(self.n)) * res
        y = self.grid.origin_y + (rows + self.rng.random(self.n)) * res
        th = self.rng.uniform(-np.pi, np.pi, self.n)
        return np.column_stack((x, y, th)).astype(float)

    def _gaussian_init(self, pose: Pose2D) -> np.ndarray:
        std = self.cfg.init_std
        sx, sy, sth = float(std[0]), float(std[1]), float(std[2])
        x = float(pose.x) + self.rng.normal(0.0, sx, self.n)
        y = float(pose.y) + self.rng.normal(0.0, sy, self.n)
        th = float(pose.theta) + self.rng.normal(0.0, sth, self.n)
        return np.column_stack((x, y, th)).astype(float)

    def predict(self, odom_delta: Pose2D) -> None:
        self.particles = sample_motion(self.particles, odom_delta,
                                       self.cfg.alphas, self.rng)

    def correct(self, scan: LaserScan) -> None:
        w = self.weights * self.field.weigh(self.particles, scan)
        total = w.sum()
        if not np.isfinite(total) or total <= 0.0:
            # Underflow / all-zero guard: reset to a uniform cloud.
            w = np.full(self.n, 1.0 / self.n, dtype=float)
        else:
            w = w / total
        self.weights = w

        neff = 1.0 / np.sum(self.weights ** 2)
        if neff < self.cfg.resample_neff_frac * self.n:
            self.particles = _low_variance_resample(self.particles,
                                                    self.weights, self.rng)
            self.weights = np.full(self.n, 1.0 / self.n, dtype=float)

    def estimate(self) -> Pose2D:
        w = self.weights
        x = float(np.sum(w * self.particles[:, 0]))
        y = float(np.sum(w * self.particles[:, 1]))
        th = self.particles[:, 2]
        s = float(np.sum(w * np.sin(th)))
        c = float(np.sum(w * np.cos(th)))
        theta = float(np.arctan2(s, c))
        return Pose2D(x, y, theta)

    def set_pose(self, pose: Pose2D) -> None:
        self.particles = self._gaussian_init(pose)
        self.weights = np.full(self.n, 1.0 / self.n, dtype=float)
