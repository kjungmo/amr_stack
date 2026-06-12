"""Correlative scan-matching SLAM (plan §Task 2.1).

A lightweight 2D SLAM front-end that fuses noisy odometry increments with a
correlative scan match against a blurred occupancy *score map* derived from the
log-odds map maintained by :class:`OccupancyGridMapper`.

Pipeline per :meth:`process` call:

1. Dead-reckon the estimate forward with the odometry increment
   (``pose = pose_compose(pose, odom_delta)``). If there is no scan, return.
2. The very first scan seeds the map and the keyframe bookkeeping, no matching.
3. Skip matching if the motion since the last *processed* scan is below
   ``min_motion`` (both translation and |rotation|).
4. Build a blurred occupancy score map: ``p_occ = sigmoid(log_odds)``,
   ``score_src = (p_occ > 0.6)`` as float, blurred with a separable Gaussian
   (sigma = ``blur_sigma_cells``, half-width 3 sigma) implemented purely with
   shifted-slice sums (no scipy), peak-normalized to 1. Rebuilt only after a
   keyframe map update.
5. Two-stage correlative search (coarse then fine) over candidate
   ``(dx, dy, dtheta)`` offsets around the predicted pose, scoring each candidate
   by the mean score-map value at the scan endpoints. Fully vectorized per theta.
6. Accept the best candidate iff its score >= ``min_match_score`` (otherwise the
   early, thin map is not trustworthy — keep the odometry prediction).
7. On keyframe motion (>= ``keyframe_trans`` or ``keyframe_rot`` since the last
   keyframe), integrate the scan into the map and rebuild the score map.

All stochastic-free; the only randomness lives upstream in the simulator.
Coordinate conventions follow plan §2.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from amr.core.config import MappingConfig, SlamConfig
from amr.core.geometry import pose_compose, wrap_angle
from amr.core.types import LaserScan, OccupancyGrid, Pose2D
from amr.mapping.occupancy_grid_mapper import OccupancyGridMapper


class ScanMatchingSlam:
    """Correlative scan-matching SLAM with keyframed log-odds mapping."""

    pose: Pose2D
    mapper: OccupancyGridMapper

    def __init__(self, cfg: SlamConfig, mapping_cfg: MappingConfig,
                 size_m: Tuple[float, float], initial_pose: Pose2D) -> None:
        self._cfg = cfg
        self._mapping_cfg = mapping_cfg
        self._resolution = float(mapping_cfg.resolution)

        self.pose = Pose2D(initial_pose.x, initial_pose.y, initial_pose.theta)
        self.mapper = OccupancyGridMapper(mapping_cfg, size_m, origin_xy=(0.0, 0.0))

        self._origin_x = 0.0
        self._origin_y = 0.0

        # Keyframe / motion bookkeeping.
        self._have_first = False
        self._last_kf_pose = Pose2D(initial_pose.x, initial_pose.y,
                                    initial_pose.theta)
        self._last_proc_pose = Pose2D(initial_pose.x, initial_pose.y,
                                      initial_pose.theta)

        # Blurred occupancy score map (rebuilt on keyframe updates).
        self._score_map = None  # type: Optional[np.ndarray]

        # Precompute the separable Gaussian blur kernel.
        self._blur_kernel = self._gaussian_kernel(cfg.blur_sigma_cells)

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def process(self, odom_delta: Pose2D, scan: Optional[LaserScan]) -> Pose2D:
        """Fuse one odometry increment and (optionally) one scan; return pose."""
        # 1. Dead-reckon the estimate forward.
        self.pose = pose_compose(self.pose, odom_delta)

        if scan is None:
            return self.pose

        # 2. First scan ever: seed the map, remember keyframe state, return.
        if not self._have_first:
            self.mapper.update(self.pose, scan)
            self._rebuild_score_map()
            self._last_kf_pose = self._clone(self.pose)
            self._last_proc_pose = self._clone(self.pose)
            self._have_first = True
            return self.pose

        # 3. Skip matching if displacement since last processed scan is tiny.
        d_trans, d_rot = self._motion_since(self._last_proc_pose)
        if d_trans < self._cfg.min_motion and d_rot < self._cfg.min_motion:
            return self.pose

        # 4/5/6. Correlative scan match (score map already current).
        pts = self._scan_points(scan)
        if pts.shape[0] > 0 and self._score_map is not None:
            matched = self._match(pts)
            if matched is not None:
                self.pose = matched

        self._last_proc_pose = self._clone(self.pose)

        # 7. Keyframe map update on sufficient motion.
        kf_trans, kf_rot = self._motion_since(self._last_kf_pose)
        if kf_trans >= self._cfg.keyframe_trans or kf_rot >= self._cfg.keyframe_rot:
            self.mapper.update(self.pose, scan)
            self._rebuild_score_map()
            self._last_kf_pose = self._clone(self.pose)

        return self.pose

    def get_map(self) -> OccupancyGrid:
        return self.mapper.to_occupancy_grid()

    # ------------------------------------------------------------------
    # Score map construction
    # ------------------------------------------------------------------

    def _rebuild_score_map(self) -> None:
        """Rebuild the blurred occupancy score map from the log-odds map."""
        log_odds = self.mapper.log_odds.astype(np.float64)
        p_occ = 1.0 - 1.0 / (1.0 + np.exp(log_odds))
        score_src = (p_occ > 0.6).astype(np.float64)

        blurred = self._separable_blur(score_src, self._blur_kernel)
        peak = blurred.max()
        if peak > 0.0:
            blurred = blurred / peak
        self._score_map = blurred

    @staticmethod
    def _gaussian_kernel(sigma: float) -> np.ndarray:
        """1D Gaussian kernel, half-width 3 sigma, normalized to sum 1."""
        sigma = max(float(sigma), 1e-6)
        half = max(int(math.ceil(3.0 * sigma)), 1)
        offsets = np.arange(-half, half + 1, dtype=np.float64)
        k = np.exp(-(offsets ** 2) / (2.0 * sigma * sigma))
        k /= k.sum()
        return k

    @staticmethod
    def _separable_blur(src: np.ndarray, kernel: np.ndarray) -> np.ndarray:
        """Separable convolution with `kernel` along both axes via shifted slices.

        Implemented as a sum of weighted, zero-padded shifted slices (no scipy,
        no np.convolve over the 2D array). Boundary = zero padding.
        """
        half = (kernel.shape[0] - 1) // 2

        # Pass 1: blur along columns (axis=1, the x axis).
        tmp = np.zeros_like(src)
        for k, w in enumerate(kernel):
            shift = k - half  # negative => take from the right, place on the left
            if shift == 0:
                tmp += w * src
            elif shift > 0:
                tmp[:, shift:] += w * src[:, :-shift]
            else:
                s = -shift
                tmp[:, :-s] += w * src[:, s:]

        # Pass 2: blur along rows (axis=0, the y axis).
        out = np.zeros_like(tmp)
        for k, w in enumerate(kernel):
            shift = k - half
            if shift == 0:
                out += w * tmp
            elif shift > 0:
                out[shift:, :] += w * tmp[:-shift, :]
            else:
                s = -shift
                out[:-s, :] += w * tmp[s:, :]

        return out

    # ------------------------------------------------------------------
    # Correlative scan matching
    # ------------------------------------------------------------------

    def _scan_points(self, scan: LaserScan) -> np.ndarray:
        """`match_beams` evenly-subsampled valid beams as (B, 2) robot-frame points."""
        angles = scan.angles()
        valid = scan.valid_mask()
        ranges = np.asarray(scan.ranges, dtype=np.float64)

        valid_idx = np.nonzero(valid)[0]
        if valid_idx.shape[0] == 0:
            return np.zeros((0, 2), dtype=np.float64)

        n_want = int(self._cfg.match_beams)
        if n_want > 0 and valid_idx.shape[0] > n_want:
            sel = np.linspace(0, valid_idx.shape[0] - 1, n_want)
            sel = np.unique(np.round(sel).astype(int))
            valid_idx = valid_idx[sel]

        a = angles[valid_idx]
        r = ranges[valid_idx]
        # Robot frame: x forward, y left; beam angle 0 = forward.
        px = r * np.cos(a)
        py = r * np.sin(a)
        return np.column_stack((px, py))

    def _match(self, pts: np.ndarray) -> Optional[Pose2D]:
        """Two-stage correlative search; return the best pose or None to keep odom."""
        cfg = self._cfg
        base = self.pose

        # Coarse stage centered on the predicted pose.
        c_best, c_score = self._search(
            pts, base,
            cfg.coarse_window_xy, cfg.coarse_step_xy,
            cfg.coarse_window_theta, cfg.coarse_step_theta,
        )

        # Fine stage centered on the coarse winner.
        f_best, f_score = self._search(
            pts, c_best,
            cfg.coarse_step_xy, cfg.fine_step_xy,
            cfg.coarse_step_theta, cfg.fine_step_theta,
        )

        if f_score >= c_score:
            best, best_score = f_best, f_score
        else:
            best, best_score = c_best, c_score

        if best_score >= cfg.min_match_score:
            return best
        return None

    def _search(self, pts: np.ndarray, center: Pose2D,
                window_xy: float, step_xy: float,
                window_theta: float, step_theta: float
                ) -> Tuple[Pose2D, float]:
        """Vectorized correlative search over a (dx, dy, dtheta) grid.

        Returns the best candidate pose and its mean score.
        """
        score_map = self._score_map
        rows, cols = score_map.shape
        res = self._resolution
        inv_res = 1.0 / res
        ox, oy = self._origin_x, self._origin_y

        offs_xy = self._linrange(window_xy, step_xy)
        offs_th = self._linrange(window_theta, step_theta)

        # Translation grid (Kxy, 2) relative to the center.
        dxx, dyy = np.meshgrid(offs_xy, offs_xy)
        trans = np.column_stack((dxx.ravel(), dyy.ravel()))  # (Kxy, 2)
        cand_x = center.x + trans[:, 0]                       # (Kxy,)
        cand_y = center.y + trans[:, 1]

        B = pts.shape[0]
        best_score = -1.0
        best_dx = 0.0
        best_dy = 0.0
        best_theta = center.theta

        for dth in offs_th:
            theta = center.theta + dth
            c, s = math.cos(theta), math.sin(theta)
            # Rotate robot-frame points into a heading-aligned world offset (B, 2).
            rx = pts[:, 0] * c - pts[:, 1] * s
            ry = pts[:, 0] * s + pts[:, 1] * c

            # Endpoints (Kxy, B): candidate position + rotated point.
            ex = cand_x[:, None] + rx[None, :]
            ey = cand_y[:, None] + ry[None, :]

            cc = np.floor((ex - ox) * inv_res).astype(np.int64)
            cr = np.floor((ey - oy) * inv_res).astype(np.int64)

            in_b = (cr >= 0) & (cr < rows) & (cc >= 0) & (cc < cols)
            crc = np.where(in_b, cr, 0)
            ccc = np.where(in_b, cc, 0)
            vals = score_map[crc, ccc] * in_b  # out-of-map -> 0

            scores = vals.sum(axis=1) / B      # mean over beams (Kxy,)

            k = int(np.argmax(scores))
            if scores[k] > best_score:
                best_score = float(scores[k])
                best_dx = float(trans[k, 0])
                best_dy = float(trans[k, 1])
                best_theta = theta

        best = Pose2D(center.x + best_dx, center.y + best_dy,
                      wrap_angle(best_theta))
        return best, best_score

    @staticmethod
    def _linrange(window: float, step: float) -> np.ndarray:
        """Symmetric offsets in [-window, window] inclusive at `step` spacing."""
        if step <= 0.0 or window <= 0.0:
            return np.array([0.0], dtype=np.float64)
        n = int(round(window / step))
        return np.arange(-n, n + 1, dtype=np.float64) * step

    # ------------------------------------------------------------------
    # Small helpers
    # ------------------------------------------------------------------

    def _motion_since(self, ref: Pose2D) -> Tuple[float, float]:
        d_trans = math.hypot(self.pose.x - ref.x, self.pose.y - ref.y)
        d_rot = abs(wrap_angle(self.pose.theta - ref.theta))
        return d_trans, d_rot

    @staticmethod
    def _clone(p: Pose2D) -> Pose2D:
        return Pose2D(p.x, p.y, p.theta)
