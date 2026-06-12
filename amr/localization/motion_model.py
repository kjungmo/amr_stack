"""Thrun odometry motion model (sample_motion_model_odometry).

Decomposes the robot-frame relative odometry into rot1 / trans / rot2 and
applies independent per-particle Gaussian noise, vectorized over all particles.
See Probabilistic Robotics (Thrun et al.) Table 5.6.
"""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from amr.core.geometry import wrap_angle, wrap_angles
from amr.core.types import Pose2D


def sample_motion(particles: np.ndarray, odom_delta: Pose2D,
                  alphas: Sequence[float], rng: np.random.Generator) -> np.ndarray:
    """Thrun odometry motion model applied to (N,3) particles; returns new (N,3).

    ``odom_delta`` is the robot-frame relative motion (dx, dy, dth) since the
    last update. ``alphas`` are the four noise coefficients (a1..a4).
    """
    particles = np.asarray(particles, dtype=float)
    p = particles.copy()
    n = p.shape[0]

    a1, a2, a3, a4 = float(alphas[0]), float(alphas[1]), float(alphas[2]), float(alphas[3])

    dx, dy, dth = float(odom_delta.x), float(odom_delta.y), float(odom_delta.theta)

    trans = math.hypot(dx, dy)
    rot1 = math.atan2(dy, dx) if trans > 1e-4 else 0.0
    rot2 = wrap_angle(dth - rot1)

    abs_rot1 = abs(rot1)
    abs_rot2 = abs(rot2)

    std_rot1 = a1 * abs_rot1 + a2 * trans
    std_trans = a3 * trans + a4 * (abs_rot1 + abs_rot2)
    std_rot2 = a1 * abs_rot2 + a2 * trans

    rot1_s = rot1 + rng.normal(0.0, std_rot1, n)
    trans_s = trans + rng.normal(0.0, std_trans, n)
    rot2_s = rot2 + rng.normal(0.0, std_rot2, n)

    th = p[:, 2]
    p[:, 0] = p[:, 0] + trans_s * np.cos(th + rot1_s)
    p[:, 1] = p[:, 1] + trans_s * np.sin(th + rot1_s)
    p[:, 2] = wrap_angles(th + rot1_s + rot2_s)

    return p
