import numpy as np
import pytest

from amr.core.config import LikelihoodConfig
from amr.core.types import LaserScan, Pose2D
from amr.localization.motion_model import sample_motion
from amr.localization.sensor_model import LikelihoodField


def test_motion_model_mean_and_spread(rng):
    p = np.tile([2.0, 3.0, 0.0], (2000, 1))
    out = sample_motion(p, Pose2D(0.5, 0.0, 0.1), [0.05] * 4, rng)
    assert out[:, 0].mean() == pytest.approx(2.5, abs=0.02)
    assert out[:, 1].std() > 0.001                       # noise actually applied
    assert out[:, 2].mean() == pytest.approx(0.1, abs=0.02)


def test_likelihood_prefers_true_pose(box_world, cfg, rng):
    from amr.sim.lidar import Lidar
    truth = Pose2D(1.5, 1.5, 0.5)
    scan = Lidar(cfg.lidar, rng).scan(box_world, truth, 0.0)
    field = LikelihoodField(box_world.grid, LikelihoodConfig())
    particles = np.array([[1.5, 1.5, 0.5],
                          [1.5, 1.5, 1.1],
                          [3.9, 1.2, 0.5],
                          [1.0, 4.0, -2.0]])
    w = field.weigh(particles, scan)
    assert np.argmax(w) == 0
    assert w[0] > 5.0 * w[2]
