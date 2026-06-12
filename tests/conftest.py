import numpy as np
import pytest

from amr.core.config import load_config
from amr.sim.world import World

BOX_WORLD = {
    "size": [6.0, 6.0], "resolution": 0.05,
    "spawn": {"x": 1.0, "y": 1.0, "theta": 0.0},
    "obstacles": [
        {"type": "rect", "x": 0.0, "y": 0.0, "w": 6.0, "h": 0.1},
        {"type": "rect", "x": 0.0, "y": 5.9, "w": 6.0, "h": 0.1},
        {"type": "rect", "x": 0.0, "y": 0.0, "w": 0.1, "h": 6.0},
        {"type": "rect", "x": 5.9, "y": 0.0, "w": 0.1, "h": 6.0},
        {"type": "rect", "x": 2.7, "y": 2.7, "w": 0.6, "h": 0.6},
    ],
}


@pytest.fixture
def box_world():
    return World.from_dict(BOX_WORLD)


@pytest.fixture
def cfg():
    c = load_config()
    c.lidar.noise_std = 0.0
    c.localization.num_particles = 300
    return c


@pytest.fixture
def rng():
    return np.random.default_rng(42)
