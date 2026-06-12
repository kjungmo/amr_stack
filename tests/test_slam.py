import math

import numpy as np
import pytest

from amr.core.types import Twist2D
from amr.slam.scan_matching_slam import ScanMatchingSlam
from amr.sim.simulator import Simulator
from amr.sim.world import World


@pytest.mark.slow
def test_slam_square_loop(cfg, rng):
    world = World.from_yaml("configs/worlds/office.yaml")
    cfg.lidar.noise_std = 0.01
    sim = Simulator(world, cfg, rng)
    slam = ScanMatchingSlam(cfg.slam, cfg.mapping, (12.0, 9.0), world.spawn)

    # drive a square-ish loop in the west room with open-loop commands
    legs = [(0.3, 0.0, 6.0), (0.0, math.pi / 8, 4.0)] * 4
    for v, w, dur in legs:
        for _ in range(int(dur / cfg.sim.dt)):
            res = sim.step(Twist2D(v, w))
            slam.process(res.odom_delta, res.scan)

    gt = sim.robot.pose
    err = math.hypot(slam.pose.x - gt.x, slam.pose.y - gt.y)
    raw_odom = res.odom_pose
    raw_err = math.hypot(raw_odom.x - gt.x, raw_odom.y - gt.y)
    assert err < 0.30                                     # bounded estimate error
    assert err <= raw_err + 0.05                          # no worse than raw odometry

    g = slam.get_map()
    occ_cells = np.argwhere(g.data >= 65)
    assert len(occ_cells) > 200                           # walls actually mapped
    # precision: mapped-occupied cells must hug true obstacles (within 2 cells)
    from amr.core.geometry import distance_field
    d_true = distance_field(world.grid.data >= 65, g.resolution, 1.0)
    hits = sum(1 for r, c in occ_cells if d_true[r, c] <= 2 * g.resolution + 1e-6)
    assert hits / len(occ_cells) > 0.85
