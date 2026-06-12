import math

import pytest

from amr.core.types import Pose2D, Twist2D
from amr.localization.mcl import MonteCarloLocalizer
from amr.sim.simulator import Simulator


@pytest.mark.slow
def test_mcl_tracks_through_motion(box_world, cfg, rng):
    cfg.lidar.noise_std = 0.01
    sim = Simulator(box_world, cfg, rng)
    mcl = MonteCarloLocalizer(box_world.grid, cfg.localization, rng,
                              initial_pose=box_world.spawn)
    script = [(0.3, 0.0, 4.0), (0.0, 0.6, 2.5), (0.3, 0.0, 4.0), (0.0, 0.6, 2.5),
              (0.3, 0.0, 4.0)]
    for v, w, dur in script:
        for _ in range(int(dur / cfg.sim.dt)):
            res = sim.step(Twist2D(v, w))
            mcl.predict(res.odom_delta)
            if res.scan is not None:
                mcl.correct(res.scan)
    est, gt = mcl.estimate(), sim.robot.pose
    assert math.hypot(est.x - gt.x, est.y - gt.y) < 0.25
    assert abs(math.remainder(est.theta - gt.theta, 2 * math.pi)) < 0.25


def test_set_pose_resets_cloud(box_world, cfg, rng):
    mcl = MonteCarloLocalizer(box_world.grid, cfg.localization, rng,
                              initial_pose=Pose2D(1, 1, 0))
    mcl.set_pose(Pose2D(4.0, 4.0, 1.0))
    assert abs(mcl.particles[:, 0].mean() - 4.0) < 0.1
    e = mcl.estimate()
    assert (round(e.x, 0), round(e.y, 0)) == (4.0, 4.0)
