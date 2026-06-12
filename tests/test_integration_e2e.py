import math

import pytest

from amr.core.config import load_config
from amr.runtime.app import AmrApp, Mode, SaveMap, SetGoal


@pytest.mark.slow
def test_full_autonomy_pipeline(tmp_path):
    cfg = load_config("configs/default.yaml",
                      overrides=["logging.console=false",
                                 "localization.num_particles=400"])

    # ---- Phase 1: autonomous SLAM mapping mission -------------------------
    app = AmrApp(cfg, Mode.SLAM,
                 mission_file="configs/missions/office_mapping.yaml")
    snap = app.run_headless(240.0, stop_when=lambda s: s.status == "mission_complete")
    assert snap.status == "mission_complete", "mapping mission did not finish in time"
    slam_err = math.hypot(snap.pose.x - snap.gt_pose.x, snap.pose.y - snap.gt_pose.y)
    assert slam_err < 0.30
    stem = str(tmp_path / "office")
    app.handle_command(SaveMap(stem))

    # ---- Phase 2: localize + navigate on the SLAM-built map ---------------
    app2 = AmrApp(cfg, Mode.NAV, map_path=stem + ".yaml")
    deadline = 180.0                                      # sim-seconds for goal 1
    for gx, gy in [(10.5, 1.5), (2.0, 7.0)]:
        app2.handle_command(SetGoal(gx, gy))
        snap = app2.run_headless(
            deadline, stop_when=lambda s: s.nav_state in ("SUCCEEDED", "FAILED"))
        assert snap.nav_state == "SUCCEEDED", "failed to reach (%s, %s)" % (gx, gy)
        true_err = math.hypot(snap.gt_pose.x - gx, snap.gt_pose.y - gy)
        assert true_err < cfg.nav.goal_tol_xy + 0.15      # honest, ground-truth check
        assert not snap.collided
        deadline = snap.sim_time + 180.0                  # ceiling is absolute sim time
