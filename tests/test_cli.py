import os

import pytest

from amr import cli


def test_help_and_version():
    with pytest.raises(SystemExit) as e:
        cli.main(["--help"])
    assert e.value.code == 0


def test_slam_smoke_writes_map(tmp_path):
    rc = cli.main(["--set", "logging.console=false",
                   "slam", "--mission", "configs/missions/office_mapping.yaml",
                   "--out", str(tmp_path / "m"), "--max-time", "5"])
    # 5 sim-seconds: mission not complete -> nonzero, but map files must exist
    assert (tmp_path / "m.pgm").exists() and (tmp_path / "m.yaml").exists()
    assert rc != 0


def test_nav_smoke_on_truth_map(tmp_path, box_world):
    from amr.mapping.map_io import save_map
    save_map(box_world.grid, str(tmp_path / "box"))
    rc = cli.main(["--set", "logging.console=false",
                   "--set", "sim.world_file=tests/box_world.yaml",
                   "nav", "--map", str(tmp_path / "box.yaml"),
                   "--goal", "5.0,5.0", "--max-time", "60"])
    assert rc == 0
