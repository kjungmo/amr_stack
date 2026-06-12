import dataclasses

import pytest

from amr.core.config import AmrConfig, ConfigError, apply_overrides, load_config


def test_defaults_match_reference_yaml():
    # configs/default.yaml is the complete documented reference; it must stay
    # in lockstep with the dataclass defaults.
    assert load_config("configs/default.yaml") == AmrConfig()


def test_partial_yaml_and_overrides(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("robot:\n  radius: 0.25\nlocalization:\n  num_particles: 100\n")
    cfg = load_config(str(p))
    assert cfg.robot.radius == 0.25
    assert cfg.localization.num_particles == 100
    assert cfg.nav.goal_tol_xy == AmrConfig().nav.goal_tol_xy  # untouched default

    cfg = apply_overrides(cfg, ["nav.goal_tol_xy=0.5", "slam.match_beams=40"])
    assert cfg.nav.goal_tol_xy == 0.5 and cfg.slam.match_beams == 40


def test_unknown_key_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("robot:\n  radíus_typo: 0.2\n")
    with pytest.raises(ConfigError):
        load_config(str(p))
    with pytest.raises(ConfigError):
        apply_overrides(AmrConfig(), ["robot.nope=1"])


def test_type_mismatch_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("localization:\n  num_particles: many\n")
    with pytest.raises(ConfigError):
        load_config(str(p))


def test_config_is_dataclass_tree():
    cfg = AmrConfig()
    assert dataclasses.is_dataclass(cfg.planning.dwa)
