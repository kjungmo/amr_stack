"""File-based configuration: dataclass schema + YAML loader + dotted overrides.

Dataclass defaults are canonical. YAML files may set any subset of keys;
unknown keys and type mismatches raise ConfigError with a dotted path.
"""
import copy
import dataclasses
import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, get_args, get_origin, get_type_hints

import yaml


class ConfigError(Exception):
    pass


@dataclass
class RobotConfig:
    radius: float = 0.18
    max_lin_vel: float = 0.6
    max_ang_vel: float = 1.8
    max_lin_acc: float = 0.8
    max_ang_acc: float = 2.5


@dataclass
class LidarConfig:
    num_beams: int = 240
    angle_min: float = -math.pi
    angle_max: float = math.pi          # increment = (max - min) / num_beams
    range_min: float = 0.12
    range_max: float = 8.0
    noise_std: float = 0.01
    scan_every: int = 2                 # emit a scan every N sim steps


@dataclass
class OdomNoiseConfig:
    alpha_v: float = 0.03               # std(v_meas) = alpha_v*|v| + floor
    alpha_w: float = 0.03
    floor: float = 1e-4


@dataclass
class SimConfig:
    dt: float = 0.05
    world_file: str = "configs/worlds/office.yaml"
    odom_noise: OdomNoiseConfig = field(default_factory=OdomNoiseConfig)


@dataclass
class MappingConfig:
    resolution: float = 0.05
    l_occ: float = 0.85
    l_free: float = -0.4
    l_clamp: float = 10.0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.25
    beam_subsample: int = 2


@dataclass
class SlamConfig:
    keyframe_trans: float = 0.2         # integrate scan into map after this motion
    keyframe_rot: float = 0.35
    min_motion: float = 0.02            # skip matching below this displacement
    match_beams: int = 80
    coarse_window_xy: float = 0.15
    coarse_step_xy: float = 0.05
    coarse_window_theta: float = 0.12
    coarse_step_theta: float = 0.03
    fine_step_xy: float = 0.025
    fine_step_theta: float = 0.01
    blur_sigma_cells: float = 1.5
    min_match_score: float = 0.1


@dataclass
class LikelihoodConfig:
    sigma_hit: float = 0.2
    z_hit: float = 0.9
    z_rand: float = 0.1
    max_dist: float = 2.0
    beam_subsample: int = 5


@dataclass
class LocalizationConfig:
    num_particles: int = 500
    alphas: List[float] = field(default_factory=lambda: [0.05, 0.05, 0.05, 0.05])
    init_std: List[float] = field(default_factory=lambda: [0.25, 0.25, 0.15])
    resample_neff_frac: float = 0.5
    likelihood: LikelihoodConfig = field(default_factory=LikelihoodConfig)


@dataclass
class CostmapConfig:
    occupied_thresh: int = 65
    unknown_is_lethal: bool = True
    inflation_radius: float = 0.45
    cost_decay: float = 6.0


@dataclass
class AstarConfig:
    w_cost: float = 4.0
    simplify: bool = True


@dataclass
class DwaConfig:
    sim_time: float = 1.5
    sim_dt: float = 0.1
    v_samples: int = 8
    w_samples: int = 15
    lookahead: float = 0.8
    w_progress: float = 1.0
    w_heading: float = 0.6
    w_clearance: float = 0.4
    w_velocity: float = 0.3


@dataclass
class PlanningConfig:
    costmap: CostmapConfig = field(default_factory=CostmapConfig)
    astar: AstarConfig = field(default_factory=AstarConfig)
    dwa: DwaConfig = field(default_factory=DwaConfig)


@dataclass
class NavConfig:
    goal_tol_xy: float = 0.25
    replan_period: float = 4.0
    path_block_check_dist: float = 1.0
    max_recoveries: int = 3
    recovery_rotate_speed: float = 0.8
    recovery_backup_dist: float = 0.3
    recovery_backup_speed: float = 0.1


@dataclass
class LoggingConfig:
    level: str = "INFO"
    file: str = "logs/amr.log"
    max_bytes: int = 1000000
    backup_count: int = 3
    console: bool = True
    module_levels: Dict[str, str] = field(default_factory=dict)


@dataclass
class GuiConfig:
    refresh_ms: int = 66
    px_per_cell: int = 3
    speed_factor: float = 1.0           # worker pacing; 0 = run flat out


@dataclass
class AmrConfig:
    seed: int = 42
    robot: RobotConfig = field(default_factory=RobotConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    slam: SlamConfig = field(default_factory=SlamConfig)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)
    planning: PlanningConfig = field(default_factory=PlanningConfig)
    nav: NavConfig = field(default_factory=NavConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    gui: GuiConfig = field(default_factory=GuiConfig)


def _coerce(value, ftype, path):
    origin = get_origin(ftype)
    if dataclasses.is_dataclass(ftype):
        if not isinstance(value, dict):
            raise ConfigError("%s: expected a mapping" % path)
        return _from_dict(ftype, value, path)
    if origin is list:
        (etype,) = get_args(ftype)
        if not isinstance(value, list):
            raise ConfigError("%s: expected a list" % path)
        return [_coerce(v, etype, "%s[%d]" % (path, i)) for i, v in enumerate(value)]
    if origin is dict:
        if not isinstance(value, dict):
            raise ConfigError("%s: expected a mapping" % path)
        return dict(value)
    if ftype is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError("%s: expected a number, got %r" % (path, value))
        return float(value)
    if ftype is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError("%s: expected an int, got %r" % (path, value))
        return value
    if ftype is bool:
        if not isinstance(value, bool):
            raise ConfigError("%s: expected a bool, got %r" % (path, value))
        return value
    if ftype is str:
        if not isinstance(value, str):
            raise ConfigError("%s: expected a string, got %r" % (path, value))
        return value
    return value


def _from_dict(dc_type, d, path=""):
    hints = get_type_hints(dc_type)
    names = {f.name for f in dataclasses.fields(dc_type)}
    unknown = set(d) - names
    if unknown:
        raise ConfigError("unknown config key(s) %s under '%s'"
                          % (sorted(unknown), path or "root"))
    kwargs = {k: _coerce(v, hints[k], (path + "." + k).lstrip("."))
              for k, v in d.items()}
    return dc_type(**kwargs)


def load_config(path=None, overrides: Sequence[str] = ()) -> AmrConfig:
    if path is None:
        cfg = AmrConfig()
    else:
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        cfg = _from_dict(AmrConfig, raw)
    return apply_overrides(cfg, overrides) if overrides else cfg


def apply_overrides(cfg: AmrConfig, overrides: Sequence[str]) -> AmrConfig:
    cfg = copy.deepcopy(cfg)
    for item in overrides:
        if "=" not in item:
            raise ConfigError("override '%s' must look like a.b.c=value" % item)
        dotted, _, raw = item.partition("=")
        keys = dotted.strip().split(".")
        node = cfg
        for k in keys[:-1]:
            if not hasattr(node, k):
                raise ConfigError("unknown config path '%s'" % dotted)
            node = getattr(node, k)
        leaf = keys[-1]
        if not dataclasses.is_dataclass(node) or not hasattr(node, leaf):
            raise ConfigError("unknown config path '%s'" % dotted)
        ftype = get_type_hints(type(node))[leaf]
        setattr(node, leaf, _coerce(yaml.safe_load(raw), ftype, dotted))
    return cfg
