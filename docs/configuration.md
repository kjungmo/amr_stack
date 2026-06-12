# AMR Stack — Configuration Reference

Configuration is managed by `amr.core.config`. The canonical source of truth is the dataclass
hierarchy in `amr/core/config.py`; the YAML file `configs/default.yaml` must be an exact mirror
(enforced by `test_config.py::test_defaults_match_reference_yaml`).

## Loading and Overriding

```python
from amr.core.config import load_config, apply_overrides

# Load defaults
cfg = load_config()

# Load a YAML file (any subset of keys)
cfg = load_config("configs/default.yaml")

# Apply dotted CLI overrides
cfg = load_config("myconfig.yaml", overrides=["slam.match_beams=60", "gui.px_per_cell=4"])
```

From the CLI: `amr --config PATH --set KEY=VAL --set KEY2=VAL2 <subcommand>`

Unknown keys or type mismatches raise `ConfigError` with a dotted path to the bad field.

---

## Top-Level (`AmrConfig`)

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `seed` | `int` | `42` | Global RNG seed shared by the simulator and MCL | `amr.runtime.app` |

---

## `robot` — `RobotConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `radius` | `float` | `0.18` | Robot footprint radius (m); used for collision detection and costmap inflation | `amr.sim.robot`, `amr.runtime.app` |
| `max_lin_vel` | `float` | `0.6` | Maximum forward speed (m/s) | `amr.sim.robot`, `amr.planning.dwa` |
| `max_ang_vel` | `float` | `1.8` | Maximum angular speed (rad/s) | `amr.sim.robot`, `amr.planning.dwa` |
| `max_lin_acc` | `float` | `0.8` | Maximum linear acceleration (m/s²); ramp-limits the commanded twist | `amr.sim.robot`, `amr.planning.dwa` |
| `max_ang_acc` | `float` | `2.5` | Maximum angular acceleration (rad/s²) | `amr.sim.robot`, `amr.planning.dwa` |

---

## `lidar` — `LidarConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `num_beams` | `int` | `240` | Number of lidar beams per scan | `amr.sim.lidar` |
| `angle_min` | `float` | `-π` | First beam angle in the robot frame (rad) | `amr.sim.lidar` |
| `angle_max` | `float` | `π` | Last beam angle; increment = (max−min)/num_beams | `amr.sim.lidar` |
| `range_min` | `float` | `0.12` | Minimum valid range (m); shorter returns are masked invalid | `amr.sim.lidar`, `amr.localization` |
| `range_max` | `float` | `8.0` | Maximum range (m); no-return beams report this value | `amr.sim.lidar`, `amr.localization` |
| `noise_std` | `float` | `0.01` | Gaussian range noise standard deviation (m) | `amr.sim.lidar` |
| `scan_every` | `int` | `2` | Emit a scan every N simulator steps (at dt=0.05 s → 10 Hz) | `amr.sim.simulator` |

---

## `sim` — `SimConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `dt` | `float` | `0.05` | Simulator time step (s) | `amr.sim.simulator` |
| `world_file` | `str` | `"configs/worlds/office.yaml"` | Path to the world YAML file | `amr.runtime.app` |
| `odom_noise` | `OdomNoiseConfig` | see below | Odometry noise parameters | `amr.sim.simulator` |

### `sim.odom_noise` — `OdomNoiseConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `alpha_v` | `float` | `0.03` | Proportional velocity noise: std = alpha_v·|v| + floor | `amr.sim.simulator` |
| `alpha_w` | `float` | `0.03` | Proportional angular noise: std = alpha_w·|ω| + floor | `amr.sim.simulator` |
| `floor` | `float` | `1e-4` | Minimum noise floor (prevents zero noise at standstill) | `amr.sim.simulator` |

---

## `mapping` — `MappingConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `resolution` | `float` | `0.05` | Grid cell size (m/cell) | `amr.mapping.occupancy_grid_mapper` |
| `l_occ` | `float` | `0.85` | Log-odds increment when a beam endpoint hits a cell | `amr.mapping.occupancy_grid_mapper` |
| `l_free` | `float` | `-0.4` | Log-odds increment for cells traversed by a beam ray | `amr.mapping.occupancy_grid_mapper` |
| `l_clamp` | `float` | `10.0` | Log-odds clamping limit (±); prevents certainty lock-in | `amr.mapping.occupancy_grid_mapper` |
| `occupied_thresh` | `float` | `0.65` | Probability threshold: p > thresh → mark as occupied (100) | `amr.mapping.occupancy_grid_mapper` |
| `free_thresh` | `float` | `0.25` | Probability threshold: p < thresh → mark as free (0) | `amr.mapping.occupancy_grid_mapper` |
| `beam_subsample` | `int` | `2` | Use every Nth beam during map update (speed vs resolution tradeoff) | `amr.mapping.occupancy_grid_mapper` |

---

## `slam` — `SlamConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `keyframe_trans` | `float` | `0.2` | Minimum translation (m) since last keyframe to trigger map update | `amr.slam.scan_matching_slam` |
| `keyframe_rot` | `float` | `0.35` | Minimum rotation (rad) since last keyframe to trigger map update | `amr.slam.scan_matching_slam` |
| `min_motion` | `float` | `0.02` | Skip scan matching entirely below this displacement | `amr.slam.scan_matching_slam` |
| `match_beams` | `int` | `80` | Number of beams subsampled for the scan-matching search | `amr.slam.scan_matching_slam` |
| `coarse_window_xy` | `float` | `0.15` | Half-width of the coarse XY search window (m) | `amr.slam.scan_matching_slam` |
| `coarse_step_xy` | `float` | `0.05` | Step size in the coarse XY search (m) | `amr.slam.scan_matching_slam` |
| `coarse_window_theta` | `float` | `0.12` | Half-width of the coarse angular search window (rad) | `amr.slam.scan_matching_slam` |
| `coarse_step_theta` | `float` | `0.03` | Step size in the coarse angular search (rad) | `amr.slam.scan_matching_slam` |
| `fine_step_xy` | `float` | `0.025` | Step size in the fine XY refinement (m) | `amr.slam.scan_matching_slam` |
| `fine_step_theta` | `float` | `0.01` | Step size in the fine angular refinement (rad) | `amr.slam.scan_matching_slam` |
| `blur_sigma_cells` | `float` | `1.5` | Gaussian blur sigma (cells) applied to the score map | `amr.slam.scan_matching_slam` |
| `min_match_score` | `float` | `0.1` | Minimum correlation score to accept a match (else keep odometry) | `amr.slam.scan_matching_slam` |

**Performance tuning:** lower `match_beams` to speed up SLAM at the cost of accuracy; increase
`blur_sigma_cells` to tolerate larger initial misalignments; widen `coarse_window_*` if the robot
makes fast turns.

---

## `localization` — `LocalizationConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `num_particles` | `int` | `500` | Number of particles in the MCL filter | `amr.localization.mcl` |
| `alphas` | `List[float]` | `[0.05, 0.05, 0.05, 0.05]` | Odometry noise model coefficients [a1, a2, a3, a4] | `amr.localization.motion_model` |
| `init_std` | `List[float]` | `[0.25, 0.25, 0.15]` | Initial pose uncertainty [std_x, std_y, std_theta] (m, m, rad) | `amr.localization.mcl` |
| `resample_neff_frac` | `float` | `0.5` | Resample when effective N < frac·N (low-variance resampler) | `amr.localization.mcl` |
| `likelihood` | `LikelihoodConfig` | see below | Sensor model parameters | `amr.localization.sensor_model` |

### `localization.likelihood` — `LikelihoodConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `sigma_hit` | `float` | `0.2` | Standard deviation of the Gaussian hit likelihood (m) | `amr.localization.sensor_model` |
| `z_hit` | `float` | `0.9` | Weight of the Gaussian hit term in the beam likelihood | `amr.localization.sensor_model` |
| `z_rand` | `float` | `0.1` | Weight of the uniform random term | `amr.localization.sensor_model` |
| `max_dist` | `float` | `2.0` | Distance field cutoff (m); endpoints beyond are clamped here | `amr.localization.sensor_model` |
| `beam_subsample` | `int` | `5` | Use every Nth valid beam for likelihood computation | `amr.localization.sensor_model` |

---

## `planning` — `PlanningConfig`

### `planning.costmap` — `CostmapConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `occupied_thresh` | `int` | `65` | Occupancy value threshold for the obstacle mask (cells >= this are obstacles) | `amr.planning.costmap` |
| `unknown_is_lethal` | `bool` | `True` | Treat unknown cells (data < 0) as lethal obstacles | `amr.planning.costmap` |
| `inflation_radius` | `float` | `0.45` | Outer radius of the cost inflation band (m) | `amr.planning.costmap` |
| `cost_decay` | `float` | `6.0` | Exponential decay coefficient in the inflation band | `amr.planning.costmap` |

### `planning.astar` — `AstarConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `w_cost` | `float` | `4.0` | Weight on the costmap term in A* step cost: cost = step_len·(1 + w_cost·cell_cost) | `amr.planning.astar` |
| `simplify` | `bool` | `True` | Enable greedy path simplification via line-of-sight shortcutting | `amr.planning.astar` |

### `planning.dwa` — `DwaConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `sim_time` | `float` | `1.5` | Rollout horizon (s) also used as the dynamic-window reachability horizon | `amr.planning.dwa` |
| `sim_dt` | `float` | `0.1` | Rollout integration timestep (s) | `amr.planning.dwa` |
| `v_samples` | `int` | `8` | Number of linear velocity samples in the dynamic window | `amr.planning.dwa` |
| `w_samples` | `int` | `15` | Number of angular velocity samples in the dynamic window | `amr.planning.dwa` |
| `lookahead` | `float` | `0.8` | Carrot-point lookahead distance on the global path (m) | `amr.planning.dwa` |
| `w_progress` | `float` | `1.0` | Weight for the progress-toward-carrot objective | `amr.planning.dwa` |
| `w_heading` | `float` | `0.6` | Weight for the heading-toward-carrot objective | `amr.planning.dwa` |
| `w_clearance` | `float` | `0.4` | Weight for the minimum-clearance objective | `amr.planning.dwa` |
| `w_velocity` | `float` | `0.3` | Weight for the forward-speed objective | `amr.planning.dwa` |

---

## `nav` — `NavConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `goal_tol_xy` | `float` | `0.25` | Distance threshold for declaring goal reached (m) | `amr.navigation.navigator` |
| `replan_period` | `float` | `4.0` | Periodic replan interval (s) in FOLLOWING state | `amr.navigation.navigator` |
| `path_block_check_dist` | `float` | `1.0` | Look-ahead distance for detecting lethal cells on the current path (m) | `amr.navigation.navigator` |
| `max_recoveries` | `int` | `3` | Maximum recovery attempts before declaring FAILED | `amr.navigation.navigator` |
| `recovery_rotate_speed` | `float` | `0.8` | Angular speed during rotate-in-place recovery (rad/s) | `amr.navigation.navigator` |
| `recovery_backup_dist` | `float` | `0.3` | Distance to back up during backup recovery (m) | `amr.navigation.navigator` |
| `recovery_backup_speed` | `float` | `0.1` | Speed during backup recovery (m/s, applied in reverse) | `amr.navigation.navigator` |

---

## `logging` — `LoggingConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `level` | `str` | `"INFO"` | Root log level for the `amr` logger | `amr.core.log` |
| `file` | `str` | `"logs/amr.log"` | Path for the rotating log file (directory created on demand) | `amr.core.log` |
| `max_bytes` | `int` | `1000000` | Maximum log file size before rotation (bytes) | `amr.core.log` |
| `backup_count` | `int` | `3` | Number of rotated log file backups to keep | `amr.core.log` |
| `console` | `bool` | `True` | Enable console (stderr) log output | `amr.core.log` |
| `module_levels` | `Dict[str, str]` | `{}` | Per-module level overrides, e.g. `{"amr.slam": "DEBUG"}` | `amr.core.log` |

---

## `gui` — `GuiConfig`

| Key | Type | Default | Meaning | Read by |
|---|---|---|---|---|
| `refresh_ms` | `int` | `66` | GUI polling interval (ms), approximately 15 Hz | `amr.gui.app` |
| `px_per_cell` | `int` | `3` | Zoom factor: pixels per grid cell in the map canvas | `amr.gui.map_canvas` |
| `speed_factor` | `float` | `1.0` | Worker thread pacing: wall time = dt/speed_factor per step; 0 = flat out | `amr.gui.app` |
