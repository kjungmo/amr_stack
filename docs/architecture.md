# AMR Stack — Architecture

## Module Diagram

```
amr_stack/
│
│  ┌─────────────────────────────────────────────────────────┐
│  │  amr.core  (frozen public contract — no dependencies)   │
│  │  types.py · geometry.py · config.py · log.py            │
│  └──────────────────────────┬──────────────────────────────┘
│                             │  (all subsystems depend on core only)
│       ┌─────────────────────┼──────────────────────┐
│       │                     │                       │
│  ┌────▼────┐  ┌──────────┐  ┌────────────┐  ┌──────▼──────┐
│  │ amr.sim │  │amr.mapping│  │amr.planning│  │amr.locali-  │
│  │ world   │  │map_io     │  │costmap     │  │zation       │
│  │ robot   │  │occ_grid_  │  │astar       │  │motion_model │
│  │ lidar   │  │mapper     │  │dwa         │  │sensor_model │
│  │ simul.  │  └──────┬───┘  └──────┬─────┘  │mcl          │
│  └────┬────┘         │             │        └──────┬──────┘
│       │              │             │               │
│       │         ┌────▼────┐   ┌────▼────┐         │
│       └────────►│ amr.slam│   │amr.navi-│◄────────┘
│                 │scan_    │   │gation   │
│                 │matching │   │navigator│
│                 │_slam    │   └────┬────┘
│                 └────┬────┘        │
│                      │             │
│               ┌──────▼─────────────▼──────┐
│               │   amr.runtime.app          │
│               │   AmrApp (SLAM / NAV mode) │
│               └──────────┬────────────────┘
│                          │  Snapshot + Commands
│              ┌───────────┴────────────┐
│         ┌────▼─────┐           ┌──────▼──────┐
│         │  amr.cli  │           │  amr.gui    │
│         │ (headless)│           │ (tkinter)   │
│         └───────────┘           └─────────────┘
```

Dependencies are strictly one-way and downward. `amr.slam` depends on both `amr.sim` and
`amr.mapping`; `amr.navigation` depends on `amr.planning`; `amr.runtime.app` is the only
module that wires subsystems together.

---

## Coordinate and Data Conventions (FROZEN — every module obeys these)

1. **World frame:** x right, y up, meters. `theta` CCW radians wrapped to `(-pi, pi]`.
2. **Robot frame:** x forward, y left. Lidar beam angle 0 = robot forward.
3. **Grid indexing:** `data[row, col]`; `col` maps to x, `row` maps to y; **row 0 = minimum-y
   (south) edge**. No flipping anywhere except PGM image I/O (image row 0 = top).
4. **Cell to world:** `col = floor((x - origin_x)/res)`; `grid_to_world` returns **cell centers**.
5. **Occupancy values (int8, ROS convention):** `-1` unknown, `0` free, `100` occupied.
   A cell is "occupied" iff `data >= 65`.
6. **Map files:** ROS `map_server`-compatible pair `stem.pgm` (P5 binary: occupied→0, free→254,
   unknown→205, `np.flipud` on write/read) + `stem.yaml` (`image, resolution,
   origin: [x, y, 0], negate: 0, occupied_thresh: 0.65, free_thresh: 0.25`).
7. **Determinism:** every stochastic component takes an `np.random.Generator`; all are seeded
   from `cfg.seed` (default 42). Tests rely on this — `np.random.*` module-level functions are
   never called directly.
8. **Units:** SI everywhere (m, rad, s, m/s, rad/s).

---

## SLAM Mode — Data-Flow Step Order

Executed every call to `AmrApp.step()` in `Mode.SLAM`:

1. **Command selection:** if e-stop active → zero twist; if manual mode → use `teleop` twist;
   otherwise call `WaypointDriver.update(slam.pose, last_scan)` to get the mission command.
2. **Simulator advance:** `res = sim.step(cmd)` — integrates kinematics, applies odometry noise,
   fires the lidar every `scan_every` sim steps.
3. **SLAM fusion:** `slam.process(res.odom_delta, res.scan)` — odometry propagation then (when
   a scan is available) correlative scan matching and log-odds map update.
4. **Snapshot assembly:** collects `slam.pose` (estimate), `res.ground_truth` (GT), current scan
   endpoints, the live occupancy grid, and status strings into an immutable `Snapshot`.

---

## NAV Mode — Data-Flow Step Order

Executed every call to `AmrApp.step()` in `Mode.NAV`:

1. **Simulator advance:** `res = sim.step(prev_cmd)` using the command computed at the previous
   tick (zero on the first tick and during e-stop).
2. **MCL predict:** `mcl.predict(res.odom_delta)` — Thrun odometry motion model applied to all
   particles.
3. **MCL correct (when scan available):** `mcl.correct(res.scan)` — likelihood-field sensor model
   weights particles; low-variance resampling when effective N falls below threshold.
4. **Pose estimate:** `pose = mcl.estimate()` — weighted mean for x, y; circular mean for theta.
5. **Navigator update:** `cmd = navigator.update(pose, robot.vel, costmap, sim_time)` — runs the
   NAV FSM: A* replanning, DWA local planning, recovery behaviors.
6. **Snapshot assembly:** collects pose estimate, ground truth, particles array (N×3), planned
   path, goal, static map grid, nav-state string.

---

## Algorithm Summaries

### Log-Odds Occupancy Mapping (`amr.mapping.OccupancyGridMapper`)

A standard Bayesian binary occupancy grid using log-odds accumulation. For each incoming scan,
beams are subsampled by `beam_subsample` (default 2) to control update rate. A Bresenham ray is
traced from the robot cell to the endpoint: all intermediate cells receive `l_free = -0.4`
(log-odds decrement), and the endpoint cell receives `l_occ = +0.85` if the beam is a valid
return. Log-odds values are clamped to `±l_clamp = 10.0` to prevent certainty lock-in. At query
time, log-odds are converted to probability `p = 1 - 1/(1+exp(lo))`: cells with `p > 0.65` are
occupied (`100`), `p < 0.25` are free (`0`), otherwise unknown (`-1`). The asymmetric l_occ/l_free
ratio (0.85 vs 0.4) reflects that a lidar hit is a strong positive signal while free-space
traversal is noisier.

### Correlative Scan-Matching SLAM (`amr.slam.ScanMatchingSlam`)

Pose-graph SLAM is out of scope for v0.1; office-scale drift is bounded by the scan-matching
correction. On each scan, the algorithm searches for the best-fit rigid body transform around the
odometry-predicted pose against a blurred version of the current occupancy score map. The score
map is computed by thresholding p_occ > 0.6, then applying a separable Gaussian blur
(σ = `blur_sigma_cells = 1.5` cells, half-width 3σ) to give soft attraction towards walls. The
search is two-stage: a coarse grid (±0.15 m, 0.05 m step; ±0.12 rad, 0.03 rad step) followed by
a fine grid (±coarse_step, 0.025 m / 0.01 rad step) around the coarse winner. For each candidate
angle, the `match_beams = 80` subsampled scan endpoints are rotated once and the translation grid
is applied via broadcasting, keeping the search O(Kθ · Kxy · B) with numpy. The match score must
exceed `min_match_score = 0.1`; below that threshold the odometry prediction is kept (protecting
against noisy updates on a nearly empty map). Map updates are keyframed: a new scan is integrated
into the occupancy map only after the robot has moved ≥ `keyframe_trans = 0.2 m` or
`keyframe_rot = 0.35 rad` since the last keyframe, preventing redundant updates while stationary.
**Loop closure is out of scope for v0.1.**

### MCL Likelihood-Field Sensor Model (`amr.localization`)

The particle filter implements the Thrun-Burgard-Fox probabilistic robotics formulation. The
**motion model** decomposes the odometry delta into (rot1, trans, rot2) in the robot frame and
adds zero-mean Gaussian noise to each component, with standard deviations proportional to
`alphas = [0.05, 0.05, 0.05, 0.05]` — empirically chosen to match the simulator's odometry noise
level. The **sensor model** uses a pre-computed Euclidean distance field (chamfer approximation,
max_dist = 2.0 m) over the static map's occupied cells. For each particle, `beam_subsample = 5`
valid beams are vectorized over all N particles simultaneously via broadcasting, yielding N×B
endpoint-to-obstacle distances. The per-beam likelihood is
`q = z_hit · exp(-d²/(2·sigma_hit²)) + z_rand / range_max`, with `sigma_hit = 0.2 m`,
`z_hit = 0.9`, `z_rand = 0.1`. The weight is the product of per-beam likelihoods (computed as
`exp(sum(log(q)))`). With `num_particles = 500` and the low-variance resampler (triggered when
the effective N falls below `resample_neff_frac = 0.5 · N`), the filter converges reliably from
a Gaussian initialization at the spawn pose.

### Costmap Inflation (`amr.planning.Costmap`)

The costmap converts the binary occupancy map into a scalar cost field suitable for
collision-aware planning. The obstacle mask includes cells with `data >= 65` and, when
`unknown_is_lethal = True`, cells with `data < 0` (unknown). A Euclidean distance field (chamfer
approximation) is computed over the mask up to `inflation_radius = 0.45 m`. Cells within
`robot_radius` of any obstacle receive cost 1.0 (lethal); cells in the band
`(robot_radius, inflation_radius)` receive `cost = exp(-cost_decay · (d - robot_radius))` with
`cost_decay = 6.0`, giving a smooth repulsive gradient. Because the costmap already accounts for
the robot radius, all downstream planners treat the robot as a point. In NAV mode the
`AmrApp._init_nav` method constructs the costmap with `robot_radius + _NAV_PLANNING_MARGIN`
(0.06 m) to keep global paths off razor-thin obstacle corners without narrowing real doorways
(office door gaps are ≥ 0.7 m clear).

### A* Global Planner (`amr.planning.plan_path`)

8-connected A* search on the costmap grid. The step cost for a neighbor is
`step_len · (1 + w_cost · cost[nbr])`, with `step_len ∈ {1, √2}` (cell units) and `w_cost = 4.0`.
The heuristic is the octile distance (admissible because the cost multiplier is always ≥ 1). Lethal
cells are excluded from the open set. If the start cell is lethal (e.g., due to localization lag),
a BFS within 0.3 m finds the nearest non-lethal cell to start from; if the goal cell is lethal the
planner returns None. After reconstruction, path simplification (when `simplify = True`) greedily
removes waypoints whose successor can be reached via a non-lethal line of sight, reducing the number
of DWA tracking points and smoothing the trajectory through open spaces.

### DWA Local Planner (`amr.planning.DwaPlanner`)

The Dynamic Window Approach samples a grid of (`v_samples = 8`) × (`w_samples = 15`) velocity
candidates within the dynamic window — the intersection of the velocity limits and the set
reachable in one `sim_time = 1.5 s` horizon given the acceleration limits. Each candidate is rolled
out as a unicycle at `sim_dt = 0.1 s` steps; any rollout containing a cell with cost ≥ 0.99 is
rejected. Surviving rollouts are scored on four normalized objectives:
`progress` (negative distance from rollout end to the carrot point at `lookahead = 0.8 m` ahead on
the path), `heading` (negative bearing error to the carrot), `clearance` (minimum 1−cost along the
rollout), and `velocity` (v / v_max). Weights: `w_progress = 1.0`, `w_heading = 0.6`,
`w_clearance = 0.4`, `w_velocity = 0.3`. Near-goal slowdown caps sampled v when the robot is within
one lookahead distance of the goal. If all rollouts collide, the planner returns `blocked=True` and
the navigator triggers a recovery behavior.

### Navigation FSM (`amr.navigation.Navigator`)

A deterministic FSM with six states: IDLE, PLANNING, FOLLOWING, RECOVERY, SUCCEEDED, FAILED.
From PLANNING, the A* planner is invoked; success transitions to FOLLOWING, failure increments a
recovery counter (FAILED after `max_recoveries = 3`) and enters RECOVERY. In FOLLOWING, the DWA
provides the velocity command; the path is replanned every `replan_period = 4.0 s` or immediately
if any waypoint within `path_block_check_dist = 1.0 m` ahead is lethal. DWA-blocked transitions
to RECOVERY; within `goal_tol_xy = 0.25 m` of the goal transitions to SUCCEEDED. RECOVERY
alternates between rotating in place at `recovery_rotate_speed = 0.8 rad/s` for one full revolution
(2π rad) and backing up `recovery_backup_dist = 0.3 m` at `recovery_backup_speed = 0.1 m/s`;
progress is integrated from the caller's simulation clock, not from pose feedback. After recovery
the FSM re-enters PLANNING with a fresh path request.
