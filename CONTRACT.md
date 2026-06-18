# AMR Stack — ROS 2 C++ Port: Frozen Contract

This is the single source of truth for the C++/ROS 2 port. Every package obeys
it exactly. It ports the pure-Python `amr_stack` (`main` branch) faithfully —
same algorithms, same constants, same coordinate conventions — re-expressed as
idiomatic ROS 2 nodes. Source of algorithm truth: `amr/` and `docs/architecture.md`
in `main`.

Target: **ROS 2 Humble** (this branch). The `jazzy` branch is this workspace with
the Humble→Jazzy deltas in §10 applied.

**CONTRACT_VERSION:** 1.0 (see §11 for the `amr_api` binding).

---

## 1. Workspace layout & dependency graph

```
ros2_ws/src/
  amr_core          lib: geometry, types, config (yaml-cpp)         [deps: yaml-cpp]
  amr_api           lib: module interfaces + adapters seam (node-free)  [deps: amr_core]
  amr_interfaces    msgs/srv/action                                 [deps: rosidl, std/geometry_msgs]
  amr_sim           node: simulator                                 [amr_core, sensor/nav/geometry_msgs, tf2_ros]
  amr_mapping       lib: map_io + log-odds mapper; node: map_publisher [amr_core, nav_msgs]
  amr_localization  lib: motion/sensor model + MCL; node: mcl_node  [amr_core, amr_mapping, sensor/nav/geometry_msgs, tf2_ros]
  amr_slam          lib: scan matcher; node: slam_node              [amr_core, amr_mapping, sensor/nav msgs, tf2_ros, amr_interfaces]
  amr_planning      lib: costmap + A* + DWA                         [amr_core, nav_msgs]
  amr_navigation    node: navigator (FSM + action server)          [amr_core, amr_planning, amr_interfaces, nav/geometry_msgs, tf2_ros]
  amr_bringup       launch + params + rviz                         [all nodes; ament_cmake, launch_testing]
  amr_hri           rviz panel plugin (HRI)                         [rviz_common, pluginlib, amr_interfaces]
  amr_reference_core demo: wires the adapters as a reference orchestrator   [all module libs, amr_api]
```

Dependencies are strictly one-way and downward. Each package builds against
`amr_core` (already built & gtest-verified). **Algorithm = library; ROS wiring =
thin node.** Every algorithm lives in a `*_lib` target with gtests that run
without a node; the node is a rclcpp wrapper. This keeps fidelity to the Python
unit tests and makes the port verifiable package-by-package.

---

## 2. Frozen coordinate & data conventions (identical to `main` docs/architecture.md §Conventions)

1. World frame: x right, y up, meters. `theta` CCW rad wrapped to `(-pi, pi]`.
2. Robot frame: x forward, y left. Lidar beam angle 0 = robot forward.
3. Grid: `data[row,col]` row-major; col→x, row→y; **row 0 = minimum-y**. No flip
   except PGM image I/O (image row 0 = top → flip on write/read).
4. Cell→world returns cell centers; `col = floor((x-origin_x)/res)`.
5. Occupancy int8 (ROS): -1 unknown, 0 free, 100 occupied; occupied iff `>=65`.
6. Map files: ROS `map_server` pair — `stem.pgm` (P5; occupied→0, free→254,
   unknown→205; vertical flip) + `stem.yaml` (`image, resolution, origin:[x,y,0],
   negate:0, occupied_thresh:0.65, free_thresh:0.25`).
7. Determinism: every stochastic node takes a `seed` param (default 42) and owns
   a `std::mt19937`. No global RNG.
8. SI units everywhere (m, rad, s, m/s, rad/s).

---

## 3. tf frames (REP-105)

```
map --(slam_node OR mcl_node)--> odom --(sim_node)--> base_link --(static)--> base_scan
```

- `sim_node` publishes `odom→base_link` (from noisy odometry) on `/tf`, and the
  static `base_link→base_scan` on `/tf_static`.
- `slam_node` (SLAM mode) and `mcl_node` (NAV mode) publish the `map→odom`
  correction on `/tf`. Exactly one of them runs at a time.
- Ground-truth pose is published only as a topic (`/ground_truth`), never as tf.

---

## 4. Topics (name | type | dir | publisher)

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | sub: sim. pub: navigator / teleop |
| `/scan` | `sensor_msgs/LaserScan` | pub: sim (frame `base_scan`), every `lidar.scan_every` steps |
| `/odom` | `nav_msgs/Odometry` | pub: sim (frame `odom`, child `base_link`) |
| `/ground_truth` | `nav_msgs/Odometry` | pub: sim (eval only) |
| `/map` | `nav_msgs/OccupancyGrid` | pub: slam (live, SLAM) or map_publisher (static, NAV). QoS: latched (TransientLocal, depth 1) |
| `/pose` | `geometry_msgs/PoseWithCovarianceStamped` | pub: mcl (NAV) |
| `/particles` | `geometry_msgs/PoseArray` | pub: mcl (viz) |
| `/plan` | `nav_msgs/Path` | pub: navigator (frame `map`) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | sub: navigator (RViz "2D Goal Pose") |

Default QoS: sensor data (`/scan`,`/odom`) = SensorDataQoS (best-effort, depth 5).
`/map` = reliable + TransientLocal (latched). Others = reliable depth 10.

---

## 5. Interfaces (`amr_interfaces`)

`srv/SaveMap.srv`
```
string path     # output stem (no extension); writes path.pgm + path.yaml
---
bool success
string message
```

`action/NavigateToGoal.action`
```
geometry_msgs/PoseStamped goal
---
bool success
float64 final_error      # metres from final pose to goal (eval)
string final_state       # SUCCEEDED | FAILED
---
string state             # IDLE|PLANNING|FOLLOWING|RECOVERY|SUCCEEDED|FAILED
float64 distance_remaining
```

---

## 6. Parameters

Each node declares ROS parameters whose names and defaults mirror the matching
`amr_core` config struct (§config.hpp). A node may load a full `AmrConfig` from a
YAML file given by the `config_file` param, then individual ROS params override.
Param files live in `amr_bringup/config/`. Default values are the struct defaults
— never reinvent constants; pull them from `amr_core::*Config`.

Key per-node params: sim → `robot.*`, `lidar.*`, `sim.*`, `seed`, `world_file`;
slam → `slam.*`, `mapping.*`; mcl → `localization.*`; navigator → `planning.*`,
`nav.*`, `robot.radius` (+ NAV planning margin 0.06 m, see architecture.md).

---

## 7. Per-package port brief

Each agent: read the cited Python source in `/home/cona/kangj/amr_stack/`, port the
algorithm to a C++ library target with gtests (faithful constants/logic), then wrap
in a rclcpp node. Reuse `amr_core` types/geometry/config. Add SPDX `Apache-2.0`
header to every file. Package builds with `colcon build --packages-select <pkg>`
and tests with `colcon test --packages-select <pkg>` in the `ros2_humble` env.

- **amr_sim** ← `amr/sim/{world,robot,lidar,simulator}.py`. Lib: World (load
  `world_file` YAML segments/circles), differential-drive exact-arc kinematics,
  vectorized ray-cast lidar (argmax first-hit), footprint collision, odom noise
  (seeded). Node: timer at `sim.dt`; sub `/cmd_vel`; pub `/scan`,`/odom`,
  `/ground_truth`, tf `odom→base_link` + static `base_link→base_scan`. gtest:
  kinematics arc, lidar ranges vs known wall, collision flag.
- **amr_mapping** ← `amr/mapping/{map_io,occupancy_grid_mapper}.py`. Lib: log-odds
  mapper (Bresenham via `amr_core::bresenham`, `l_occ/l_free/l_clamp`, p>0.65/<0.25
  thresholds) + PGM/YAML map_io (P5, flip). Node: `map_publisher` loads a map yaml
  and latches `/map`. gtest: map_io round-trip (write→read), mapper marks a wall
  occupied from synthetic scans.
- **amr_localization** ← `amr/localization/{motion_model,sensor_model,mcl}.py`.
  Lib: odometry motion model (rot1/trans/rot2 + seeded Gaussian), likelihood-field
  sensor model (chamfer `distance_field` from `amr_core`, `z_hit·exp(-d²/2σ²)+
  z_rand/range_max`), MCL (500 particles, low-variance resample at neff<0.5N,
  weighted/circular-mean estimate). Node: `mcl_node` sub `/scan`,`/map`,`/odom`;
  pub `/pose`,`/particles`, tf `map→odom`. gtest: filter converges on a known map
  (estimate error < tol) — mark `slow`.
- **amr_slam** ← `amr/slam/scan_matching_slam.py`. Lib: correlative scan matcher
  (two-stage coarse→fine, separable Gaussian blur via shifted-slice sums, keyframed
  log-odds map, `min_match_score` gate). Node: `slam_node` sub `/scan`,`/odom`; pub
  `/map`, tf `map→odom`; srv `/save_map`. gtest: matcher recovers a known small
  transform; mark `slow`.
- **amr_planning** ← `amr/planning/{costmap,astar,dwa}.py`. Lib only: costmap
  inflation (chamfer, lethal `<=robot_radius`, `exp(-cost_decay·(d-r))` band), A*
  (8-conn, octile heuristic, `step·(1+w_cost·cost)`, BFS start-escape, line-of-sight
  simplify), DWA (8×15 window, unicycle rollout, 4 weighted objectives). gtest:
  A* finds path around a wall + avoids lethal; DWA picks sane v,w toward carrot;
  costmap lethal/decay bands.
- **amr_navigation** ← `amr/navigation/navigator.py`. Node: FSM (IDLE/PLANNING/
  FOLLOWING/RECOVERY/SUCCEEDED/FAILED) per the exact transition table; action server
  `NavigateToGoal` + `/goal_pose` sub; links `amr_planning`; costmap from `/map`;
  pose from tf `map→base_link`; pub `/cmd_vel`,`/plan`. Replan every
  `replan_period` or on blocked path; recovery rotate/backup by sim clock. gtest on
  FSM transitions with a stub planner.
- **amr_bringup**: `launch/slam.launch.py` (sim+slam+rviz), `nav.launch.py`
  (sim+map_publisher+mcl+navigator+rviz), `demo.launch.py`. `config/*.yaml` params,
  `rviz/amr.rviz`. **launch_testing** `test/test_demo.py`: SLAM a mission, save map,
  then nav two goals on the saved map, assert success + GT error < 0.6 m + no
  collision — the ROS 2 analogue of `amr demo` → DEMO PASS.
- **amr_hri**: `rviz_common::Panel` plugin "AMR Console" — goal x/y entry (pub
  `/goal_pose`), E-STOP toggle (pub zero `/cmd_vel` / `std_srvs`), nav-state label
  (sub action feedback). pluginlib export + `plugin_description.xml`. This is the
  HRI deliverable (RViz2 replaces the tkinter GUI's role).

---

## 8. Build & verification (Humble, RoboStack `ros2_humble` env — no Docker/root)

```
micromamba run -n ros2_humble bash -c 'cd ros2_ws && unset PYTHONPATH && \
  colcon build && colcon test && colcon test-result --all'
```
Gate = full workspace builds + all gtests pass + `launch_testing` demo passes.
Also ship `docker/humble.Dockerfile` (FROM `osrf/ros:humble-desktop`) + a
`scripts/verify.sh` for reproducible container verification on a real machine.

---

## 9. Determinism & faithfulness rules

- Pull every numeric constant from `amr_core::*Config` defaults; never hardcode.
- Match the Python algorithm step-for-step; if a deviation is needed, document it
  in the package README and keep the gtest assertion strength.
- Seed all RNG from the `seed` param; no global/un-seeded randomness.

---

## 10. Humble → Jazzy deltas (applied on the `jazzy` branch)

- `package.xml`: bump tested distro; same `format=3`.
- tf2 headers: prefer `.hpp` (`tf2_geometry_msgs/tf2_geometry_msgs.hpp` exists in
  both; Jazzy drops the deprecated `.h`). Use `.hpp` everywhere (works on both).
- `rclcpp` API is stable across Humble→Jazzy for what we use; no code change
  expected in nodes.
- `ament_cmake`/`rosidl` generator: Jazzy uses newer CMake; keep
  `cmake_minimum_required(VERSION 3.8)` (bump to 3.10+ to silence deprecation).
- message_filters / rclcpp_action signatures unchanged for our usage.
- Docker base → `osrf/ros:jazzy-desktop` (Ubuntu 24.04).
- RViz plugin API (`rviz_common::Panel`) unchanged across the two.
- Verify on Jazzy via the shipped `docker/jazzy.Dockerfile`; on this box the Jazzy
  branch is static-validated (no Jazzy env on Ubuntu 20.04).

---

## 11. amr_api data model ↔ ROS topic mapping (CONTRACT_VERSION 1.0)

`amr_api` (version 0.1.0) standardizes module I/O as pure data. A `core` running
as or beside a node maps that data onto the frozen §4 topics; the ROS nodes
themselves are unchanged ("freeze + version"). Each node logs `amr_api::VERSION`
+ `CONTRACT_VERSION` once at startup.

| amr_api datum | ROS topic (§4) | Direction |
|---|---|---|
| `SlamInput.scan` / `LocalizerInput.scan` / `MapperInput.scan` | `/scan` | in |
| `*.odom_delta` (derived between consecutive `/odom`) | `/odom` | in |
| `ISlam::map()` / `IMapper::map()` | `/map` | out |
| `LocalizerOutput.pose` | `/pose` | out |
| `LocalizerOutput.cloud` | `/particles` | out |
| `BehaviorOutput.cmd` | `/cmd_vel` | out |
| `BehaviorOutput.plan` | `/plan` | out |
| `IBehavior::set_goal` | `/goal_pose` | in |

The six interfaces (`ISlam`, `ILocalizer`, `IMapper`, `IGlobalPlanner`,
`ILocalPlanner`, `IBehavior`) live in `amr_api`; each module package ships a thin
adapter that delegates to its existing, gtest-verified class. Coordinate/units/
occupancy conventions (§2) and QoS (§4) are unchanged; the adapters carry
`amr_core` types, so no conversion semantics are added. See
`src/amr_api/README.md` and `docs/CORE_INTEGRATION.md`.
