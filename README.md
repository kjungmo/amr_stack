# AMR Stack — ROS 2 Humble (C++)

A complete, self-contained Autonomous Mobile Robot (AMR) software stack for **ROS 2
Humble**, written in idiomatic C++. It is a faithful port of the pure-Python
[`amr_stack`](https://github.com/kjungmo/amr_stack) reference implementation (the
`main` branch): same algorithms, same constants, same coordinate conventions,
re-expressed as proper ROS 2 packages (rclcpp nodes, custom messages/actions,
tf2, launch files, RViz, gtest + launch_testing).

> Branches: **`main`** — the Python reference. **`humble`** (this branch) — the
> C++/ROS 2 Humble port. **`jazzy`** — the same workspace targeting ROS 2 Jazzy.

## What's inside

A 2-D differential-drive robot with a 360° lidar, simulated, mapped (SLAM),
localized (MCL), and navigated (A* + DWA) end to end — plus an RViz HRI panel.

| Package | Role |
|---|---|
| `amr_core` | Shared library: geometry, types, config (yaml-cpp). No ROS deps. |
| `amr_interfaces` | `SaveMap` service + `NavigateToGoal` action. |
| `amr_sim` | 2-D simulator node (`/scan`, `/odom`, `/ground_truth`, tf) + `world_to_map` tool. |
| `amr_mapping` | Log-odds occupancy mapper + map I/O (ROS map_server PGM/YAML); `map_publisher` node. |
| `amr_localization` | Likelihood-field MCL; `mcl_node`. |
| `amr_slam` | Correlative scan-matching SLAM; `slam_node` (+ `/save_map`). |
| `amr_planning` | Costmap inflation + A* + DWA (library). |
| `amr_navigation` | Navigation FSM + `NavigateToGoal` action server; `navigator_node`. |
| `amr_bringup` | Launch files, params, RViz config, integration tests. |
| `amr_hri` | RViz `Panel` plugin (goal entry, E-STOP, nav-state). |

**Design rule:** *algorithm = library; ROS wiring = thin node.* Every package has
a unit-testable `*_lib` target (gtests, no running node) plus an rclcpp node that
wraps it. See [`CONTRACT.md`](CONTRACT.md) for the frozen architecture spec and
[`EXTENDING.md`](EXTENDING.md) for how to add your own modules.

## Build & test

### With a native ROS 2 Humble install (Ubuntu 22.04)

```bash
mkdir -p ~/amr_ws/src && cp -r . ~/amr_ws/   # this repo is a colcon workspace
cd ~/amr_ws
source /opt/ros/humble/setup.bash
colcon build
colcon test && colcon test-result --all
```

### With Docker (no local ROS needed)

```bash
docker build -f docker/humble.Dockerfile -t amr_stack:humble .
docker run --rm amr_stack:humble        # builds + runs all tests
```

### On a host without native Humble (RoboStack / micromamba)

```bash
micromamba create -n ros2_humble -c robostack-staging ros-humble-desktop
micromamba run -n ros2_humble bash -c 'unset PYTHONPATH && colcon build && colcon test'
```

## Run the demo

```bash
# SLAM: drive the robot, build a map live, save it.
ros2 launch amr_bringup slam.launch.py

# NAV: localize on a prebuilt map and navigate to goals (RViz "2D Goal Pose").
ros2 launch amr_bringup nav.launch.py map_file:=<path-to-map.yaml>
```

Render a perfect map straight from a world file (handy for NAV without SLAM):

```bash
ros2 run amr_sim world_to_map src/amr_bringup/config/office.world.yaml office_map
```

## Verification

The build gate is: **full workspace builds + all gtest unit tests pass + the
launch_testing integration tests pass.** The integration tests
(`amr_bringup/test/`) are the ROS 2 analogue of the Python `amr demo` → DEMO PASS:

- `test_slam.py` — sim + SLAM build a map of the office and save a valid map
  (asserting the saved PGM has real walls and interior).
- `test_nav.py` — sim + map_publisher + MCL + navigator localize on a known map
  and drive a **full-office goal tour**: west-room north → middle room (through the
  wall-A door gap) → east room (through the wall-B gap), asserting each goal is
  reached with ground-truth error < 0.6 m and no collision.

Verified in the RoboStack `ros2_humble` environment:

```
43/43 gtest unit tests pass (amr_core, sim, mapping, localization, slam, planning, navigation)
test_slam : office map built + saved (occupied + free cells present)
test_nav  : 3/3 goals SUCCEEDED — GT error ~0.25 m each, no collision
```

## Scope & limitations

This is a compact, dependency-light **educational** stack — a faithful, readable
port of the reference algorithms, not a hardened production navigation system. It
runs entirely in its own 2-D simulator (no Gazebo/hardware drivers). The MCL is a
basic fixed-size likelihood-field filter seeded from a known start pose; global
("kidnapped robot") recovery, KLD-adaptive resampling, and dynamic-obstacle
avoidance are intentionally out of scope and are natural extensions (see
[`EXTENDING.md`](EXTENDING.md)). All tuning lives in `amr_bringup/config/amr.yaml`.

## License

[Apache-2.0](LICENSE) © 2026 Kang Jung Mo.
