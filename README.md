# AMR Stack — ROS 2 Jazzy (C++)

A complete, self-contained Autonomous Mobile Robot (AMR) software stack for **ROS 2
Jazzy**, written in idiomatic C++. It is a faithful port of the pure-Python
[`amr_stack`](https://github.com/kjungmo/amr_stack) reference implementation (the
`main` branch): same algorithms, same constants, same coordinate conventions,
re-expressed as proper ROS 2 packages (rclcpp nodes, custom messages/actions,
tf2, launch files, RViz, gtest + launch_testing).

> Branches: **`main`** — the Python reference. **`humble`** — the C++/ROS 2
> Humble port. **`jazzy`** (this branch) — the same workspace targeting ROS 2 Jazzy
> (CONTRACT §10 deltas applied: `cmake_minimum_required` 3.10, tf2 `.hpp` headers,
> Ubuntu 24.04 base).

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

### With a native ROS 2 Jazzy install (Ubuntu 24.04)

```bash
mkdir -p ~/amr_ws/src && cp -r . ~/amr_ws/   # this repo is a colcon workspace
cd ~/amr_ws
source /opt/ros/jazzy/setup.bash
colcon build
colcon test && colcon test-result --all
```

### With Docker (no local ROS needed)

```bash
docker build -f docker/jazzy.Dockerfile -t amr_stack:jazzy .
docker run --rm amr_stack:jazzy        # builds + runs all tests
```

### On a host without native Jazzy (RoboStack / micromamba)

```bash
micromamba create -n ros2_jazzy -c robostack-jazzy ros-jazzy-desktop
micromamba run -n ros2_jazzy bash -c 'unset PYTHONPATH && colcon build && colcon test'
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

The implementation on this branch is byte-for-byte identical to the `humble`
branch apart from the CONTRACT §10 deltas (CMake floor, header extensions, Docker
base), where the full suite is verified:

```
52/52 tests pass on ROS 2 Humble (43 gtest unit + test_slam + test_nav full-office tour)
```

On ROS 2 Jazzy the stack is built and tested via `docker/jazzy.Dockerfile`
(`docker run --rm amr_stack:jazzy`); rclcpp/tf2/rosidl APIs used here are stable
across Humble→Jazzy, so no source changes beyond §10 are required.

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
