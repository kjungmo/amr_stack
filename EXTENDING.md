# Extending the AMR Stack (ROS 2 Humble, C++)

This guide is for developers who want to **add a new package/module** or **extend
existing functionality** in this workspace. It is a companion to
[`CONTRACT.md`](CONTRACT.md), which is the frozen architecture and the single
source of truth. Where this guide and the contract appear to disagree, the
contract wins — fix this guide.

Everything here is copy-paste runnable against this tree. The worked example
(`amr_perception`) is modelled directly on the real `amr_sim` / `amr_mapping`
packages; only the algorithm is new.

---

## 1. Overview & philosophy

**Algorithm = library; ROS wiring = thin node.**

Every package is split into two parts:

1. A **`*_lib` target** — a plain C++17 library that holds the algorithm. It has
   **no `rclcpp` dependency** and is unit-tested with `gtest` *without a running
   node*. This is where fidelity to the original pure-Python `amr_stack` lives:
   the constants, the math, the step-for-step logic. Because it runs without ROS,
   it is fast to test and verifiable in isolation.
2. A **thin rclcpp node executable** that wraps the library: it declares ROS
   parameters, sets up publishers/subscribers/timers/tf with the correct QoS,
   converts ROS messages to/from `amr_core` types, and calls into the library.

Why this split:

- It mirrors how the Python original is unit-tested, so each package can be
  ported and verified **package-by-package** (see `CONTRACT.md` §1, §7).
- The numeric heart of the system is exercised by deterministic gtests that never
  touch the DDS layer, so regressions surface as a unit-test failure, not a flaky
  integration test.
- The node becomes boring glue — easy to read, easy to review, hard to hide a bug
  in.

`amr_core` is the shared foundation (types, geometry, config). `amr_planning`
and `amr_navigation`'s FSM library are pure libraries; the rest pair a library
with a node. `amr_navigation` and `amr_slam` additionally consume the custom
interfaces in `amr_interfaces`.

---

## 2. Repository layout & dependency graph

Ten packages, with **strictly one-way, downward** dependencies (from
`CONTRACT.md` §1). A package may only depend on packages above it:

```
amr_core          lib: geometry, types, config (yaml-cpp)
amr_interfaces    msgs/srv/action (rosidl)
amr_sim           lib + sim_node                 [amr_core, amr_mapping]
amr_mapping       lib + map_publisher            [amr_core]
amr_localization  lib + mcl_node                 [amr_core, amr_mapping]
amr_slam          lib + slam_node                [amr_core, amr_mapping, amr_interfaces]
amr_planning      lib only (costmap + A* + DWA)  [amr_core]
amr_navigation    lib (FSM) + navigator_node     [amr_core, amr_planning, amr_interfaces]
amr_bringup       launch + params + rviz + launch_testing  [all nodes]
amr_hri           rviz panel plugin              [rviz_common, pluginlib, amr_interfaces]
```

Build order is resolved by `colcon` from the `package.xml` `<depend>` graph;
you do not order it by hand, but you **must not** introduce an upward or cyclic
dependency. `amr_core` builds first and is gtest-verified before anything else.

---

## 3. Conventions you MUST follow

These are frozen by `CONTRACT.md` §2–§6. New code obeys them exactly.

### Coordinate & data conventions (§2)

- **World frame:** x right, y up, metres. `theta` is CCW radians wrapped to
  `(-pi, pi]` (use `amr_core::wrap_angle`).
- **Robot frame:** x forward, y left. Lidar beam angle `0` = robot forward.
- **Grid:** `data` is row-major; `col → x`, `row → y`; **row 0 = minimum-y**. No
  vertical flip except PGM image I/O (image row 0 = top, so flip on read/write).
- **Cell ↔ world:** cell→world returns cell *centres*;
  `col = floor((x - origin_x) / res)`. Use `OccupancyGrid::world_to_grid` /
  `grid_to_world` / `in_bounds` from `amr_core/types.hpp` — never re-derive them.
- **Occupancy `int8`:** `-1` unknown, `0` free, `100` occupied; a cell is
  occupied iff its value `>= 65`.
- **SI units everywhere:** m, rad, s, m/s, rad/s.

### tf frames (§3, REP-105)

```
map --(slam_node OR mcl_node)--> odom --(sim_node)--> base_link --(static)--> base_scan
```

- `sim_node` publishes `odom→base_link` on `/tf` and the static
  `base_link→base_scan` on `/tf_static`.
- Exactly one of `slam_node` (SLAM) / `mcl_node` (NAV) publishes the `map→odom`
  correction. Ground-truth pose is a topic (`/ground_truth`) only — **never** tf.

### Topic QoS (§4)

| Topic class | QoS |
|---|---|
| Sensor data (`/scan`, `/odom`) | `rclcpp::SensorDataQoS()` (best-effort, depth 5) |
| `/map` (latched) | `reliable` + `transient_local`, depth 1 |
| Everything else (`/cmd_vel`, `/plan`, `/pose`, `/particles`, `/goal_pose`) | `reliable`, depth 10 |

In code, the latched `/map` QoS is written as:

```cpp
rclcpp::QoS map_qos(rclcpp::KeepLast(1));
map_qos.reliable().transient_local();
```

A subscriber's QoS must match the publisher's durability, or it will silently
receive nothing. Always subscribe to `/map` with `transient_local`.

### Determinism (§2.7, §9)

Every stochastic node takes a `seed` parameter (default `42`) and owns its own
`std::mt19937`, seeded from that parameter. **No global RNG, no unseeded
randomness.** The library takes the engine by reference (e.g.
`Lidar(const LidarConfig&, std::mt19937& rng)`); the node owns the engine and
passes it in.

### SPDX header

The **first line of every source file** (`.hpp`, `.cpp`, launch `.py`, YAML
that you author) is:

```cpp
// SPDX-License-Identifier: Apache-2.0
```

(Use `# SPDX-License-Identifier: Apache-2.0` for Python/YAML.)

### Config-struct pattern (§6)

Defaults live **once**, in an `amr_core::*Config` struct in
[`amr_core/include/amr_core/config.hpp`](src/amr_core/include/amr_core/config.hpp).
A node:

1. declares ROS parameters whose **names mirror the dotted struct path** (e.g.
   `lidar.num_beams` ↔ `LidarConfig::num_beams`),
2. uses the **struct default** as each parameter's default value (never a
   hardcoded literal),
3. may load a full `AmrConfig` from a YAML file given by a `config_file` param,
   after which individual ROS params override.

Pull every numeric constant from the struct defaults. If you need a new constant,
add it to the matching `*Config` struct — do not sprinkle literals in nodes.

---

## 4. Adding a new package — worked walk-through

We add **`amr_perception`**: a `BlobDetector` algorithm library that scans a
laser scan for the nearest dense cluster of returns (a "blob") and reports its
bearing/range, plus a thin `blob_node` that subscribes to `/scan` and publishes a
marker pose. This exercises every part of the pattern.

> The example is deliberately small but real: it reads `/scan` with sensor QoS,
> reuses `amr_core::LaserScan`, mirrors a config struct, seeds nothing (it is
> deterministic) but shows where the seed would go, and ships a gtest.

### 4.1 Directory skeleton

```
src/amr_perception/
├── CMakeLists.txt
├── package.xml
├── include/amr_perception/blob_detector.hpp
├── src/blob_detector.cpp
├── src/blob_node.cpp
└── test/test_blob_detector.cpp
```

### 4.2 `CMakeLists.txt`

Modelled on the real `amr_mapping/CMakeLists.txt` (lib + node, no custom
interfaces). Note: the lib depends only on `amr_core`; the node adds `rclcpp` +
message packages.

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Algorithm library (node-free, unit-testable). ---
add_library(amr_perception_lib
  src/blob_detector.cpp
)
target_include_directories(amr_perception_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_perception_lib PUBLIC cxx_std_17)
ament_target_dependencies(amr_perception_lib amr_core)

# --- Thin ROS node wrapping the library. ---
add_executable(blob_node src/blob_node.cpp)
target_link_libraries(blob_node amr_perception_lib)
ament_target_dependencies(blob_node
  amr_core
  rclcpp
  sensor_msgs
  geometry_msgs
)

# --- Install. ---
install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_perception_lib
  EXPORT export_amr_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS blob_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_amr_perception HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_blob_detector test/test_blob_detector.cpp)
  target_link_libraries(test_blob_detector amr_perception_lib)
  ament_target_dependencies(test_blob_detector amr_core)
endif()

ament_package()
```

Key rules visible above (all copied from the real packages):

- `add_library(<pkg>_lib ...)` with `target_include_directories(... PUBLIC
  BUILD_INTERFACE/INSTALL_INTERFACE)` so headers resolve both in-tree and when
  installed.
- The **library** only pulls `amr_core`; ROS/message deps go on the **node**.
- `install(TARGETS <lib> EXPORT export_<pkg> ...)` plus
  `ament_export_targets(export_<pkg> HAS_LIBRARY_TARGET)` and
  `ament_export_dependencies(amr_core)` — this is what lets a *downstream*
  package `find_package(amr_perception REQUIRED)` and link the lib.
- Node executables install to `lib/${PROJECT_NAME}` so `ros2 run amr_perception
  blob_node` finds them.
- The gtest links `amr_perception_lib` only (no node, no rclcpp).

### 4.3 `package.xml` (format 3)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_perception</name>
  <version>0.1.0</version>
  <description>
    Lidar blob detector for the AMR stack: a node-free BlobDetector library that
    finds the nearest dense cluster in a LaserScan, plus a thin blob_node that
    subscribes to /scan and publishes the blob bearing/range as a PoseStamped.
  </description>
  <maintainer email="kangjmo91@gmail.com">Kang Jung Mo</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>amr_core</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`<depend>` entries must mirror the `find_package(...)` calls in
`CMakeLists.txt`. `ament_cmake_gtest` is a `<test_depend>`.

### 4.4 The algorithm library

`include/amr_perception/blob_detector.hpp` — namespaced, SPDX header, reuses
`amr_core` types, no rclcpp:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Lidar blob detector: find the nearest dense cluster of valid returns in a
// LaserScan and report its mean bearing/range. Node-free and deterministic.
#pragma once

#include "amr_core/types.hpp"

namespace amr_perception {

/// Tuning for BlobDetector. (When this graduates to a real feature, lift these
/// defaults into an amr_core::PerceptionConfig struct in config.hpp — see §3.)
struct BlobConfig {
  double cluster_gap = 0.30;  // max range jump (m) between adjacent beams in a blob
  int min_beams = 3;          // minimum beams to count as a blob
};

/// Result of a detection. `found == false` means no qualifying blob.
struct Blob {
  bool found = false;
  double bearing = 0.0;  // rad in the robot/scan frame (0 = forward)
  double range = 0.0;    // m, mean over the blob's beams
  int num_beams = 0;
};

/// Detect the nearest dense cluster of valid beams in `scan`.
class BlobDetector {
 public:
  explicit BlobDetector(const BlobConfig& cfg) : cfg_(cfg) {}

  Blob detect(const amr_core::LaserScan& scan) const;

 private:
  BlobConfig cfg_;
};

}  // namespace amr_perception
```

`src/blob_detector.cpp` — reuses `LaserScan::angles()` and `valid_mask()` from
`amr_core/types.hpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
#include "amr_perception/blob_detector.hpp"

#include <cmath>
#include <vector>

namespace amr_perception {

Blob BlobDetector::detect(const amr_core::LaserScan& scan) const {
  const std::vector<double> ang = scan.angles();
  const std::vector<char> valid = scan.valid_mask();

  Blob best;
  double best_range = std::numeric_limits<double>::infinity();

  int i = 0;
  const int n = scan.num_beams();
  while (i < n) {
    if (!valid[static_cast<std::size_t>(i)]) {
      ++i;
      continue;
    }
    // Grow a run of adjacent valid beams whose range jumps stay < cluster_gap.
    int j = i + 1;
    double sum_range = scan.ranges[static_cast<std::size_t>(i)];
    double sum_bearing = ang[static_cast<std::size_t>(i)];
    int count = 1;
    while (j < n && valid[static_cast<std::size_t>(j)] &&
           std::abs(scan.ranges[static_cast<std::size_t>(j)] -
                    scan.ranges[static_cast<std::size_t>(j - 1)]) <
               cfg_.cluster_gap) {
      sum_range += scan.ranges[static_cast<std::size_t>(j)];
      sum_bearing += ang[static_cast<std::size_t>(j)];
      ++count;
      ++j;
    }
    if (count >= cfg_.min_beams) {
      const double mean_range = sum_range / count;
      if (mean_range < best_range) {
        best_range = mean_range;
        best.found = true;
        best.range = mean_range;
        best.bearing = sum_bearing / count;
        best.num_beams = count;
      }
    }
    i = j;
  }
  return best;
}

}  // namespace amr_perception
```

### 4.5 The thin node

`src/blob_node.cpp` — declares params mirroring `BlobConfig`, subscribes to
`/scan` with **`SensorDataQoS`**, publishes a `PoseStamped`. Compare with
`amr_sim/src/sim_node.cpp` for the parameter-declaration and QoS idioms.

```cpp
// SPDX-License-Identifier: Apache-2.0
// blob_node: thin rclcpp wrapper around BlobDetector.
//
// Subscribes /scan (SensorDataQoS, frame base_scan) and publishes the detected
// blob as /blob (geometry_msgs/PoseStamped, frame base_scan): position is the
// blob point (range, bearing), orientation encodes the bearing.
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "amr_core/types.hpp"
#include "amr_perception/blob_detector.hpp"

class BlobNode : public rclcpp::Node {
 public:
  BlobNode() : rclcpp::Node("blob_node") {
    // --- Parameters mirror amr_perception::BlobConfig defaults. ---
    amr_perception::BlobConfig def;
    // A seed param is declared even though this node is deterministic, so the
    // convention holds if randomness is ever added (CONTRACT §2.7).
    seed_ = declare_parameter<int>("seed", 42);
    cfg_.cluster_gap =
        declare_parameter<double>("perception.cluster_gap", def.cluster_gap);
    cfg_.min_beams =
        declare_parameter<int>("perception.min_beams", def.min_beams);
    scan_frame_ = declare_parameter<std::string>("scan_frame", "base_scan");

    detector_ = std::make_unique<amr_perception::BlobDetector>(cfg_);

    // --- QoS: sensor data best-effort depth 5 (CONTRACT §4). ---
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&BlobNode::on_scan, this, std::placeholders::_1));

    // /blob is not sensor data -> reliable depth 10 (CONTRACT §4).
    blob_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("/blob", 10);

    RCLCPP_INFO(get_logger(), "blob_node up: seed=%d", seed_);
  }

 private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    amr_core::LaserScan scan;
    scan.angle_min = msg->angle_min;
    scan.angle_increment = msg->angle_increment;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;
    scan.ranges.assign(msg->ranges.begin(), msg->ranges.end());

    const amr_perception::Blob blob = detector_->detect(scan);
    if (!blob.found) {
      return;
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = scan_frame_;
    out.pose.position.x = blob.range * std::cos(blob.bearing);
    out.pose.position.y = blob.range * std::sin(blob.bearing);
    out.pose.orientation.z = std::sin(blob.bearing / 2.0);
    out.pose.orientation.w = std::cos(blob.bearing / 2.0);
    blob_pub_->publish(out);
  }

  int seed_{42};
  amr_perception::BlobConfig cfg_{};
  std::string scan_frame_;
  std::unique_ptr<amr_perception::BlobDetector> detector_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr blob_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlobNode>());
  rclcpp::shutdown();
  return 0;
}
```

### 4.6 The gtest

`test/test_blob_detector.cpp` — tests the library directly, no node, mirroring
`amr_sim/test/test_sim.cpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Unit tests for BlobDetector: a synthetic scan with one near cluster.
#include <cmath>

#include <gtest/gtest.h>

#include "amr_core/types.hpp"
#include "amr_perception/blob_detector.hpp"

namespace {

// A 16-beam scan over [-pi/2, pi/2). All beams at range_max (no return) except
// a 3-beam cluster around forward (beam 8) at ~2.0 m.
amr_core::LaserScan make_scan() {
  amr_core::LaserScan s;
  s.angle_min = -M_PI / 2.0;
  s.angle_increment = M_PI / 16.0;  // 16 beams across pi
  s.range_min = 0.12;
  s.range_max = 8.0;
  s.ranges.assign(16, 8.0);
  s.ranges[7] = 2.01;
  s.ranges[8] = 2.00;
  s.ranges[9] = 2.02;
  return s;
}

}  // namespace

TEST(BlobDetector, FindsNearCluster) {
  amr_perception::BlobDetector det(amr_perception::BlobConfig{});
  const amr_perception::Blob blob = det.detect(make_scan());

  ASSERT_TRUE(blob.found);
  EXPECT_EQ(blob.num_beams, 3);
  EXPECT_NEAR(blob.range, 2.01, 0.02);
  // Beam 8 is the centre; its angle is angle_min + 8 * increment ~= 0.
  EXPECT_NEAR(blob.bearing, 0.0, M_PI / 16.0);
}

TEST(BlobDetector, NoClusterWhenAllNoReturn) {
  amr_core::LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 8.0;
  s.range_min = 0.12;
  s.range_max = 8.0;
  s.ranges.assign(8, 8.0);  // all at range_max -> invalid (valid_mask filters)

  amr_perception::BlobDetector det(amr_perception::BlobConfig{});
  EXPECT_FALSE(det.detect(s).found);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

Build and test just this package:

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_ros2_dev/humble_ws && \
  unset PYTHONPATH && \
  colcon build --packages-select amr_perception && \
  colcon test --packages-select amr_perception && \
  colcon test-result --all'
```

---

## 5. Adding a new message / service / action

All custom interfaces live in **`amr_interfaces`** (see
[`CONTRACT.md`](CONTRACT.md) §5). Two sides: defining it, and consuming it.

### 5.1 Define the interface

1. Add the definition file under the matching subdir, e.g.
   `src/amr_interfaces/srv/ResetPose.srv`:

   ```
   geometry_msgs/PoseStamped pose
   ---
   bool success
   string message
   ```

   (Services use one `---`; actions use two — goal/result/feedback — as in
   `action/NavigateToGoal.action`.)

2. Register it in `src/amr_interfaces/CMakeLists.txt`'s
   `rosidl_generate_interfaces` call, and declare any message package it
   references under `DEPENDENCIES`:

   ```cmake
   find_package(geometry_msgs REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "srv/SaveMap.srv"
     "srv/ResetPose.srv"               # <-- added
     "action/NavigateToGoal.action"
     DEPENDENCIES geometry_msgs
   )
   ```

3. If the new interface pulls a message package not already listed, add a
   `<depend>` for it in `src/amr_interfaces/package.xml`. The package already has
   the rosidl plumbing — keep it intact:

   ```xml
   <buildtool_depend>rosidl_default_generators</buildtool_depend>
   <depend>geometry_msgs</depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   <exec_depend>rosidl_default_runtime</exec_depend>
   ```

### 5.2 Consume the interface from another package

This is exactly how `amr_navigation` uses `NavigateToGoal` and `amr_slam` uses
`SaveMap`. In the consumer:

1. `CMakeLists.txt`: `find_package(amr_interfaces REQUIRED)` and list it in the
   node's `ament_target_dependencies`:

   ```cmake
   find_package(amr_interfaces REQUIRED)

   ament_target_dependencies(navigator_node
     amr_core
     amr_planning
     amr_interfaces        # <-- generated headers + libs resolve via this
     rclcpp
     rclcpp_action
     # ...
   )
   ```

   In Humble, `ament_target_dependencies(... amr_interfaces)` already exposes the
   generated headers and links the typesupport — **no separate
   `rosidl_target_interfaces()` call is needed** for an external consumer.

2. `package.xml`: `<depend>amr_interfaces</depend>`.

3. In the node, include the generated header (note the **snake_case** filename
   derived from the CamelCase type) and reference the C++ type:

   ```cpp
   #include "amr_interfaces/action/navigate_to_goal.hpp"   // NavigateToGoal.action
   // or
   #include "amr_interfaces/srv/save_map.hpp"              // SaveMap.srv

   using NavigateToGoal = amr_interfaces::action::NavigateToGoal;
   // service request/response types:
   //   amr_interfaces::srv::SaveMap::Request / ::Response
   ```

`SaveMap.srv` → `amr_interfaces/srv/save_map.hpp`;
`NavigateToGoal.action` → `amr_interfaces/action/navigate_to_goal.hpp`.

---

## 6. Wiring into bringup

`amr_bringup` holds launch files, the unified params YAML, and the RViz config
([`src/amr_bringup/`](src/amr_bringup)).

### 6.1 Add the node to a launch file

Follow the `launch_ros.actions.Node` pattern from
[`launch/nav.launch.py`](src/amr_bringup/launch/nav.launch.py). Add a `Node` and
append it to the returned `LaunchDescription`:

```python
from launch_ros.actions import Node

blob_node = Node(
    package="amr_perception",
    executable="blob_node",
    name="blob_node",
    output="screen",
    parameters=[params_file],   # the shared amr.yaml
)

# ... then include blob_node in the LaunchDescription([...]) list.
```

`params_file` is the package-share `config/amr.yaml`
(`get_package_share_directory("amr_bringup") / "config" / "amr.yaml"`). Pass
per-launch overrides as a second dict entry, e.g.
`parameters=[params_file, {"seed": seed}]`, exactly as `sim_node` does.

### 6.2 Add params under a node-named block

In [`config/amr.yaml`](src/amr_bringup/config/amr.yaml), add a block keyed by the
node **name** (the `name=` you gave the `Node`, which equals the node's runtime
name). Mirror the config-struct dotted names:

```yaml
# ── blob_node ────────────────────────────────────────────────────────────────
blob_node:
  ros__parameters:
    seed: 42
    perception.cluster_gap: 0.30
    perception.min_beams:   3
```

Only parameters the node `declare_parameter`s can be resolved here. Keep the
comment header noting these mirror the config-struct defaults.

### 6.3 (Optional) RViz display

To visualise `/blob`, add a display to
[`rviz/amr.rviz`](src/amr_bringup/rviz/amr.rviz) (a `rviz/Pose` display on topic
`/blob`, fixed frame `map`). Easiest path: launch the stack, add the display
interactively in RViz, then **Save Config** over `amr.rviz`.

---

## 7. Testing & verification

Two tiers, both run by `colcon test`:

### Tier 1 — gtest unit tests (library, no node)

Defined in each package's `if(BUILD_TESTING) ... ament_add_gtest(...) endif()`
block (§4.2). They link only the `*_lib` and run without ROS. This is where you
prove the algorithm matches the Python original. Run for one package:

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_ros2_dev/humble_ws && \
  unset PYTHONPATH && \
  colcon test --packages-select amr_perception && colcon test-result --all'
```

### Tier 2 — launch_testing integration tests

Live in `amr_bringup/test/` and are registered via `add_launch_test(...)` in
[`amr_bringup/CMakeLists.txt`](src/amr_bringup/CMakeLists.txt):

```cmake
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_demo.py TARGET test_demo_launch TIMEOUT 300)
endif()
```

Each test provides a `@pytest.mark.launch_test` `generate_test_description()`
that launches the real nodes, plus `unittest.TestCase` classes that drive the
stack and assert behaviour — see
[`test/test_nav.py`](src/amr_bringup/test/test_nav.py) for the canonical pattern
(action client, ground-truth subscriber with the matching best-effort QoS,
`SUCCEEDED` + believed-error + ground-truth-error + no-collision assertions, and a
`@launch_testing.post_shutdown_test()` exit-code check).

### Full-workspace gate (the CONTRACT §8 command)

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_ros2_dev/humble_ws && \
  unset PYTHONPATH && \
  colcon build && colcon test && colcon test-result --all'
```

The gate is: **whole workspace builds + every gtest passes + the
`launch_testing` demo passes.** `unset PYTHONPATH` is required so the RoboStack
`ros2_humble` env's Python is used (a stray host `PYTHONPATH` breaks the build).
Remember to `source install/setup.bash` (inside the env) before `ros2 run` /
`ros2 launch` of your new node.

---

## 8. Faithfulness rules

From `CONTRACT.md` §9. When porting or extending an algorithm:

- **Pull every numeric constant from an `amr_core::*Config` default — never
  hardcode.** A new tunable goes into the matching config struct (and into
  `amr.yaml` as a mirrored param), not into a node literal. The one sanctioned
  in-node constant in this tree (`kNavPlanningMargin = 0.06` in
  `navigator_node.cpp`) is documented against `architecture.md` and `CONTRACT.md`
  §6 — match that bar if you ever add another.
- **Match the source algorithm step-for-step.** Keep the same ordering, the same
  thresholds, the same edge-case handling as the Python original.
- **Seed all RNG from the `seed` param.** Each node owns one `std::mt19937`; the
  library borrows it by reference. No global or unseeded randomness.
- **Document any deviation** in the package README (or a top-of-file comment) and
  keep the gtest assertion strength — a deviation that weakens a test is a
  regression.

---

## 9. Checklist — "I added a new package"

Before you open a PR against the `humble` branch:

- [ ] **SPDX header** `// SPDX-License-Identifier: Apache-2.0` is the first line
      of every `.hpp`/`.cpp` (and `# ...` for `.py`/`.yaml`).
- [ ] **Library/node split**: algorithm lives in `<pkg>_lib` (no `rclcpp`); the
      node is a thin rclcpp wrapper that only does ROS glue + type conversion.
- [ ] **gtest** exercises the library directly (links `<pkg>_lib`, no node) in an
      `if(BUILD_TESTING) ament_add_gtest(...) endif()` block.
- [ ] **Params mirror the config struct**: every `declare_parameter` uses the
      dotted struct path as its name and the `amr_core::*Config` default as its
      default; no hardcoded constants.
- [ ] **Correct QoS**: sensor topics use `rclcpp::SensorDataQoS()`; `/map` is
      `reliable().transient_local()` depth 1; everything else is reliable depth 10.
- [ ] **Determinism**: a `seed` param exists; the node owns its `std::mt19937`;
      no global/unseeded RNG.
- [ ] **CMake exports**: `install(... EXPORT export_<pkg>)` +
      `ament_export_targets(export_<pkg> HAS_LIBRARY_TARGET)` +
      `ament_export_dependencies(...)`; node installs to `lib/${PROJECT_NAME}`.
- [ ] **`package.xml`** (format 3) `<depend>`/`<test_depend>` match the
      `find_package` calls; dependencies stay strictly downward (no cycles, no
      upward deps).
- [ ] **Added to bringup**: a `Node` in the relevant launch file + a node-named
      params block in `config/amr.yaml` (+ optional RViz display).
- [ ] **Builds + tests pass** under the CONTRACT §8 command
      (`colcon build && colcon test && colcon test-result --all` in the
      `ros2_humble` env with `unset PYTHONPATH`).
- [ ] **Consistent with `CONTRACT.md`** — frames, units, occupancy semantics,
      topic/QoS table, and the faithfulness rules.
```
