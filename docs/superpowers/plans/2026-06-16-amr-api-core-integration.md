# amr_api Core-Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a node-free `amr_api` interface layer plus thin per-module adapters so a private `core` orchestrator can drive SLAM, localization, mapping, planning, and navigation through stable pure-data interfaces, without modifying the 43/43-gtest-verified algorithm code.

**Architecture:** A new `amr_api` package (depends only on `amr_core`) defines six abstract interfaces (`ISlam`, `ILocalizer`, `IMapper`, `IGlobalPlanner`, `ILocalPlanner`, `IBehavior`), pure-data I/O structs, a read-only `CostmapView`, `Logger`/`Clock` seams, and `version.hpp`. Each module package gains a thin adapter that *delegates* to its existing class and a parity gtest proving identical output. A new in-tree `amr_reference_core` package wires the adapters end-to-end as the integration guide's worked example. The ROS nodes are frozen (version-stamped + documented), not rewritten.

**Tech Stack:** ROS 2 Humble, C++17, `ament_cmake`, colcon, gtest, RoboStack `ros2_humble` micromamba env.

---

## Conventions for every task

- **Edit + build location:** the git repo `/home/cona/kangj/amr_stack` (its `build/`, `install/`, `log/` are git-ignored). Ignore the separate `~/kangj/amr_ros2_dev/humble_ws` sandbox.
- **SPDX:** first line of every new `.hpp`/`.cpp` is `// SPDX-License-Identifier: Apache-2.0`; YAML/py use `# SPDX-License-Identifier: Apache-2.0`.
- **Build/test command template** (run from anywhere — it `cd`s itself):
  ```bash
  micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
    colcon build --packages-up-to <pkg> && \
    colcon test --packages-select <pkg> && colcon test-result --all'
  ```
  `unset PYTHONPATH` is mandatory (a stray host `PYTHONPATH`/ROS1 `noetic` source breaks the build).
- **No constant invention:** all numerics come from `amr_core::*Config` defaults (verified present in `src/amr_core/include/amr_core/config.hpp`).
- **Commit messages:** Conventional Commits, English, no `Co-Authored-By`/`Generated with` footers.

---

## Task 1: Create the `amr_api` package

**Files:**
- Create: `src/amr_api/package.xml`
- Create: `src/amr_api/CMakeLists.txt`
- Create: `src/amr_api/include/amr_api/version.hpp`
- Create: `src/amr_api/include/amr_api/types.hpp`
- Create: `src/amr_api/include/amr_api/costmap_view.hpp`
- Create: `src/amr_api/include/amr_api/diagnostics.hpp`
- Create: `src/amr_api/include/amr_api/slam.hpp`
- Create: `src/amr_api/include/amr_api/localizer.hpp`
- Create: `src/amr_api/include/amr_api/mapper.hpp`
- Create: `src/amr_api/include/amr_api/global_planner.hpp`
- Create: `src/amr_api/include/amr_api/local_planner.hpp`
- Create: `src/amr_api/include/amr_api/behavior.hpp`
- Create: `src/amr_api/src/behavior.cpp`
- Test: `src/amr_api/test/test_amr_api.cpp`

- [ ] **Step 1: Write the smoke test** (`src/amr_api/test/test_amr_api.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Smoke tests for the amr_api interface layer: version constants, BehaviorState
// labels, the no-op logger, and that the pure-data structs default sanely.
#include <gtest/gtest.h>

#include "amr_api/behavior.hpp"
#include "amr_api/diagnostics.hpp"
#include "amr_api/localizer.hpp"
#include "amr_api/version.hpp"

TEST(AmrApi, VersionStrings) {
  EXPECT_STREQ(amr_api::VERSION, "0.1.0");
  EXPECT_STREQ(amr_api::CONTRACT_VERSION, "1.0");
  EXPECT_EQ(amr_api::VERSION_MAJOR, 0);
}

TEST(AmrApi, BehaviorStateLabels) {
  EXPECT_STREQ(amr_api::to_string(amr_api::BehaviorState::FOLLOWING), "FOLLOWING");
  EXPECT_STREQ(amr_api::to_string(amr_api::BehaviorState::SUCCEEDED), "SUCCEEDED");
}

TEST(AmrApi, NullLoggerIsSilent) {
  amr_api::NullLogger log;
  log.log(amr_api::Logger::Level::Info, "ignored");  // must not throw
  SUCCEED();
}

TEST(AmrApi, LocalizerOutputDefaults) {
  amr_api::LocalizerOutput out;
  EXPECT_TRUE(out.cloud.poses.empty());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: Write `version.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// amr_api semantic version + the CONTRACT.md revision it was validated against.
#pragma once

namespace amr_api {

inline constexpr int VERSION_MAJOR = 0;
inline constexpr int VERSION_MINOR = 1;
inline constexpr int VERSION_PATCH = 0;
inline constexpr const char* VERSION = "0.1.0";
inline constexpr const char* CONTRACT_VERSION = "1.0";

}  // namespace amr_api
```

- [ ] **Step 3: Write `types.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// Pure-data aggregates shared across the amr_api interfaces. Reuses amr_core
// types verbatim and adds the few aggregates the interfaces exchange.
#pragma once

#include <array>
#include <vector>

#include "amr_core/types.hpp"

namespace amr_api {

using amr_core::LaserScan;
using amr_core::OccupancyGrid;
using amr_core::Pose2D;
using amr_core::Twist2D;

/// World (x, y) waypoints (identical shape to amr_planning::Path).
using Path = std::vector<std::array<double, 2>>;

/// Particle-filter snapshot for visualization / diagnostics.
struct ParticleCloud {
  std::vector<Pose2D> poses;
  std::vector<double> weights;
};

}  // namespace amr_api
```

- [ ] **Step 4: Write `costmap_view.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// Read-only costmap abstraction. Lets amr_api expose a costmap to planners and
// behaviors without depending on amr_planning (where the concrete Costmap and
// its inflation live). The reference backend implements this in amr_planning
// (CostmapAdapter + make_costmap).
#pragma once

namespace amr_api {

class CostmapView {
 public:
  static constexpr float LETHAL = 1.0f;

  virtual ~CostmapView() = default;

  virtual int rows() const = 0;
  virtual int cols() const = 0;
  virtual double resolution() const = 0;

  virtual void world_to_grid(double x, double y, int& row, int& col) const = 0;
  virtual void grid_to_world(int row, int col, double& x, double& y) const = 0;
  virtual bool in_bounds(int row, int col) const = 0;

  virtual float cost_at(int row, int col) const = 0;        // in [0, 1]
  virtual double cost_at_world(double x, double y) const = 0;
  virtual bool is_lethal(int row, int col) const = 0;
};

}  // namespace amr_api
```

- [ ] **Step 5: Write `diagnostics.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// Diagnostics seams owned by core: a logging sink and a clock. amr_api defines
// the abstract seams plus a no-op logger so modules/adapters run standalone
// (e.g. in gtests) without a core present.
#pragma once

#include <string>

namespace amr_api {

class Logger {
 public:
  enum class Level { Debug, Info, Warn, Error };
  virtual ~Logger() = default;
  virtual void log(Level level, const std::string& msg) = 0;
};

/// Default no-op logger (used when no core is wired in).
class NullLogger : public Logger {
 public:
  void log(Level /*level*/, const std::string& /*msg*/) override {}
};

class Clock {
 public:
  virtual ~Clock() = default;
  virtual double now() const = 0;  // seconds
};

}  // namespace amr_api
```

- [ ] **Step 6: Write `slam.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// ISlam: standardized SLAM front-end interface. Fuses an odometry increment and
// an optional laser scan into a map-frame pose, and owns the live map.
#pragma once

#include <optional>

#include "amr_api/types.hpp"

namespace amr_api {

struct SlamInput {
  Pose2D odom_delta;              // robot-frame increment since last tick
  std::optional<LaserScan> scan;  // present only on scan ticks
};

class ISlam {
 public:
  virtual ~ISlam() = default;

  /// Fuse one tick; return the updated map-frame pose.
  virtual Pose2D update(const SlamInput& in) = 0;

  /// Current best map-frame pose.
  virtual Pose2D pose() const = 0;

  /// Snapshot the live occupancy map.
  virtual OccupancyGrid map() const = 0;
};

}  // namespace amr_api
```

- [ ] **Step 7: Write `localizer.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// ILocalizer: standardized localization interface over a known map. Bridges a
// predict/correct/estimate filter to a single tick.
#pragma once

#include <optional>

#include "amr_api/types.hpp"

namespace amr_api {

struct LocalizerInput {
  Pose2D odom_delta;
  std::optional<LaserScan> scan;  // correction runs only when present
};

struct LocalizerOutput {
  Pose2D pose;
  ParticleCloud cloud;
};

class ILocalizer {
 public:
  virtual ~ILocalizer() = default;

  /// Predict with odom_delta, correct if a scan is present, return the estimate.
  virtual LocalizerOutput update(const LocalizerInput& in) = 0;

  /// Current weighted-mean pose estimate.
  virtual Pose2D estimate() const = 0;

  /// Re-seed the belief about `pose`.
  virtual void set_pose(const Pose2D& pose) = 0;
};

}  // namespace amr_api
```

- [ ] **Step 8: Write `mapper.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// IMapper: standardized occupancy mapping interface. Integrates scans taken at a
// known pose and exposes the resulting grid.
#pragma once

#include "amr_api/types.hpp"

namespace amr_api {

struct MapperInput {
  Pose2D pose;
  LaserScan scan;
};

class IMapper {
 public:
  virtual ~IMapper() = default;

  /// Integrate one scan taken at `in.pose`.
  virtual void integrate(const MapperInput& in) = 0;

  /// Snapshot the occupancy map.
  virtual OccupancyGrid map() const = 0;
};

}  // namespace amr_api
```

- [ ] **Step 9: Write `global_planner.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// IGlobalPlanner: standardized global path planner interface.
#pragma once

#include <array>
#include <optional>

#include "amr_api/costmap_view.hpp"
#include "amr_api/types.hpp"

namespace amr_api {

struct GlobalPlanRequest {
  std::array<double, 2> start_xy;
  std::array<double, 2> goal_xy;
  const CostmapView* costmap;  // non-owning; must outlive the call
};

class IGlobalPlanner {
 public:
  virtual ~IGlobalPlanner() = default;

  /// Plan start->goal over the costmap; nullopt if unreachable.
  virtual std::optional<Path> plan(const GlobalPlanRequest& req) = 0;
};

}  // namespace amr_api
```

- [ ] **Step 10: Write `local_planner.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// ILocalPlanner: standardized local planner / trajectory controller interface.
#pragma once

#include "amr_api/costmap_view.hpp"
#include "amr_api/types.hpp"

namespace amr_api {

struct LocalPlanRequest {
  Pose2D pose;
  Twist2D vel;
  const Path* path;            // non-owning; the global plan to track
  const CostmapView* costmap;  // non-owning
};

struct LocalPlanResult {
  Twist2D cmd;
  bool blocked;  // true if every sampled rollout collides
};

class ILocalPlanner {
 public:
  virtual ~ILocalPlanner() = default;

  /// Compute one velocity command tracking `req.path`.
  virtual LocalPlanResult compute(const LocalPlanRequest& req) = 0;
};

}  // namespace amr_api
```

- [ ] **Step 11: Write `behavior.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// IBehavior: standardized goal-driven behavior (navigation FSM) interface.
#pragma once

#include <optional>

#include "amr_api/costmap_view.hpp"
#include "amr_api/types.hpp"

namespace amr_api {

enum class BehaviorState {
  IDLE,
  PLANNING,
  FOLLOWING,
  RECOVERY,
  SUCCEEDED,
  FAILED
};

/// Stable label, matching the amr_navigation NavState strings and the
/// amr_interfaces feedback/result `state` field values.
const char* to_string(BehaviorState s);

struct BehaviorInput {
  Pose2D pose;
  Twist2D vel;
  const CostmapView* costmap;  // non-owning
  double now;                  // seconds, from core's Clock
};

struct BehaviorOutput {
  Twist2D cmd;
  BehaviorState state;
  std::optional<Path> plan;  // current global plan, for viz
};

class IBehavior {
 public:
  virtual ~IBehavior() = default;

  virtual void set_goal(const Pose2D& goal) = 0;
  virtual void cancel() = 0;
  virtual BehaviorOutput update(const BehaviorInput& in) = 0;
  virtual BehaviorState state() const = 0;
};

}  // namespace amr_api
```

- [ ] **Step 12: Write `src/behavior.cpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
#include "amr_api/behavior.hpp"

namespace amr_api {

const char* to_string(BehaviorState s) {
  switch (s) {
    case BehaviorState::IDLE: return "IDLE";
    case BehaviorState::PLANNING: return "PLANNING";
    case BehaviorState::FOLLOWING: return "FOLLOWING";
    case BehaviorState::RECOVERY: return "RECOVERY";
    case BehaviorState::SUCCEEDED: return "SUCCEEDED";
    case BehaviorState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace amr_api
```

- [ ] **Step 13: Write `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_api</name>
  <version>0.1.0</version>
  <description>
    Node-free interface layer that standardizes AMR-stack module I/O so an
    external orchestrator ("core") can drive SLAM, localization, mapping,
    planning and navigation through stable pure-data interfaces.
  </description>
  <maintainer email="kangjmo91@gmail.com">Kang Jung Mo</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>amr_core</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 14: Write `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_api)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)

# Interface layer: mostly headers; one small .cpp for to_string(BehaviorState).
add_library(amr_api
  src/behavior.cpp
)
target_include_directories(amr_api PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_api PUBLIC cxx_std_17)
ament_target_dependencies(amr_api amr_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_api
  EXPORT export_amr_api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_amr_api HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_amr_api test/test_amr_api.cpp)
  target_link_libraries(test_amr_api amr_api)
  ament_target_dependencies(test_amr_api amr_core)
endif()

ament_package()
```

- [ ] **Step 15: Build + test**

Run:
```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build --packages-up-to amr_api && \
  colcon test --packages-select amr_api && colcon test-result --all'
```
Expected: `amr_api` builds; `test_amr_api` passes (4 tests, 0 failures).

- [ ] **Step 16: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add src/amr_api && \
  git commit -m "feat(amr_api): add node-free interface layer for core integration"
```

---

## Task 2: `amr_planning` — CostmapView + global/local planner adapters

**Files:**
- Create: `src/amr_planning/include/amr_planning/costmap_adapter.hpp`
- Create: `src/amr_planning/src/costmap_adapter.cpp`
- Create: `src/amr_planning/include/amr_planning/global_planner_adapter.hpp`
- Create: `src/amr_planning/src/global_planner_adapter.cpp`
- Create: `src/amr_planning/include/amr_planning/local_planner_adapter.hpp`
- Create: `src/amr_planning/src/local_planner_adapter.cpp`
- Modify: `src/amr_planning/CMakeLists.txt`
- Modify: `src/amr_planning/package.xml`
- Test: `src/amr_planning/test/test_planning_adapters.cpp`

- [ ] **Step 1: Write the parity test** (`test/test_planning_adapters.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Parity tests: the CostmapView / IGlobalPlanner / ILocalPlanner adapters must
// produce output identical to the raw amr_planning calls on the same inputs.
#include <array>

#include <gtest/gtest.h>

#include "amr_api/global_planner.hpp"
#include "amr_api/local_planner.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/astar.hpp"
#include "amr_planning/costmap.hpp"
#include "amr_planning/costmap_adapter.hpp"
#include "amr_planning/dwa.hpp"
#include "amr_planning/global_planner_adapter.hpp"
#include "amr_planning/local_planner_adapter.hpp"

namespace {
amr_core::OccupancyGrid free_grid() {
  amr_core::OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = 40;
  g.cols = 40;
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);
  return g;
}
}  // namespace

TEST(PlanningAdapters, CostmapViewMatchesRaw) {
  const auto grid = free_grid();
  amr_core::CostmapConfig cfg;
  const double r = 0.15;
  amr_planning::Costmap raw(grid, cfg, r);
  auto view = amr_planning::make_costmap(grid, cfg, r);

  ASSERT_EQ(view->rows(), raw.rows());
  ASSERT_EQ(view->cols(), raw.cols());
  for (int row = 0; row < raw.rows(); ++row) {
    for (int col = 0; col < raw.cols(); ++col) {
      EXPECT_FLOAT_EQ(view->cost_at(row, col), raw.cost_at(row, col));
    }
  }
}

TEST(PlanningAdapters, GlobalPlannerMatchesRaw) {
  const auto grid = free_grid();
  amr_core::CostmapConfig ccfg;
  amr_core::AstarConfig acfg;
  const double r = 0.15;
  amr_planning::Costmap raw(grid, ccfg, r);
  auto view = amr_planning::make_costmap(grid, ccfg, r);

  const std::array<double, 2> start{0.2, 0.2};
  const std::array<double, 2> goal{1.6, 1.6};

  const auto expected = amr_planning::plan_path(raw, start, goal, acfg);
  amr_planning::AstarGlobalPlanner planner(acfg);
  amr_api::GlobalPlanRequest req{start, goal, view.get()};
  const auto got = planner.plan(req);

  ASSERT_EQ(expected.has_value(), got.has_value());
  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(expected->size(), got->size());
  for (std::size_t i = 0; i < got->size(); ++i) {
    EXPECT_DOUBLE_EQ((*expected)[i][0], (*got)[i][0]);
    EXPECT_DOUBLE_EQ((*expected)[i][1], (*got)[i][1]);
  }
}

TEST(PlanningAdapters, LocalPlannerMatchesRaw) {
  const auto grid = free_grid();
  amr_core::CostmapConfig ccfg;
  amr_core::DwaConfig dcfg;
  amr_core::RobotConfig rob;
  const double r = rob.radius;
  amr_planning::Costmap raw(grid, ccfg, r);
  auto view = amr_planning::make_costmap(grid, ccfg, r);

  amr_core::Pose2D pose{0.2, 0.2, 0.0};
  amr_core::Twist2D vel{0.0, 0.0};
  amr_api::Path path{{0.2, 0.2}, {0.6, 0.2}, {1.0, 0.2}};

  amr_planning::DwaPlanner raw_planner(dcfg, rob);
  const auto expected = raw_planner.compute(pose, vel, path, raw);

  amr_planning::DwaLocalPlanner planner(dcfg, rob);
  amr_api::LocalPlanRequest req{pose, vel, &path, view.get()};
  const auto got = planner.compute(req);

  EXPECT_DOUBLE_EQ(expected.cmd.v, got.cmd.v);
  EXPECT_DOUBLE_EQ(expected.cmd.omega, got.cmd.omega);
  EXPECT_EQ(expected.blocked, got.blocked);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: Run the build to verify the test fails to compile** (adapters don't exist yet) — run the Step 9 command; expect a "costmap_adapter.hpp: No such file" compile error.

- [ ] **Step 3: Write `costmap_adapter.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// CostmapAdapter: wraps an amr_planning::Costmap as an amr_api::CostmapView so
// core can hold and inspect a costmap without depending on amr_planning. The
// reference planners recover the concrete Costmap via as_costmap(); the pairing
// of "reference planner <-> reference costmap" is a documented contract.
#pragma once

#include <memory>

#include "amr_api/costmap_view.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/costmap.hpp"

namespace amr_planning {

class CostmapAdapter : public amr_api::CostmapView {
 public:
  CostmapAdapter(const amr_core::OccupancyGrid& grid,
                 const amr_core::CostmapConfig& cfg, double robot_radius)
      : costmap_(grid, cfg, robot_radius) {}

  int rows() const override { return costmap_.rows(); }
  int cols() const override { return costmap_.cols(); }
  double resolution() const override { return costmap_.resolution(); }
  void world_to_grid(double x, double y, int& row, int& col) const override {
    costmap_.world_to_grid(x, y, row, col);
  }
  void grid_to_world(int row, int col, double& x, double& y) const override {
    costmap_.grid_to_world(row, col, x, y);
  }
  bool in_bounds(int row, int col) const override {
    return costmap_.in_bounds(row, col);
  }
  float cost_at(int row, int col) const override {
    return costmap_.cost_at(row, col);
  }
  double cost_at_world(double x, double y) const override {
    return costmap_.cost_at_world(x, y);
  }
  bool is_lethal(int row, int col) const override {
    return costmap_.is_lethal(row, col);
  }

  /// Concrete costmap, for the reference planners.
  const Costmap& raw() const { return costmap_; }

 private:
  Costmap costmap_;
};

/// Build a CostmapView from an occupancy grid (runs chamfer inflation once).
std::unique_ptr<amr_api::CostmapView> make_costmap(
    const amr_core::OccupancyGrid& grid, const amr_core::CostmapConfig& cfg,
    double robot_radius);

/// Recover the concrete Costmap from a CostmapView produced by make_costmap.
/// Throws std::bad_cast if `view` is not a CostmapAdapter (paired contract).
const Costmap& as_costmap(const amr_api::CostmapView& view);

}  // namespace amr_planning
```

- [ ] **Step 4: Write `costmap_adapter.cpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
#include "amr_planning/costmap_adapter.hpp"

namespace amr_planning {

std::unique_ptr<amr_api::CostmapView> make_costmap(
    const amr_core::OccupancyGrid& grid, const amr_core::CostmapConfig& cfg,
    double robot_radius) {
  return std::make_unique<CostmapAdapter>(grid, cfg, robot_radius);
}

const Costmap& as_costmap(const amr_api::CostmapView& view) {
  // The reference planners are only ever paired with the reference costmap;
  // dynamic_cast throws std::bad_cast if a foreign CostmapView slips through.
  return dynamic_cast<const CostmapAdapter&>(view).raw();
}

}  // namespace amr_planning
```

- [ ] **Step 5: Write `global_planner_adapter.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// AstarGlobalPlanner: implements amr_api::IGlobalPlanner by delegating to
// amr_planning::plan_path over the costmap recovered from the CostmapView.
#pragma once

#include <optional>

#include "amr_api/global_planner.hpp"
#include "amr_core/config.hpp"

namespace amr_planning {

class AstarGlobalPlanner : public amr_api::IGlobalPlanner {
 public:
  explicit AstarGlobalPlanner(const amr_core::AstarConfig& cfg) : cfg_(cfg) {}

  std::optional<amr_api::Path> plan(
      const amr_api::GlobalPlanRequest& req) override;

 private:
  amr_core::AstarConfig cfg_;
};

}  // namespace amr_planning
```

- [ ] **Step 6: Write `global_planner_adapter.cpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
#include "amr_planning/global_planner_adapter.hpp"

#include "amr_planning/astar.hpp"
#include "amr_planning/costmap_adapter.hpp"

namespace amr_planning {

std::optional<amr_api::Path> AstarGlobalPlanner::plan(
    const amr_api::GlobalPlanRequest& req) {
  const Costmap& costmap = as_costmap(*req.costmap);
  return plan_path(costmap, req.start_xy, req.goal_xy, cfg_);
}

}  // namespace amr_planning
```

- [ ] **Step 7: Write `local_planner_adapter.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// DwaLocalPlanner: implements amr_api::ILocalPlanner by delegating to
// amr_planning::DwaPlanner over the costmap recovered from the CostmapView.
#pragma once

#include "amr_api/local_planner.hpp"
#include "amr_core/config.hpp"
#include "amr_planning/dwa.hpp"

namespace amr_planning {

class DwaLocalPlanner : public amr_api::ILocalPlanner {
 public:
  DwaLocalPlanner(const amr_core::DwaConfig& cfg,
                  const amr_core::RobotConfig& robot)
      : planner_(cfg, robot) {}

  amr_api::LocalPlanResult compute(
      const amr_api::LocalPlanRequest& req) override;

 private:
  DwaPlanner planner_;
};

}  // namespace amr_planning
```

- [ ] **Step 8: Write `local_planner_adapter.cpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
#include "amr_planning/local_planner_adapter.hpp"

#include "amr_planning/costmap_adapter.hpp"

namespace amr_planning {

amr_api::LocalPlanResult DwaLocalPlanner::compute(
    const amr_api::LocalPlanRequest& req) {
  const Costmap& costmap = as_costmap(*req.costmap);
  const DwaResult res = planner_.compute(req.pose, req.vel, *req.path, costmap);
  return amr_api::LocalPlanResult{res.cmd, res.blocked};
}

}  // namespace amr_planning
```

- [ ] **Step 9: Modify `CMakeLists.txt`**

Add `find_package(amr_api REQUIRED)` after the `find_package(amr_core REQUIRED)` line. Replace the `add_library(amr_planning ...)` block's source list so it reads:
```cmake
add_library(amr_planning
  src/costmap.cpp
  src/astar.cpp
  src/dwa.cpp
  src/costmap_adapter.cpp
  src/global_planner_adapter.cpp
  src/local_planner_adapter.cpp
)
```
Change `ament_target_dependencies(amr_planning amr_core)` → `ament_target_dependencies(amr_planning amr_core amr_api)` and `ament_export_dependencies(amr_core)` → `ament_export_dependencies(amr_core amr_api)`. In the `if(BUILD_TESTING)` block, after the existing `test_planning` block, add:
```cmake
  ament_add_gtest(test_planning_adapters test/test_planning_adapters.cpp)
  target_link_libraries(test_planning_adapters amr_planning)
  ament_target_dependencies(test_planning_adapters amr_core amr_api)
```

- [ ] **Step 10: Modify `package.xml`** — add `<depend>amr_api</depend>` directly below `<depend>amr_core</depend>`.

- [ ] **Step 11: Build + test**

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build --packages-up-to amr_planning && \
  colcon test --packages-select amr_planning && colcon test-result --all'
```
Expected: builds; both `test_planning` (existing) and `test_planning_adapters` (3 tests) pass.

- [ ] **Step 12: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add src/amr_planning && \
  git commit -m "feat(amr_planning): add CostmapView + global/local planner adapters"
```

---

## Task 3: `amr_mapping` — IMapper adapter

**Files:**
- Create: `src/amr_mapping/include/amr_mapping/mapper_adapter.hpp`
- Modify: `src/amr_mapping/CMakeLists.txt`
- Modify: `src/amr_mapping/package.xml`
- Test: `src/amr_mapping/test/test_mapper_adapter.cpp`

- [ ] **Step 1: Write the parity test** (`test/test_mapper_adapter.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Parity test: OccupancyGridMapperAdapter must produce the same grid as the raw
// OccupancyGridMapper for the same scan at the same pose.
#include <utility>

#include <gtest/gtest.h>

#include "amr_api/mapper.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_mapping/mapper_adapter.hpp"
#include "amr_mapping/occupancy_grid_mapper.hpp"

namespace {
amr_core::LaserScan ring_scan() {
  amr_core::LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 60.0;
  s.range_min = 0.12;
  s.range_max = 8.0;
  s.ranges.assign(60, 1.0);
  return s;
}
}  // namespace

TEST(MapperAdapter, MatchesRaw) {
  amr_core::MappingConfig cfg;
  const std::pair<double, double> size{4.0, 4.0};
  const std::pair<double, double> origin{0.0, 0.0};
  const amr_core::Pose2D pose{2.0, 2.0, 0.0};
  const auto scan = ring_scan();

  amr_mapping::OccupancyGridMapper raw(cfg, size, origin);
  raw.update(pose, scan);
  const auto g_raw = raw.to_occupancy_grid();

  amr_mapping::OccupancyGridMapperAdapter adp(cfg, size, origin);
  adp.integrate(amr_api::MapperInput{pose, scan});
  const auto g_adp = adp.map();

  ASSERT_EQ(g_raw.rows, g_adp.rows);
  ASSERT_EQ(g_raw.cols, g_adp.cols);
  ASSERT_EQ(g_raw.data.size(), g_adp.data.size());
  for (std::size_t i = 0; i < g_raw.data.size(); ++i) {
    EXPECT_EQ(g_raw.data[i], g_adp.data[i]);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: Write `mapper_adapter.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// OccupancyGridMapperAdapter: implements amr_api::IMapper by delegating to
// amr_mapping::OccupancyGridMapper.
#pragma once

#include <utility>

#include "amr_api/mapper.hpp"
#include "amr_core/config.hpp"
#include "amr_mapping/occupancy_grid_mapper.hpp"

namespace amr_mapping {

class OccupancyGridMapperAdapter : public amr_api::IMapper {
 public:
  OccupancyGridMapperAdapter(const amr_core::MappingConfig& cfg,
                             std::pair<double, double> size_m,
                             std::pair<double, double> origin_xy = {0.0, 0.0})
      : mapper_(cfg, size_m, origin_xy) {}

  void integrate(const amr_api::MapperInput& in) override {
    mapper_.update(in.pose, in.scan);
  }
  amr_api::OccupancyGrid map() const override {
    return mapper_.to_occupancy_grid();
  }

  const OccupancyGridMapper& raw() const { return mapper_; }

 private:
  OccupancyGridMapper mapper_;
};

}  // namespace amr_mapping
```

- [ ] **Step 3: Modify `CMakeLists.txt`**

Add `find_package(amr_api REQUIRED)` after `find_package(nav_msgs REQUIRED)`. Change `ament_target_dependencies(amr_mapping_lib amr_core)` → `ament_target_dependencies(amr_mapping_lib amr_core amr_api)`. Change `ament_export_dependencies(amr_core rclcpp nav_msgs)` → `ament_export_dependencies(amr_core rclcpp nav_msgs amr_api)`. In the `if(BUILD_TESTING)` block append:
```cmake
  ament_add_gtest(test_mapper_adapter test/test_mapper_adapter.cpp)
  target_link_libraries(test_mapper_adapter amr_mapping_lib)
  ament_target_dependencies(test_mapper_adapter amr_core amr_api)
```

- [ ] **Step 4: Modify `package.xml`** — add `<depend>amr_api</depend>` below `<depend>amr_core</depend>`.

- [ ] **Step 5: Build + test**

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build --packages-up-to amr_mapping && \
  colcon test --packages-select amr_mapping && colcon test-result --all'
```
Expected: builds; `test_map_io`, `test_mapper`, `test_mapper_adapter` all pass.

- [ ] **Step 6: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add src/amr_mapping && \
  git commit -m "feat(amr_mapping): add IMapper adapter over OccupancyGridMapper"
```

---

## Task 4: `amr_slam` — ISlam adapter

**Files:**
- Create: `src/amr_slam/include/amr_slam/slam_adapter.hpp`
- Modify: `src/amr_slam/CMakeLists.txt`
- Modify: `src/amr_slam/package.xml`
- Test: `src/amr_slam/test/test_slam_adapter.cpp`

- [ ] **Step 1: Write the parity test** (`test/test_slam_adapter.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Parity test: ScanMatchingSlamAdapter == raw ScanMatchingSlam on identical
// input (pose at each tick + the final map).
#include <utility>

#include <gtest/gtest.h>

#include "amr_api/slam.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_slam/scan_matching_slam.hpp"
#include "amr_slam/slam_adapter.hpp"

namespace {
amr_core::LaserScan ring_scan() {
  amr_core::LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 60.0;
  s.range_min = 0.12;
  s.range_max = 8.0;
  s.ranges.assign(60, 3.0);
  return s;
}
}  // namespace

TEST(SlamAdapter, MatchesRaw) {
  amr_core::SlamConfig scfg;
  amr_core::MappingConfig mcfg;
  const std::pair<double, double> size{10.0, 10.0};
  const amr_core::Pose2D init{5.0, 5.0, 0.0};
  const auto scan = ring_scan();

  amr_slam::ScanMatchingSlam raw(scfg, mcfg, size, init);
  amr_slam::ScanMatchingSlamAdapter adp(scfg, mcfg, size, init);

  const amr_core::Pose2D delta{0.05, 0.0, 0.0};
  for (int i = 0; i < 3; ++i) {
    const amr_core::Pose2D pr = raw.process(delta, scan);
    const amr_core::Pose2D pa = adp.update(amr_api::SlamInput{delta, scan});
    EXPECT_DOUBLE_EQ(pr.x, pa.x);
    EXPECT_DOUBLE_EQ(pr.y, pa.y);
    EXPECT_DOUBLE_EQ(pr.theta, pa.theta);
  }
  const auto gr = raw.get_map();
  const auto ga = adp.map();
  ASSERT_EQ(gr.data.size(), ga.data.size());
  for (std::size_t i = 0; i < gr.data.size(); ++i) {
    EXPECT_EQ(gr.data[i], ga.data[i]);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: Write `slam_adapter.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// ScanMatchingSlamAdapter: implements amr_api::ISlam by delegating to
// amr_slam::ScanMatchingSlam.
#pragma once

#include <utility>

#include "amr_api/slam.hpp"
#include "amr_core/config.hpp"
#include "amr_slam/scan_matching_slam.hpp"

namespace amr_slam {

class ScanMatchingSlamAdapter : public amr_api::ISlam {
 public:
  ScanMatchingSlamAdapter(const amr_core::SlamConfig& cfg,
                          const amr_core::MappingConfig& mapping_cfg,
                          std::pair<double, double> size_m,
                          const amr_core::Pose2D& initial_pose)
      : slam_(cfg, mapping_cfg, size_m, initial_pose) {}

  amr_api::Pose2D update(const amr_api::SlamInput& in) override {
    return slam_.process(in.odom_delta, in.scan);
  }
  amr_api::Pose2D pose() const override { return slam_.pose(); }
  amr_api::OccupancyGrid map() const override { return slam_.get_map(); }

  const ScanMatchingSlam& raw() const { return slam_; }

 private:
  ScanMatchingSlam slam_;
};

}  // namespace amr_slam
```

- [ ] **Step 3: Modify `CMakeLists.txt`**

Add `find_package(amr_api REQUIRED)` after `find_package(amr_interfaces REQUIRED)`. Change `ament_target_dependencies(amr_slam_lib amr_core amr_mapping)` → `ament_target_dependencies(amr_slam_lib amr_core amr_mapping amr_api)`. Change `ament_export_dependencies(amr_core amr_mapping)` → `ament_export_dependencies(amr_core amr_mapping amr_api)`. In `if(BUILD_TESTING)` append:
```cmake
  ament_add_gtest(test_slam_adapter test/test_slam_adapter.cpp)
  target_link_libraries(test_slam_adapter amr_slam_lib)
  ament_target_dependencies(test_slam_adapter amr_core amr_mapping amr_api)
```

- [ ] **Step 4: Modify `package.xml`** — add `<depend>amr_api</depend>` below `<depend>amr_mapping</depend>`.

- [ ] **Step 5: Build + test**

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build --packages-up-to amr_slam && \
  colcon test --packages-select amr_slam && colcon test-result --all'
```
Expected: builds; `test_scan_matching_slam` + `test_slam_adapter` pass.

- [ ] **Step 6: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add src/amr_slam && \
  git commit -m "feat(amr_slam): add ISlam adapter over ScanMatchingSlam"
```

---

## Task 5: `amr_localization` — ILocalizer adapter

**Files:**
- Create: `src/amr_localization/include/amr_localization/mcl_adapter.hpp`
- Modify: `src/amr_localization/CMakeLists.txt`
- Modify: `src/amr_localization/package.xml`
- Test: `src/amr_localization/test/test_mcl_adapter.cpp`

- [ ] **Step 1: Write the parity test** (`test/test_mcl_adapter.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Parity test: MclAdapter (own seeded RNG) == raw MonteCarloLocalizer seeded
// identically, over the same predict/correct sequence. Both engines start at the
// same seed and draw in the same order, so estimates are bit-identical.
#include <random>

#include <gtest/gtest.h>

#include "amr_api/localizer.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_localization/mcl.hpp"
#include "amr_localization/mcl_adapter.hpp"

namespace {
amr_core::OccupancyGrid free_grid() {
  amr_core::OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = 80;
  g.cols = 80;
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);
  return g;
}
amr_core::LaserScan ring_scan() {
  amr_core::LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 40.0;
  s.range_min = 0.12;
  s.range_max = 8.0;
  s.ranges.assign(40, 2.0);
  return s;
}
}  // namespace

TEST(MclAdapter, MatchesRaw) {
  const auto grid = free_grid();
  amr_core::LocalizationConfig cfg;
  const unsigned int seed = 7;
  const amr_core::Pose2D init{2.0, 2.0, 0.0};

  std::mt19937 rng(seed);
  amr_localization::MonteCarloLocalizer raw(grid, cfg, rng, init);
  amr_localization::MclAdapter adp(grid, cfg, seed, init);

  const amr_core::Pose2D delta{0.05, 0.0, 0.01};
  const auto scan = ring_scan();
  for (int i = 0; i < 3; ++i) {
    raw.predict(delta);
    raw.correct(scan);
    const amr_core::Pose2D er = raw.estimate();

    const auto out = adp.update(amr_api::LocalizerInput{delta, scan});

    EXPECT_DOUBLE_EQ(er.x, out.pose.x);
    EXPECT_DOUBLE_EQ(er.y, out.pose.y);
    EXPECT_DOUBLE_EQ(er.theta, out.pose.theta);
  }
  EXPECT_EQ(raw.particles().size(), adp.raw().particles().size());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: Write `mcl_adapter.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// MclAdapter: implements amr_api::ILocalizer by delegating to
// amr_localization::MonteCarloLocalizer. Owns the seeded RNG (the interface is
// pure-data; the adapter holds the determinism seam) and bridges the
// predict/correct/estimate filter to a single update() tick.
//
// Non-copyable / non-movable: MonteCarloLocalizer stores a reference to this
// object's rng_ member, so the adapter must keep a stable address. Hold it via
// std::unique_ptr at call sites.
#pragma once

#include <optional>
#include <random>

#include "amr_api/localizer.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_localization/mcl.hpp"

namespace amr_localization {

class MclAdapter : public amr_api::ILocalizer {
 public:
  MclAdapter(const amr_core::OccupancyGrid& grid,
             const amr_core::LocalizationConfig& cfg, unsigned int seed,
             std::optional<amr_core::Pose2D> initial_pose = std::nullopt)
      : rng_(seed), mcl_(grid, cfg, rng_, initial_pose) {}

  MclAdapter(const MclAdapter&) = delete;
  MclAdapter& operator=(const MclAdapter&) = delete;
  MclAdapter(MclAdapter&&) = delete;
  MclAdapter& operator=(MclAdapter&&) = delete;

  amr_api::LocalizerOutput update(const amr_api::LocalizerInput& in) override {
    mcl_.predict(in.odom_delta);
    if (in.scan.has_value()) {
      mcl_.correct(*in.scan);
    }
    amr_api::LocalizerOutput out;
    out.pose = mcl_.estimate();
    out.cloud.poses = mcl_.particles();
    out.cloud.weights = mcl_.weights();
    return out;
  }
  amr_api::Pose2D estimate() const override { return mcl_.estimate(); }
  void set_pose(const amr_api::Pose2D& pose) override { mcl_.set_pose(pose); }

  const MonteCarloLocalizer& raw() const { return mcl_; }

 private:
  std::mt19937 rng_;            // declared before mcl_ so it outlives the ref
  MonteCarloLocalizer mcl_;
};

}  // namespace amr_localization
```

- [ ] **Step 3: Modify `CMakeLists.txt`**

Add `find_package(amr_api REQUIRED)` after `find_package(amr_core REQUIRED)`. Change `ament_target_dependencies(amr_localization_lib amr_core)` → `ament_target_dependencies(amr_localization_lib amr_core amr_api)`. Change `ament_export_dependencies(amr_core)` → `ament_export_dependencies(amr_core amr_api)`. In `if(BUILD_TESTING)` append:
```cmake
  ament_add_gtest(test_mcl_adapter test/test_mcl_adapter.cpp)
  target_link_libraries(test_mcl_adapter amr_localization_lib)
  ament_target_dependencies(test_mcl_adapter amr_core amr_api)
```

- [ ] **Step 4: Modify `package.xml`** — add `<depend>amr_api</depend>` below `<depend>amr_core</depend>`.

- [ ] **Step 5: Build + test**

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build --packages-up-to amr_localization && \
  colcon test --packages-select amr_localization && colcon test-result --all'
```
Expected: builds; `test_localization` + `test_mcl_adapter` pass.

- [ ] **Step 6: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add src/amr_localization && \
  git commit -m "feat(amr_localization): add ILocalizer adapter over MCL"
```

---

## Task 6: `amr_navigation` — IBehavior adapter

**Files:**
- Create: `src/amr_navigation/include/amr_navigation/navigator_behavior.hpp`
- Create: `src/amr_navigation/src/navigator_behavior.cpp`
- Modify: `src/amr_navigation/CMakeLists.txt`
- Modify: `src/amr_navigation/package.xml`
- Test: `src/amr_navigation/test/test_navigator_behavior.cpp`

- [ ] **Step 1: Write the parity test** (`test/test_navigator_behavior.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Parity test: NavigatorBehavior (IBehavior) == raw Navigator (make_default) on
// identical inputs; plus BehaviorState labels match NavState labels 1:1.
#include <gtest/gtest.h>

#include "amr_api/behavior.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_navigation/navigator.hpp"
#include "amr_navigation/navigator_behavior.hpp"
#include "amr_planning/costmap.hpp"
#include "amr_planning/costmap_adapter.hpp"

namespace {
amr_core::OccupancyGrid free_grid() {
  amr_core::OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = 80;
  g.cols = 80;
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);
  return g;
}
}  // namespace

TEST(NavigatorBehavior, MatchesRaw) {
  amr_core::NavConfig ncfg;
  amr_core::AstarConfig acfg;
  amr_core::DwaConfig dcfg;
  amr_core::RobotConfig rob;
  amr_core::CostmapConfig ccfg;
  const auto grid = free_grid();
  amr_planning::Costmap raw_cm(grid, ccfg, rob.radius);
  auto view = amr_planning::make_costmap(grid, ccfg, rob.radius);

  amr_navigation::Navigator raw =
      amr_navigation::Navigator::make_default(ncfg, acfg, dcfg, rob);
  amr_navigation::NavigatorBehavior adp(ncfg, acfg, dcfg, rob);

  const amr_core::Pose2D goal{2.0, 2.0, 0.0};
  raw.set_goal(goal);
  adp.set_goal(goal);

  amr_core::Pose2D pose{0.5, 0.5, 0.0};
  amr_core::Twist2D vel{0.0, 0.0};
  for (int i = 0; i < 5; ++i) {
    const double now = 0.1 * i;
    const amr_core::Twist2D cr = raw.update(pose, vel, raw_cm, now);
    const auto out =
        adp.update(amr_api::BehaviorInput{pose, vel, view.get(), now});
    EXPECT_DOUBLE_EQ(cr.v, out.cmd.v);
    EXPECT_DOUBLE_EQ(cr.omega, out.cmd.omega);
    EXPECT_STREQ(amr_navigation::to_string(raw.state()),
                 amr_api::to_string(out.state));
    vel = cr;
  }
}

TEST(NavigatorBehavior, StateLabelsMatchNavState) {
  using amr_api::BehaviorState;
  using amr_navigation::NavState;
  EXPECT_STREQ(amr_navigation::to_string(NavState::IDLE),
               amr_api::to_string(BehaviorState::IDLE));
  EXPECT_STREQ(amr_navigation::to_string(NavState::PLANNING),
               amr_api::to_string(BehaviorState::PLANNING));
  EXPECT_STREQ(amr_navigation::to_string(NavState::FOLLOWING),
               amr_api::to_string(BehaviorState::FOLLOWING));
  EXPECT_STREQ(amr_navigation::to_string(NavState::RECOVERY),
               amr_api::to_string(BehaviorState::RECOVERY));
  EXPECT_STREQ(amr_navigation::to_string(NavState::SUCCEEDED),
               amr_api::to_string(BehaviorState::SUCCEEDED));
  EXPECT_STREQ(amr_navigation::to_string(NavState::FAILED),
               amr_api::to_string(BehaviorState::FAILED));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: Write `navigator_behavior.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// NavigatorBehavior: implements amr_api::IBehavior by wrapping the default
// Navigator (built via make_default, wiring real A* + DWA). The behaviour FSM is
// the "Planning + Navigation" swappable unit; core may instead supply its own
// IBehavior implementation.
#pragma once

#include "amr_api/behavior.hpp"
#include "amr_core/config.hpp"
#include "amr_navigation/navigator.hpp"

namespace amr_navigation {

class NavigatorBehavior : public amr_api::IBehavior {
 public:
  NavigatorBehavior(const amr_core::NavConfig& nav_cfg,
                    const amr_core::AstarConfig& astar_cfg,
                    const amr_core::DwaConfig& dwa_cfg,
                    const amr_core::RobotConfig& robot_cfg)
      : nav_(Navigator::make_default(nav_cfg, astar_cfg, dwa_cfg, robot_cfg)) {}

  void set_goal(const amr_core::Pose2D& goal) override { nav_.set_goal(goal); }
  void cancel() override { nav_.cancel(); }
  amr_api::BehaviorOutput update(const amr_api::BehaviorInput& in) override;
  amr_api::BehaviorState state() const override { return to_api(nav_.state()); }

 private:
  static amr_api::BehaviorState to_api(NavState s);
  Navigator nav_;
};

}  // namespace amr_navigation
```

- [ ] **Step 3: Write `navigator_behavior.cpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
#include "amr_navigation/navigator_behavior.hpp"

#include "amr_planning/costmap_adapter.hpp"

namespace amr_navigation {

amr_api::BehaviorState NavigatorBehavior::to_api(NavState s) {
  switch (s) {
    case NavState::IDLE: return amr_api::BehaviorState::IDLE;
    case NavState::PLANNING: return amr_api::BehaviorState::PLANNING;
    case NavState::FOLLOWING: return amr_api::BehaviorState::FOLLOWING;
    case NavState::RECOVERY: return amr_api::BehaviorState::RECOVERY;
    case NavState::SUCCEEDED: return amr_api::BehaviorState::SUCCEEDED;
    case NavState::FAILED: return amr_api::BehaviorState::FAILED;
  }
  return amr_api::BehaviorState::IDLE;
}

amr_api::BehaviorOutput NavigatorBehavior::update(
    const amr_api::BehaviorInput& in) {
  const amr_planning::Costmap& costmap = amr_planning::as_costmap(*in.costmap);
  const amr_core::Twist2D cmd = nav_.update(in.pose, in.vel, costmap, in.now);

  amr_api::BehaviorOutput out;
  out.cmd = cmd;
  out.state = to_api(nav_.state());
  if (nav_.path().has_value()) {
    out.plan = *nav_.path();
  }
  return out;
}

}  // namespace amr_navigation
```

- [ ] **Step 4: Modify `CMakeLists.txt`**

Add `find_package(amr_api REQUIRED)` after `find_package(amr_planning REQUIRED)`. Replace the `add_library(amr_navigation ...)` source list so it reads:
```cmake
add_library(amr_navigation
  src/navigator.cpp
  src/navigator_behavior.cpp
)
```
Change `ament_target_dependencies(amr_navigation amr_core amr_planning)` → `ament_target_dependencies(amr_navigation amr_core amr_planning amr_api)`. Change `ament_export_dependencies(amr_core amr_planning)` → `ament_export_dependencies(amr_core amr_planning amr_api)`. In `if(BUILD_TESTING)` append:
```cmake
  ament_add_gtest(test_navigator_behavior test/test_navigator_behavior.cpp)
  target_link_libraries(test_navigator_behavior amr_navigation)
  ament_target_dependencies(test_navigator_behavior amr_core amr_planning amr_api)
```

- [ ] **Step 5: Modify `package.xml`** — add `<depend>amr_api</depend>` below `<depend>amr_planning</depend>`.

- [ ] **Step 6: Build + test**

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build --packages-up-to amr_navigation && \
  colcon test --packages-select amr_navigation && colcon test-result --all'
```
Expected: builds; `test_navigator` + `test_navigator_behavior` pass.

- [ ] **Step 7: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add src/amr_navigation && \
  git commit -m "feat(amr_navigation): add IBehavior adapter over Navigator FSM"
```

---

## Task 7: `amr_reference_core` — end-to-end worked example

**Files:**
- Create: `src/amr_reference_core/package.xml`
- Create: `src/amr_reference_core/CMakeLists.txt`
- Create: `src/amr_reference_core/include/amr_reference_core/core_bus.hpp`
- Test: `src/amr_reference_core/test/test_reference_core.cpp`

- [ ] **Step 1: Write `core_bus.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// CoreBus: the minimal shared-memory blackboard a host orchestrator owns. It
// holds the latest map / pose / velocity / plan / particle cloud, and provides a
// concrete Logger (stderr) and Clock (monotonic sim seconds) implementing the
// amr_api diagnostics seams. This is the reference template for the private core.
#pragma once

#include <cstdio>
#include <string>

#include "amr_api/diagnostics.hpp"
#include "amr_api/types.hpp"

namespace amr_reference_core {

class StderrLogger : public amr_api::Logger {
 public:
  void log(Level level, const std::string& msg) override {
    const char* tag = "INFO";
    switch (level) {
      case Level::Debug: tag = "DEBUG"; break;
      case Level::Info: tag = "INFO"; break;
      case Level::Warn: tag = "WARN"; break;
      case Level::Error: tag = "ERROR"; break;
    }
    std::fprintf(stderr, "[core][%s] %s\n", tag, msg.c_str());
  }
};

/// Monotonic sim clock advanced by the orchestrator (no wall-clock dependency,
/// so runs are deterministic and replayable).
class SimClock : public amr_api::Clock {
 public:
  double now() const override { return t_; }
  void advance(double dt) { t_ += dt; }
  void set(double t) { t_ = t; }

 private:
  double t_{0.0};
};

/// The shared blackboard: every module reads/writes plain data here.
struct CoreBus {
  amr_api::OccupancyGrid map;
  amr_api::Pose2D pose;
  amr_api::Twist2D vel;
  amr_api::Path plan;
  amr_api::ParticleCloud particles;
  StderrLogger logger;
  SimClock clock;
};

}  // namespace amr_reference_core
```

- [ ] **Step 2: Write the worked-example test** (`test/test_reference_core.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Reference core: the worked example for CORE_INTEGRATION. A minimal host
// orchestrator (CoreBus) drives the standardized module interfaces end to end.
// This file doubles as the integration guide's canonical example.
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "amr_api/behavior.hpp"
#include "amr_api/mapper.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_mapping/mapper_adapter.hpp"
#include "amr_navigation/navigator_behavior.hpp"
#include "amr_planning/costmap_adapter.hpp"
#include "amr_reference_core/core_bus.hpp"

namespace {
// A 4x4 m open arena at 0.05 m resolution, all free.
amr_api::OccupancyGrid open_arena() {
  amr_api::OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = 80;
  g.cols = 80;
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);
  return g;
}
}  // namespace

// Core owns the bus + clock + logger, builds a CostmapView via the planning
// backend, and drives the navigation behavior to a goal, advancing the pose by
// integrating the commanded twist (a stand-in for the simulator / real robot).
TEST(ReferenceCore, NavMissionReachesGoal) {
  amr_reference_core::CoreBus bus;
  bus.map = open_arena();
  bus.pose = amr_core::Pose2D{0.5, 0.5, 0.0};
  bus.vel = amr_core::Twist2D{0.0, 0.0};

  amr_core::RobotConfig robot;
  amr_core::CostmapConfig costmap_cfg;
  auto costmap = amr_planning::make_costmap(bus.map, costmap_cfg, robot.radius);

  // The "Planning + Navigation" module, behind amr_api::IBehavior.
  std::unique_ptr<amr_api::IBehavior> behavior =
      std::make_unique<amr_navigation::NavigatorBehavior>(
          amr_core::NavConfig{}, amr_core::AstarConfig{}, amr_core::DwaConfig{},
          robot);

  const amr_core::Pose2D goal{3.0, 3.0, 0.0};
  behavior->set_goal(goal);
  bus.logger.log(amr_api::Logger::Level::Info, "goal set (3.0, 3.0)");

  const double dt = 0.1;
  amr_api::BehaviorState state = amr_api::BehaviorState::IDLE;
  int ticks = 0;
  for (; ticks < 600; ++ticks) {
    amr_api::BehaviorInput in;
    in.pose = bus.pose;
    in.vel = bus.vel;
    in.costmap = costmap.get();
    in.now = bus.clock.now();

    const amr_api::BehaviorOutput out = behavior->update(in);
    state = out.state;
    bus.vel = out.cmd;
    if (out.plan.has_value()) {
      bus.plan = *out.plan;
    }

    // Advance the pose by integrating the command (unicycle); stands in for the
    // simulator / real robot closing the loop.
    bus.pose.x += out.cmd.v * std::cos(bus.pose.theta) * dt;
    bus.pose.y += out.cmd.v * std::sin(bus.pose.theta) * dt;
    bus.pose.theta += out.cmd.omega * dt;
    bus.clock.advance(dt);

    if (state == amr_api::BehaviorState::SUCCEEDED ||
        state == amr_api::BehaviorState::FAILED) {
      break;
    }
  }

  EXPECT_EQ(state, amr_api::BehaviorState::SUCCEEDED);
  const double err = std::hypot(bus.pose.x - goal.x, bus.pose.y - goal.y);
  EXPECT_LT(err, amr_core::NavConfig{}.goal_tol_xy + 0.05);
  EXPECT_GT(ticks, 0);
}

// Demonstrates the IMapper interface: integrate a scan and read back the grid.
TEST(ReferenceCore, MapperIntegratesScan) {
  amr_core::MappingConfig cfg;
  amr_mapping::OccupancyGridMapperAdapter mapper(cfg, {4.0, 4.0}, {0.0, 0.0});

  amr_core::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / 60.0;
  scan.range_min = 0.12;
  scan.range_max = 8.0;
  scan.ranges.assign(60, 1.0);

  amr_api::IMapper& iface = mapper;
  iface.integrate(amr_api::MapperInput{amr_core::Pose2D{2.0, 2.0, 0.0}, scan});
  const amr_api::OccupancyGrid g = iface.map();

  int occupied = 0;
  for (const auto v : g.data) {
    if (v == 100) ++occupied;
  }
  EXPECT_GT(occupied, 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 3: Write `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_reference_core</name>
  <version>0.1.0</version>
  <description>
    Reference orchestrator ("core") demonstrating how to drive the AMR stack
    modules through the amr_api interfaces. Header-only CoreBus blackboard plus a
    worked-example gtest that wires the adapters end to end.
  </description>
  <maintainer email="kangjmo91@gmail.com">Kang Jung Mo</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>amr_core</depend>
  <depend>amr_api</depend>
  <depend>amr_mapping</depend>
  <depend>amr_localization</depend>
  <depend>amr_slam</depend>
  <depend>amr_planning</depend>
  <depend>amr_navigation</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_reference_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)
find_package(amr_api REQUIRED)
find_package(amr_mapping REQUIRED)
find_package(amr_localization REQUIRED)
find_package(amr_slam REQUIRED)
find_package(amr_planning REQUIRED)
find_package(amr_navigation REQUIRED)

install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(
  amr_core amr_api amr_mapping amr_localization amr_slam amr_planning
  amr_navigation)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reference_core test/test_reference_core.cpp)
  target_include_directories(test_reference_core PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
  ament_target_dependencies(test_reference_core
    amr_core amr_api amr_mapping amr_localization amr_slam amr_planning
    amr_navigation)
endif()

ament_package()
```

- [ ] **Step 5: Build + test**

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build --packages-up-to amr_reference_core && \
  colcon test --packages-select amr_reference_core && colcon test-result --all'
```
Expected: builds; `test_reference_core` passes both tests. **If `NavMissionReachesGoal` does not reach `SUCCEEDED`**, increase the tick budget (600 → 1200) and re-run; if still failing, temporarily log `out.state` each tick to see whether the FSM stalls in `PLANNING`/`RECOVERY`, and confirm a path exists from start to goal in the arena. Do not weaken the `goal_tol_xy + 0.05` assertion.

- [ ] **Step 6: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add src/amr_reference_core && \
  git commit -m "feat(amr_reference_core): add end-to-end reference orchestrator + gtest"
```

---

## Task 8: Full-workspace verification gate

- [ ] **Step 1: Build + test the entire workspace** (CONTRACT.md §8)

```bash
micromamba run -n ros2_humble bash -c 'cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build && colcon test && colcon test-result --all'
```
Expected: every package builds; **all 43 pre-existing gtests still pass** plus the new adapter/reference tests; the `amr_bringup` `launch_testing` demo passes. Zero failures in `colcon test-result --all`.

- [ ] **Step 2: If any pre-existing test regressed**, an adapter changed observable behavior — it must not. Confirm the algorithm sources are untouched (`git diff --stat` should show only new adapter files + CMake/package.xml/test additions), then fix the adapter/CMake only. Re-run the gate.

- [ ] **Step 3: Commit** (only if Step 1 required a fix)

```bash
cd /home/cona/kangj/amr_stack && git add -A && \
  git commit -m "test: green full-workspace gate after amr_api integration"
```

---

## Task 9: Update `CONTRACT.md` + write `amr_api/README.md`

**Files:**
- Modify: `CONTRACT.md`
- Create: `src/amr_api/README.md`

- [ ] **Step 1: Add the two new packages to the §1 dependency graph in `CONTRACT.md`.**

In the code block under "## 1. Workspace layout & dependency graph", insert directly below the `amr_core` line:
```
  amr_api           lib: module interfaces + adapters seam (node-free)  [deps: amr_core]
```
and below the `amr_hri` line:
```
  amr_reference_core demo: wires the adapters as a reference orchestrator   [all module libs, amr_api]
```

- [ ] **Step 2: Append a new §11 to `CONTRACT.md`** (after §10).

```markdown

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

Coordinate/units/occupancy conventions (§2) and QoS (§4) are unchanged; the
adapters carry `amr_core` types, so no conversion semantics are added.
```

- [ ] **Step 3: Add a CONTRACT_VERSION note.** Under the top heading, after the "Target: **ROS 2 Humble**…" paragraph, add a line: `**CONTRACT_VERSION:** 1.0 (see §11 for the amr_api binding).`

- [ ] **Step 4: Write `src/amr_api/README.md`**

````markdown
<!-- SPDX-License-Identifier: Apache-2.0 -->
# amr_api — module interface layer

`amr_api` standardizes the AMR-stack module I/O so an external orchestrator
("core") can drive each module through stable, node-free, pure-data interfaces
and swap any module for its own implementation.

## Interfaces

| Interface | Header | Reference adapter |
|---|---|---|
| `ISlam` | `amr_api/slam.hpp` | `amr_slam::ScanMatchingSlamAdapter` |
| `ILocalizer` | `amr_api/localizer.hpp` | `amr_localization::MclAdapter` |
| `IMapper` | `amr_api/mapper.hpp` | `amr_mapping::OccupancyGridMapperAdapter` |
| `IGlobalPlanner` | `amr_api/global_planner.hpp` | `amr_planning::AstarGlobalPlanner` |
| `ILocalPlanner` | `amr_api/local_planner.hpp` | `amr_planning::DwaLocalPlanner` |
| `IBehavior` | `amr_api/behavior.hpp` | `amr_navigation::NavigatorBehavior` |

Supporting types: `amr_api/types.hpp` (`Path`, `ParticleCloud`, re-exported
`amr_core` types), `amr_api/costmap_view.hpp` (`CostmapView`),
`amr_api/diagnostics.hpp` (`Logger`, `NullLogger`, `Clock`),
`amr_api/version.hpp` (`VERSION`, `CONTRACT_VERSION`).

## The costmap pairing contract

`amr_api` does not depend on `amr_planning`, so it exposes a read-only
`CostmapView`. The reference backend builds one with
`amr_planning::make_costmap(grid, cfg, robot_radius)`; the reference planners
recover the concrete costmap via `amr_planning::as_costmap(view)`. **A planner
and the costmap it consumes are a matched pair:** if `core` swaps the costmap
representation, it must also swap the planners that read it (or supply its own
`CostmapView` that its planners understand).

## Determinism

Interfaces are pure data; the stochastic seam (the RNG seed) lives in the
adapter — e.g. `MclAdapter(grid, cfg, seed, initial_pose)` owns its
`std::mt19937`. This keeps the verified algorithm classes unchanged (CONTRACT.md
§9). `MclAdapter` is non-copyable/non-movable; hold it via `std::unique_ptr`.

## Example

See `amr_reference_core` (`test/test_reference_core.cpp`) for a complete,
runnable example of a host orchestrator driving the modules.
````

- [ ] **Step 5: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add CONTRACT.md src/amr_api/README.md && \
  git commit -m "docs: document amr_api binding in CONTRACT §11 + amr_api README"
```

---

## Task 10: Write `docs/CORE_INTEGRATION.md` (English guide)

**Files:**
- Create: `docs/CORE_INTEGRATION.md`

- [ ] **Step 1: Write the guide.** It MUST contain these sections, in order, with concrete content (no placeholders):

  1. **Overview** — what `amr_api` is; the "core is the brain + bus" model; the one-way dependency position (`amr_core → amr_api → modules → amr_reference_core`).
  2. **Mental model** — algorithm = library; ROS wiring = thin node; the adapter delegates and never modifies verified code. Include the ASCII architecture diagram from the design spec (`docs/superpowers/specs/2026-06-16-amr-core-integration-design.md` §3).
  3. **The six interfaces** — one subsection each (`ISlam`, `ILocalizer`, `IMapper`, `IGlobalPlanner`, `ILocalPlanner`, `IBehavior`): paste the interface declaration (from the headers created in Task 1), name the reference adapter, and give the one-line tick contract (inputs → outputs).
  4. **The CostmapView pairing contract** — copy from `amr_api/README.md`; show `make_costmap` + `as_costmap`.
  5. **Diagnostics seams** — `Logger`/`Clock`; core implements them (show `StderrLogger`/`SimClock` from `core_bus.hpp`).
  6. **Wiring a core: step by step** — walk through `amr_reference_core/test/test_reference_core.cpp::NavMissionReachesGoal` line by line: own the bus, build the `CostmapView`, construct the `IBehavior`, `set_goal`, the tick loop (`update` → integrate → advance clock), termination on `SUCCEEDED`.
  7. **Swapping a module** — to replace, e.g., SLAM: implement `amr_api::ISlam` in your `core` repo, construct it instead of `ScanMatchingSlamAdapter`, keep every other wire identical. Note the "Planning + Navigation" unit is swappable wholesale via `IBehavior`, or finely via `IGlobalPlanner`/`ILocalPlanner`.
  8. **Building against amr_api from an external repo** — `find_package(amr_api REQUIRED)` + `find_package(amr_<module> REQUIRED)`; `ament_target_dependencies(<your_target> amr_api amr_<module> ...)`; `<depend>` entries in `package.xml`; `source install/setup.bash` from this workspace so `find_package` resolves.
  9. **Determinism & faithfulness** — seed lives in the adapter; pull constants from `amr_core::*Config`; the parity gtests guarantee adapters are pass-throughs; CONTRACT.md §2/§9 still hold.
  10. **Verification** — the full gate command; the 43 pre-existing tests stay green.
  11. **Reference** — links to `CONTRACT.md` §11, `src/amr_api/README.md`, and the design spec.

  Write real prose and real code blocks (copy the actual declarations from the headers). Keep it copy-paste runnable against this tree.

- [ ] **Step 2: Save the guide to the Obsidian vault.** Invoke the `obsidian-vault-save` skill to save `docs/CORE_INTEGRATION.md` into the `general_vault` (per the global instruction that every authored standalone document is saved to the vault).

- [ ] **Step 3: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add docs/CORE_INTEGRATION.md && \
  git commit -m "docs: add CORE_INTEGRATION guide (English)"
```

---

## Task 11: Korean translation of the guide + vault save

**Files:**
- Create: `docs/CORE_INTEGRATION.ko.md`

- [ ] **Step 1: Translate the guide into Korean** as `docs/CORE_INTEGRATION.ko.md`, following the global Korean-register rules: 존댓말; 직역체 회피; no English-adjective + Korean mixing; no English-Korean slashes; CS concept terms rendered meaning-first in Korean (e.g. `adapter` → "위임 계층", `interface` → "표준 인터페이스", `shared-memory bus` → "공유 메모리 버스", `dependency injection` → "의존성 주입"). Keep all code blocks, type names, file paths, and CLI commands verbatim (do not translate identifiers). Mirror the English section order 1–11 exactly.

- [ ] **Step 2: Save the Korean guide to the Obsidian vault** via the `obsidian-vault-save` skill.

- [ ] **Step 3: Commit**

```bash
cd /home/cona/kangj/amr_stack && git add docs/CORE_INTEGRATION.ko.md && \
  git commit -m "docs: add CORE_INTEGRATION guide (Korean translation)"
```

---

## Self-review notes (author)

- **Spec coverage:** every spec §4–§9 deliverable maps to a task — interfaces/types/version/costmap_view/diagnostics (Task 1), CostmapView+planners (Task 2), mapper/slam/localizer/behavior adapters (Tasks 3–6), reference core (Task 7), full gate (Task 8), CONTRACT §11 + README (Task 9), EN guide + vault (Task 10), KR guide + vault (Task 11). No spec requirement is untasked.
- **Type consistency:** interface method names (`update`, `integrate`, `compute`, `plan`, `set_goal`, `cancel`, `state`), struct field names (`odom_delta`, `scan`, `pose`, `cmd`, `blocked`, `costmap`, `now`, `start_xy`, `goal_xy`), and adapter accessors (`raw()`, `as_costmap`, `make_costmap`) are used identically across Tasks 1–7 and match the headers in `src/amr_*` verified during planning.
- **Known risk (flagged in Task 7 Step 5):** the reference-core nav mission depends on DWA converging under naive unicycle integration; the task includes a concrete remediation that never weakens the goal-tolerance assertion.
