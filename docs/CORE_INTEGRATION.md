<!-- SPDX-License-Identifier: Apache-2.0 -->
# Core Integration Guide — driving the AMR stack through `amr_api`

This guide shows how to implant a private orchestrator ("core") on top of the
AMR stack. After reading it you should be able to wire your own `core` repo to
drive SLAM, localization, mapping, planning, and navigation through the
standardized `amr_api` interfaces — and replace any single module with your own
implementation.

Companion documents: [`CONTRACT.md`](../CONTRACT.md) (the frozen architecture,
§11 has the data↔topic mapping), [`src/amr_api/README.md`](../src/amr_api/README.md),
and the design spec under `docs/superpowers/specs/`.

---

## 1. Overview

`amr_api` is a **node-free interface layer**. It defines six abstract C++
interfaces with plain-data inputs and outputs. Each existing module ships a thin
**adapter** that implements one interface by delegating to its already-verified
algorithm class — the algorithm code is not touched.

Your `core` is the **brain and the bus**: it owns the shared blackboard (the
latest map, pose, velocity, plan, particle cloud), the clock, and logging, and it
drives the modules tick-by-tick through the interfaces.

The dependency direction is strictly one-way and downward:

```
amr_core  →  amr_api  →  (each module + its adapter)  →  amr_reference_core / your core
```

`amr_api` depends only on `amr_core` (shared types). Every module package depends
on `amr_api` and provides its adapter. Your `core` depends on `amr_api` plus
whichever module packages you reuse.

---

## 2. Mental model

**Algorithm = library; ROS wiring = thin node; integration = pure data.**

- The numeric heart of each module lives in a node-free `*_lib` target with
  gtests (43 pre-existing tests). The adapter is a pass-through; a parity gtest
  proves the adapter's output is identical to the raw class.
- `core` never speaks ROS messages to the modules. It exchanges `amr_core`
  structs (`Pose2D`, `Twist2D`, `LaserScan`, `OccupancyGrid`) and a few `amr_api`
  aggregates (`Path`, `ParticleCloud`). How those map onto ROS topics, if `core`
  runs as a node, is documented once in `CONTRACT.md` §11.

Architecture:

```
                         ┌───────────────────────────────┐
                         │  core  (your private repo)     │
                         │  brain · bus · clock · logging │
                         └───────────────┬───────────────┘
                                         │ programs against
                                         ▼
                         ┌───────────────────────────────┐
                         │  amr_api   (node-free)         │
                         │  interfaces · I/O structs ·    │
                         │  CostmapView · Logger/Clock ·  │
                         │  version.hpp   [dep: amr_core] │
                         └───────────────┬───────────────┘
            implemented by adapters that delegate to ↓ (unchanged) libs
   ┌──────────┬──────────────┬──────────┬───────────────┬──────────────┐
   │ amr_slam │amr_localiza- │amr_map-  │ amr_planning  │amr_navigation│
   │  ISlam   │tion ILocalizer│ping      │ IGlobalPlanner│  IBehavior   │
   │          │              │ IMapper  │ ILocalPlanner │              │
   │          │              │          │ CostmapView   │              │
   └──────────┴──────────────┴──────────┴───────────────┴──────────────┘
```

---

## 3. The six interfaces

All inputs/outputs are plain structs. All methods are tick-style. Pointers inside
request structs are non-owning and must outlive the call.

### 3.1 `ISlam` (`amr_api/slam.hpp`) — reference adapter `amr_slam::ScanMatchingSlamAdapter`

```cpp
struct SlamInput {
  Pose2D odom_delta;              // robot-frame increment since last tick
  std::optional<LaserScan> scan;  // present only on scan ticks
};
class ISlam {
 public:
  virtual ~ISlam() = default;
  virtual Pose2D update(const SlamInput& in) = 0;  // returns map-frame pose
  virtual Pose2D pose() const = 0;
  virtual OccupancyGrid map() const = 0;
};
```

Tick contract: feed the odometry increment (and a scan when one is available);
get back the corrected map-frame pose. Read `map()` at whatever cadence you want
(the reference SLAM emits no "map dirty" event, so `core` polls).

### 3.2 `ILocalizer` (`amr_api/localizer.hpp`) — reference adapter `amr_localization::MclAdapter`

```cpp
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
  virtual LocalizerOutput update(const LocalizerInput& in) = 0;
  virtual Pose2D estimate() const = 0;
  virtual void set_pose(const Pose2D& pose) = 0;
};
```

Tick contract: `update()` predicts with the odometry increment, corrects with the
scan if present, and returns the weighted-mean pose plus the particle cloud (for
visualization). The adapter bridges the underlying predict/correct/estimate
filter to this single call.

### 3.3 `IMapper` (`amr_api/mapper.hpp`) — reference adapter `amr_mapping::OccupancyGridMapperAdapter`

```cpp
struct MapperInput {
  Pose2D pose;
  LaserScan scan;
};
class IMapper {
 public:
  virtual ~IMapper() = default;
  virtual void integrate(const MapperInput& in) = 0;
  virtual OccupancyGrid map() const = 0;
};
```

Tick contract: integrate a scan taken at a known pose into the log-odds map; read
the grid back as an `OccupancyGrid`.

### 3.4 `IGlobalPlanner` (`amr_api/global_planner.hpp`) — reference adapter `amr_planning::AstarGlobalPlanner`

```cpp
struct GlobalPlanRequest {
  std::array<double, 2> start_xy;
  std::array<double, 2> goal_xy;
  const CostmapView* costmap;  // non-owning; must outlive the call
};
class IGlobalPlanner {
 public:
  virtual ~IGlobalPlanner() = default;
  virtual std::optional<Path> plan(const GlobalPlanRequest& req) = 0;
};
```

Tick contract: plan a path from start to goal over the costmap; `nullopt` if
unreachable.

### 3.5 `ILocalPlanner` (`amr_api/local_planner.hpp`) — reference adapter `amr_planning::DwaLocalPlanner`

```cpp
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
  virtual LocalPlanResult compute(const LocalPlanRequest& req) = 0;
};
```

Tick contract: given the current pose/velocity and the global path, return one
velocity command (and a `blocked` flag).

### 3.6 `IBehavior` (`amr_api/behavior.hpp`) — reference adapter `amr_navigation::NavigatorBehavior`

```cpp
enum class BehaviorState { IDLE, PLANNING, FOLLOWING, RECOVERY, SUCCEEDED, FAILED };
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
  std::optional<Path> plan;    // current global plan, for viz
};
class IBehavior {
 public:
  virtual ~IBehavior() = default;
  virtual void set_goal(const Pose2D& goal) = 0;
  virtual void cancel() = 0;
  virtual BehaviorOutput update(const BehaviorInput& in) = 0;
  virtual BehaviorState state() const = 0;
};
```

Tick contract: set a goal once, then call `update()` each tick to get the
velocity command, the FSM state, and (for visualization) the current global plan.
`IBehavior` is the "Planning + Navigation" unit: the default implementation runs
the full goal-driven FSM (global A* + DWA + recovery). `BehaviorState` mirrors the
navigator's internal `NavState` one-to-one.

---

## 4. The `CostmapView` pairing contract

`amr_api` does not depend on `amr_planning` (where the concrete `Costmap` and its
chamfer inflation live), so planners and behaviors take a read-only
`amr_api::CostmapView`. The reference backend builds one in `amr_planning`:

```cpp
#include "amr_planning/costmap_adapter.hpp"

std::unique_ptr<amr_api::CostmapView> costmap =
    amr_planning::make_costmap(grid, amr_core::CostmapConfig{}, robot.radius);
```

Internally the reference planners recover the concrete costmap with
`amr_planning::as_costmap(view)`. **A planner and the costmap it consumes are a
matched pair.** If your `core` swaps the costmap representation, it must also swap
the planners that read it (or supply a `CostmapView` its planners understand).

---

## 5. Diagnostics seams (`amr_api/diagnostics.hpp`)

`core` owns logging and the clock. `amr_api` defines only the seams (`Logger`,
`Clock`) plus a `NullLogger`, so adapters run standalone in gtests. The reference
core implements them like this (from `amr_reference_core/core_bus.hpp`):

```cpp
class StderrLogger : public amr_api::Logger {
 public:
  void log(Level level, const std::string& msg) override {
    const char* tag = "INFO";
    switch (level) {
      case Level::Debug: tag = "DEBUG"; break;
      case Level::Info:  tag = "INFO";  break;
      case Level::Warn:  tag = "WARN";  break;
      case Level::Error: tag = "ERROR"; break;
    }
    std::fprintf(stderr, "[core][%s] %s\n", tag, msg.c_str());
  }
};

class SimClock : public amr_api::Clock {
 public:
  double now() const override { return t_; }
  void advance(double dt) { t_ += dt; }
 private:
  double t_{0.0};
};
```

Using a monotonic sim clock (rather than wall time) keeps runs deterministic and
replayable — pass `clock.now()` into `BehaviorInput::now`.

---

## 6. Wiring a core: step by step

The complete, runnable example is
`src/amr_reference_core/test/test_reference_core.cpp` (`NavMissionReachesGoal`).
The shared blackboard is `CoreBus`:

```cpp
struct CoreBus {
  amr_api::OccupancyGrid map;
  amr_api::Pose2D pose;
  amr_api::Twist2D vel;
  amr_api::Path plan;
  amr_api::ParticleCloud particles;
  StderrLogger logger;
  SimClock clock;
};
```

The navigation loop, annotated:

```cpp
amr_reference_core::CoreBus bus;
bus.map  = open_arena();                       // the live map (from SLAM or a file)
bus.pose = amr_core::Pose2D{0.5, 0.5, 0.0};
bus.vel  = amr_core::Twist2D{0.0, 0.0};

amr_core::RobotConfig robot;
amr_core::CostmapConfig costmap_cfg;

// (1) Core owns the costmap, built via the planning backend.
auto costmap = amr_planning::make_costmap(bus.map, costmap_cfg, robot.radius);

// (2) The "Planning + Navigation" module, behind the interface.
std::unique_ptr<amr_api::IBehavior> behavior =
    std::make_unique<amr_navigation::NavigatorBehavior>(
        amr_core::NavConfig{}, amr_core::AstarConfig{}, amr_core::DwaConfig{},
        robot);

// (3) Set the goal once.
const amr_core::Pose2D goal{3.0, 3.0, 0.0};
behavior->set_goal(goal);

// (4) Tick the FSM. Core feeds pure data in and routes the command out.
const double dt = 0.1;
for (int ticks = 0; ticks < 600; ++ticks) {
  amr_api::BehaviorInput in;
  in.pose    = bus.pose;
  in.vel     = bus.vel;
  in.costmap = costmap.get();
  in.now     = bus.clock.now();

  const amr_api::BehaviorOutput out = behavior->update(in);
  bus.vel = out.cmd;
  if (out.plan.has_value()) bus.plan = *out.plan;

  // (5) Hand the command to the robot / simulator; here we integrate a unicycle.
  bus.pose.x += out.cmd.v * std::cos(bus.pose.theta) * dt;
  bus.pose.y += out.cmd.v * std::sin(bus.pose.theta) * dt;
  bus.pose.theta += out.cmd.omega * dt;
  bus.clock.advance(dt);

  if (out.state == amr_api::BehaviorState::SUCCEEDED ||
      out.state == amr_api::BehaviorState::FAILED) break;
}
```

A SLAM-then-NAV mission follows the same shape: in SLAM mode, feed
`SlamInput{odom_delta, scan}` to `ISlam::update` and copy `ISlam::map()` into
`bus.map`; switch to NAV mode using `ILocalizer` for the pose and `IBehavior` for
the command.

---

## 7. Swapping a module

To replace, for example, SLAM with your own implementation:

1. In your `core` repo, write a class that implements `amr_api::ISlam`.
2. Construct it instead of `amr_slam::ScanMatchingSlamAdapter`.
3. Keep every other wire identical — the rest of the stack only sees `ISlam`.

The same pattern applies to any interface. Granularity:

- Replace the whole "Planning + Navigation" unit by implementing `IBehavior`.
- Or keep the default behavior FSM and replace only the global or local planner
  by implementing `IGlobalPlanner` / `ILocalPlanner` and driving them in your own
  orchestration loop.

Because the interfaces are pure data, your implementation can live entirely in
your private repo and link only `amr_api` (plus `amr_core` for the types).

---

## 8. Building a core against `amr_api` from an external repo

In your package's `CMakeLists.txt`:

```cmake
find_package(amr_core REQUIRED)
find_package(amr_api REQUIRED)
find_package(amr_planning REQUIRED)      # only the modules you reuse
find_package(amr_navigation REQUIRED)

ament_target_dependencies(<your_target>
  amr_core amr_api amr_planning amr_navigation)
```

In `package.xml`:

```xml
<depend>amr_core</depend>
<depend>amr_api</depend>
<depend>amr_planning</depend>
<depend>amr_navigation</depend>
```

Source this workspace's overlay before building so `find_package` resolves:

```bash
micromamba run -n ros2_humble bash -c '
  cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  source install/setup.bash && \
  colcon build --packages-select <your_package>'
```

If you reuse the seeded `MclAdapter`, hold it via `std::unique_ptr` — it is
non-copyable/non-movable (it owns the RNG the underlying filter references).

---

## 9. Determinism & faithfulness

- The stochastic seam (RNG seed) lives in the adapter, not the interface — e.g.
  `MclAdapter(grid, cfg, seed, initial_pose)`. Same seed → bit-identical runs.
- Every numeric constant comes from an `amr_core::*Config` default; nothing is
  hardcoded (`CONTRACT.md` §9).
- Each adapter has a parity gtest asserting its output equals the raw class's
  output on identical inputs — the adapters are provably pass-throughs.
- All `CONTRACT.md` §2 conventions (frames, units, occupancy semantics) hold
  unchanged; the adapters carry `amr_core` types, adding no conversion semantics.

---

## 10. Verification

Build and test the whole workspace (the `CONTRACT.md` §8 gate):

```bash
micromamba run -n ros2_humble bash -c '
  cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build && colcon test && colcon test-result --all'
```

Expected: every package builds; the 43 pre-existing gtests still pass; the new
adapter parity tests and the `amr_reference_core` mission test pass; the
`amr_bringup` launch-testing demo passes. The integration adds tests on top —
it never changes the verified algorithm code.

---

## 11. Reference

- [`CONTRACT.md`](../CONTRACT.md) — §1 dependency graph, §11 data↔topic mapping,
  `CONTRACT_VERSION` 1.0.
- [`src/amr_api/README.md`](../src/amr_api/README.md) — interface list and the
  costmap pairing contract.
- `docs/superpowers/specs/2026-06-16-amr-core-integration-design.md` — the design
  rationale.
- `src/amr_reference_core/` — the runnable reference orchestrator and its gtest.
