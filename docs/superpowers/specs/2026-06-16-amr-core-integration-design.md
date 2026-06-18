# Design Spec: `amr_api` — Standardized Module Interface Layer for Core Integration

- **Date:** 2026-06-16
- **Branch:** `humble`
- **Status:** Approved (design); ready for implementation planning
- **Author:** kangjmo (with Claude Code)
- **Supersedes:** none
- **Source of truth it extends:** `CONTRACT.md` (frozen ROS 2 C++ port contract)

---

## 1. Problem & goal

The user maintains a **private, separate repository ("core")** — a top-level
brain/orchestrator that is *also* a shared-memory bus carrying logging and
utility functions. The goal is to let `core` **drive** `amr_stack`'s perception
and motion modules (SLAM, localization, mapping, planning, navigation) through
**stable, pure-data interfaces**, swapping any module with `core`'s own
proprietary implementation without touching the rest of the stack.

Today the modules are clean C++ libraries with node-free `*_lib` targets
(43/43 gtests pass), but each exposes a *bespoke* surface (`ScanMatchingSlam::process`,
`MonteCarloLocalizer::predict/correct/estimate`, `Navigator::update`, free
function `plan_path`, …). There is no single, uniform contract a host
orchestrator can program against.

**Goal:** add a thin, node-free **interface layer (`amr_api`)** plus per-module
**adapters** so that:

1. Every module is reachable through a small set of abstract C++ interfaces with
   plain-data inputs/outputs.
2. The existing, gtest-verified algorithm code stays **byte-for-byte untouched**
   (adapters delegate; they do not modify).
3. `core` can inject its own implementation of any interface and own the bus,
   the clock, and logging.
4. A copy-pasteable **reference core** demonstrates the wiring end-to-end and
   doubles as the integration guide's worked example.

Out of scope: rewriting the ROS nodes to route through the interfaces (the ROS
layer is *frozen + version-stamped* per the decision in §2); changing any
algorithm constant or step; touching the `jazzy`/`main` branches.

---

## 2. Approved decisions (recap)

From two rounds of clarification, the user approved:

| Decision | Choice |
|---|---|
| Layer boundary | **Both layers** — pure-C++ interface layer *and* ROS layer awareness |
| `core`'s role | **Becomes the brain** — top-level orchestrator + shared-memory bus + logging/utils |
| Swappable modules | **SLAM, Localization, Mapping, Planning + Navigation** |
| Wiring style | **Pure-data; core owns the bus** (modules exchange plain structs, never ROS msgs) |
| ROS layer treatment | **Freeze + version existing** (no node rewrite; stamp + document the data↔topic mapping) |
| Approach | **A — Adapter layer** (vs. B, modify classes in place) |
| Interface package name | **`amr_api`** (avoids collision with existing shared-types lib `amr_core`) |
| Costmap typing | lightweight read-only **`amr_api::CostmapView`** abstraction |
| Adapter location | **inside each module package** (not all gathered in `amr_api`) |

---

## 3. Architecture overview

```
                         ┌───────────────────────────────┐
                         │  core  (PRIVATE, external)     │
                         │  brain · bus · clock · logging │
                         └───────────────┬───────────────┘
                                         │ programs against
                                         ▼
                         ┌───────────────────────────────┐
                         │  amr_api   (NEW, node-free)    │
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
   (each module package gains a dependency on amr_api + an adapter + parity gtest)

   ┌───────────────────────────────────────────────────────────────────┐
   │  amr_reference_core (NEW, in-tree)                                  │
   │  CoreBus + Logger/Clock impl + tick loop; wires the 6 adapters;     │
   │  headless SLAM→NAV mission gtest = guide's worked example           │
   └───────────────────────────────────────────────────────────────────┘
```

Dependency direction stays strictly one-way and downward, consistent with
CONTRACT.md §1. `amr_api` sits just above `amr_core` and below every module.

---

## 4. The `amr_api` package

A new `ament_cmake` package. Almost entirely headers; one small `.cpp`
(`to_string(BehaviorState)`). **Depends only on `amr_core`.** No ROS, no node.

### 4.1 Data types — `amr_api/types.hpp`

Reuse `amr_core` types verbatim (no duplication) and add the few aggregates the
interfaces need:

```cpp
namespace amr_api {
using amr_core::Pose2D;
using amr_core::Twist2D;
using amr_core::LaserScan;
using amr_core::OccupancyGrid;

/// World (x, y) waypoints — identical shape to amr_planning::Path.
using Path = std::vector<std::array<double, 2>>;

/// Particle filter snapshot (for viz / diagnostics).
struct ParticleCloud {
  std::vector<Pose2D> poses;
  std::vector<double> weights;
};
}  // namespace amr_api
```

### 4.2 Read-only costmap — `amr_api/costmap_view.hpp`

`amr_api` must not depend on `amr_planning` (where the concrete `Costmap`
lives), yet planners and the behavior FSM need a costmap. Solution: an abstract
read-only view mirroring the *read* surface of `amr_planning::Costmap`.

```cpp
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

  virtual float cost_at(int row, int col) const = 0;       // [0, 1]
  virtual double cost_at_world(double x, double y) const = 0;
  virtual bool is_lethal(int row, int col) const = 0;
};
}  // namespace amr_api
```

**Construction:** costmap inflation (chamfer) lives in `amr_planning`. The
reference backend therefore ships a factory in `amr_planning`:

```cpp
// amr_planning/costmap_adapter.hpp
std::unique_ptr<amr_api::CostmapView> make_costmap(
    const amr_core::OccupancyGrid& grid,
    const amr_core::CostmapConfig& cfg, double robot_radius);
```

The returned object (`CostmapAdapter`) owns a real `amr_planning::Costmap` and
forwards every query. **Bridging contract:** the default `amr_planning` planners
recover the concrete `Costmap` from a `CostmapView` via a contained
`static_cast` to `CostmapAdapter` inside `amr_planning` (safe because the
reference planners are only ever paired with the reference costmap). This keeps
the verified A*/DWA code untouched. A `core` that swaps the costmap
representation must supply a matching planner — documented as a paired contract
in `amr_api/README.md`.

### 4.3 The six interfaces

All inputs/outputs are plain structs of `amr_core`/`amr_api` types. All methods
are tick-style and side-effect-explicit. Pointers in request structs are
non-owning and must outlive the call.

**`amr_api/slam.hpp`** — `amr_slam::ScanMatchingSlam`
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

**`amr_api/localizer.hpp`** — `amr_localization::MonteCarloLocalizer`
```cpp
struct LocalizerInput {
  Pose2D odom_delta;
  std::optional<LaserScan> scan;  // correct() runs only when present
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
*Adapter bridges the 3-step predict/correct/estimate to one `update()`:*
`predict(odom_delta)`; if `scan`, `correct(*scan)`; return `estimate()` + cloud.

**`amr_api/mapper.hpp`** — `amr_mapping::OccupancyGridMapper`
```cpp
struct MapperInput {
  Pose2D pose;
  LaserScan scan;
};
class IMapper {
 public:
  virtual ~IMapper() = default;
  virtual void integrate(const MapperInput& in) = 0;  // -> OccupancyGridMapper::update
  virtual OccupancyGrid map() const = 0;
};
```

**`amr_api/global_planner.hpp`** — `amr_planning::plan_path`
```cpp
struct GlobalPlanRequest {
  std::array<double, 2> start_xy;
  std::array<double, 2> goal_xy;
  const CostmapView* costmap;  // non-owning
};
class IGlobalPlanner {
 public:
  virtual ~IGlobalPlanner() = default;
  virtual std::optional<Path> plan(const GlobalPlanRequest& req) = 0;
};
```

**`amr_api/local_planner.hpp`** — `amr_planning::DwaPlanner`
```cpp
struct LocalPlanRequest {
  Pose2D pose;
  Twist2D vel;
  const Path* path;            // non-owning
  const CostmapView* costmap;  // non-owning
};
struct LocalPlanResult {
  Twist2D cmd;
  bool blocked;
};
class ILocalPlanner {
 public:
  virtual ~ILocalPlanner() = default;
  virtual LocalPlanResult compute(const LocalPlanRequest& req) = 0;
};
```

**`amr_api/behavior.hpp`** — `amr_navigation::Navigator`
```cpp
enum class BehaviorState { IDLE, PLANNING, FOLLOWING, RECOVERY, SUCCEEDED, FAILED };
const char* to_string(BehaviorState s);  // stable labels (match NavState strings)

struct BehaviorInput {
  Pose2D pose;
  Twist2D vel;
  const CostmapView* costmap;  // non-owning
  double now;                  // seconds, from core's Clock
};
struct BehaviorOutput {
  Twist2D cmd;
  BehaviorState state;
  std::optional<Path> plan;    // current global plan (viz), if any
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
The default `IBehavior` (Navigator) internally drives an `IGlobalPlanner` +
`ILocalPlanner`; `core` may inject its own planners into the default behavior
*or* replace the whole behavior. `BehaviorState` mirrors `NavState` 1:1; the
adapter maps between them (`amr_api` cannot depend on `amr_navigation`).

### 4.4 Diagnostics seams — `amr_api/diagnostics.hpp`

`core` owns logging and the clock; `amr_api` defines only the seams plus no-op
defaults so modules/adapters run standalone (e.g. in gtests) without a core.

```cpp
namespace amr_api {
class Logger {
 public:
  enum class Level { Debug, Info, Warn, Error };
  virtual ~Logger() = default;
  virtual void log(Level level, const std::string& msg) = 0;
};
class NullLogger : public Logger {
 public:
  void log(Level, const std::string&) override {}
};
class Clock {
 public:
  virtual ~Clock() = default;
  virtual double now() const = 0;  // seconds
};
}  // namespace amr_api
```

Logging stays at the **adapter boundary** (adapters accept a nullable
`Logger*`); it is never injected into the verified algorithm classes.

### 4.5 Versioning — `amr_api/version.hpp`

```cpp
namespace amr_api {
inline constexpr int VERSION_MAJOR = 0;
inline constexpr int VERSION_MINOR = 1;
inline constexpr int VERSION_PATCH = 0;
inline constexpr const char* VERSION = "0.1.0";
inline constexpr const char* CONTRACT_VERSION = "1.0";  // CONTRACT.md revision
}  // namespace amr_api
```

Every node logs `amr_api::VERSION` + `CONTRACT_VERSION` once at startup
(the only ROS-layer code change — see §6).

---

## 5. Adapters (one per interface, inside the owning module package)

Each module package gains: a dependency on `amr_api`, an adapter
header+source compiled into its existing `*_lib` target, and a parity gtest.

| Package | Adapter class | Implements | Delegates to |
|---|---|---|---|
| `amr_slam` | `ScanMatchingSlamAdapter` | `ISlam` | `ScanMatchingSlam::process/pose/get_map` |
| `amr_localization` | `MclAdapter` | `ILocalizer` | `MonteCarloLocalizer::predict/correct/estimate/particles/weights` |
| `amr_mapping` | `OccupancyGridMapperAdapter` | `IMapper` | `OccupancyGridMapper::update/to_occupancy_grid` |
| `amr_planning` | `AstarGlobalPlanner` | `IGlobalPlanner` | `plan_path` |
| `amr_planning` | `DwaLocalPlanner` | `ILocalPlanner` | `DwaPlanner::compute` |
| `amr_planning` | `CostmapAdapter` + `make_costmap` | `CostmapView` | `Costmap` |
| `amr_navigation` | `NavigatorBehavior` | `IBehavior` | `Navigator` (via injected PlanFn/ControlFn that call the API planners) |

Adapters are thin: construct the delegate from the same config structs, forward
each call, translate struct↔struct and enum↔enum. No business logic.

**Parity gtest per adapter:** drive the adapter and the raw class with identical
inputs and assert bit-identical outputs (pose components, command twist, map
cells, path waypoints). This proves the adapter is a pure pass-through and that
the 43/43 verified behaviors are preserved.

---

## 6. ROS Layer-2 changes (light — "freeze + version")

No node is rewritten to route through the interfaces. Two minimal additions:

1. **Version stamp:** each node logs `amr_api::VERSION` + `CONTRACT_VERSION` once
   on startup (`RCLCPP_INFO`).
2. **Data-model ↔ topic mapping table** added to `CONTRACT.md` as a new §11, e.g.
   `ISlam::map()`/`IMapper::map()` ↔ `/map`; `ILocalizer::cloud` ↔ `/particles`;
   `LocalizerOutput::pose` ↔ `/pose`; `BehaviorOutput::cmd` ↔ `/cmd_vel`;
   `BehaviorOutput::plan` ↔ `/plan`; `SlamInput/LocalizerInput.scan` ↔ `/scan`;
   `*.odom_delta` derived from `/odom`; `IBehavior::set_goal` ↔ `/goal_pose`.

This documents how a `core` running as (or alongside) a node maps the pure-data
bus onto the frozen ROS topic contract, without changing the nodes themselves.

---

## 7. Reference core (`amr_reference_core`, new in-tree package)

A minimal, dependency-light demonstration that *is* the guide's worked example
and a CI test. Depends on `amr_api` + all five module libs.

- `include/amr_reference_core/core_bus.hpp` — the shared-memory blackboard:
  owns the live `OccupancyGrid` map, latest `Pose2D`, `Twist2D`, `Path`,
  `ParticleCloud`; provides a concrete `Logger` (stderr) and `Clock`
  (monotonic sim seconds).
- `src/reference_core.cpp` — wires the six adapters through the interfaces and
  runs a tick loop:
  - **SLAM mode:** feed `/scan`+`/odom`-equivalents to `ISlam::update`; publish
    `ISlam::map()` into the bus.
  - **NAV mode:** `ILocalizer::update` for pose; `IBehavior::update` (driving the
    injected `IGlobalPlanner`/`ILocalPlanner` over a `CostmapView` built by
    `make_costmap`) for `cmd`.
- `test/test_reference_core.cpp` — headless mission: SLAM a short trajectory,
  save the map, localize + navigate to a goal on it; assert the CONTRACT.md §7
  DEMO-style criteria (reaches goal, ground-truth error < tol, no collision).

The reference core deliberately mirrors what the user's private `core` will do,
so "implant core" = "replace `amr_reference_core` wiring with your repo, keep the
interfaces."

---

## 8. Testing & verification

- **Parity gtests** (one per adapter) — adapter output ≡ raw class output.
- **Reference-core mission gtest** — end-to-end SLAM→NAV success headlessly.
- **Full gate (CONTRACT.md §8):** `colcon build && colcon test &&
  colcon test-result --all` in the RoboStack `ros2_humble` env. The existing
  43/43 must still pass unchanged; new tests add on top.
- **Faithfulness (CONTRACT.md §9):** adapters pull every constant from
  `amr_core::*Config`; no hardcoded numbers; no RNG outside the seeded paths.

---

## 9. Deliverables

1. `src/amr_api/` — package (interfaces, types, `costmap_view`, `diagnostics`,
   `version`) + `README.md` (interface contract, paired-costmap rule, examples).
2. Adapters in `amr_slam`, `amr_localization`, `amr_mapping`, `amr_planning`,
   `amr_navigation` + parity gtests.
3. `src/amr_reference_core/` + headless mission gtest.
4. `CONTRACT.md` updates: `amr_api` in the §1 dependency graph; new §11
   data-model↔topic mapping; `CONTRACT_VERSION` note.
5. `docs/CORE_INTEGRATION.md` — the integration guide (English), walking through
   each interface, the adapter pattern, and the reference-core example.
6. Korean translation of the guide (per global Korean-register rules).
7. Both guide versions saved to the `general_vault` via the `obsidian-vault-save`
   skill.

---

## 10. Faithfulness & constraints

- Verified algorithm classes are **not modified**; adapters delegate only.
- All constants come from `amr_core::*Config`; CONTRACT.md §2/§9 conventions hold.
- No `Generated with Claude Code` / `Co-Authored-By` footers anywhere.
- Korean guide: 존댓말, 직역체 회피, no English-adjective/Korean mixing, no
  English-Korean slashes, CS terms rendered meaning-first in Korean.
- SPDX `Apache-2.0` header on every new file.

---

## 11. Risks & open questions

| Item | Resolution |
|---|---|
| `CostmapView`→`Costmap` downcast in default planners | Contained within `amr_planning`; documented paired-contract; verified A*/DWA untouched. Acceptable. |
| SLAM "map dirty" signal | `ScanMatchingSlam` exposes no keyframe event; `core` polls `ISlam::map()` at its own cadence. No class change. |
| Adding two new packages (`amr_api`, `amr_reference_core`) | Both follow `EXTENDING.md` template; one-way deps preserved. |
| Logger/Clock injection scope | Adapter boundary only; algorithm classes stay seam-free. |
| `BehaviorState` vs `NavState` drift | Enum mirrored 1:1 with a mapping check in the navigator adapter parity gtest. |

---

## 12. Implementation order

1. `amr_api` package — interfaces/types/version compile against `amr_core`.
2. `CostmapAdapter` + `make_costmap` in `amr_planning`.
3. Per-module adapters + parity gtests (steps 3a–3e, parallelizable).
4. `amr_reference_core` + headless mission gtest.
5. Full `colcon build && colcon test` gate (43/43 + new tests green).
6. `CONTRACT.md` updates (§1 graph, §11 mapping, version note).
7. `docs/CORE_INTEGRATION.md` (EN) → Korean translation → save both to vault.
