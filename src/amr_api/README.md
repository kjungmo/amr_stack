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
runnable example of a host orchestrator driving the modules end to end.
