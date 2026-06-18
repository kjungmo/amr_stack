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
