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
