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
