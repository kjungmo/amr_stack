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
