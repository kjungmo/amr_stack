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
