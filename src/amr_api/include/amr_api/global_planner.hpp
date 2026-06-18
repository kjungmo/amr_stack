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
