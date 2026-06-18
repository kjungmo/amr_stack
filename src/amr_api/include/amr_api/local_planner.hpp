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
