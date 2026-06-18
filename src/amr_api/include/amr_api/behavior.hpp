// SPDX-License-Identifier: Apache-2.0
// IBehavior: standardized goal-driven behavior (navigation FSM) interface.
#pragma once

#include <optional>

#include "amr_api/costmap_view.hpp"
#include "amr_api/types.hpp"

namespace amr_api {

enum class BehaviorState {
  IDLE,
  PLANNING,
  FOLLOWING,
  RECOVERY,
  SUCCEEDED,
  FAILED
};

/// Stable label, matching the amr_navigation NavState strings and the
/// amr_interfaces feedback/result `state` field values.
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
  std::optional<Path> plan;  // current global plan, for viz
};

class IBehavior {
 public:
  virtual ~IBehavior() = default;

  virtual void set_goal(const Pose2D& goal) = 0;
  virtual void cancel() = 0;
  virtual BehaviorOutput update(const BehaviorInput& in) = 0;
  virtual BehaviorState state() const = 0;
};

}  // namespace amr_api
