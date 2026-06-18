// SPDX-License-Identifier: Apache-2.0
#include "amr_navigation/navigator_behavior.hpp"

#include "amr_planning/costmap_adapter.hpp"

namespace amr_navigation {

amr_api::BehaviorState NavigatorBehavior::to_api(NavState s) {
  switch (s) {
    case NavState::IDLE: return amr_api::BehaviorState::IDLE;
    case NavState::PLANNING: return amr_api::BehaviorState::PLANNING;
    case NavState::FOLLOWING: return amr_api::BehaviorState::FOLLOWING;
    case NavState::RECOVERY: return amr_api::BehaviorState::RECOVERY;
    case NavState::SUCCEEDED: return amr_api::BehaviorState::SUCCEEDED;
    case NavState::FAILED: return amr_api::BehaviorState::FAILED;
  }
  return amr_api::BehaviorState::IDLE;
}

amr_api::BehaviorOutput NavigatorBehavior::update(
    const amr_api::BehaviorInput& in) {
  const amr_planning::Costmap& costmap = amr_planning::as_costmap(*in.costmap);
  const amr_core::Twist2D cmd = nav_.update(in.pose, in.vel, costmap, in.now);

  amr_api::BehaviorOutput out;
  out.cmd = cmd;
  out.state = to_api(nav_.state());
  if (nav_.path().has_value()) {
    out.plan = *nav_.path();
  }
  return out;
}

}  // namespace amr_navigation
