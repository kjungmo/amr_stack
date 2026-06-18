// SPDX-License-Identifier: Apache-2.0
// NavigatorBehavior: implements amr_api::IBehavior by wrapping the default
// Navigator (built via make_default, wiring real A* + DWA). The behaviour FSM is
// the "Planning + Navigation" swappable unit; core may instead supply its own
// IBehavior implementation.
#pragma once

#include "amr_api/behavior.hpp"
#include "amr_core/config.hpp"
#include "amr_navigation/navigator.hpp"

namespace amr_navigation {

class NavigatorBehavior : public amr_api::IBehavior {
 public:
  NavigatorBehavior(const amr_core::NavConfig& nav_cfg,
                    const amr_core::AstarConfig& astar_cfg,
                    const amr_core::DwaConfig& dwa_cfg,
                    const amr_core::RobotConfig& robot_cfg)
      : nav_(Navigator::make_default(nav_cfg, astar_cfg, dwa_cfg, robot_cfg)) {}

  void set_goal(const amr_core::Pose2D& goal) override { nav_.set_goal(goal); }
  void cancel() override { nav_.cancel(); }
  amr_api::BehaviorOutput update(const amr_api::BehaviorInput& in) override;
  amr_api::BehaviorState state() const override { return to_api(nav_.state()); }

 private:
  static amr_api::BehaviorState to_api(NavState s);
  Navigator nav_;
};

}  // namespace amr_navigation
