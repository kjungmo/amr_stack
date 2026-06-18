// SPDX-License-Identifier: Apache-2.0
// Parity test: NavigatorBehavior (IBehavior) == raw Navigator (make_default) on
// identical inputs; plus BehaviorState labels match NavState labels 1:1.
#include <gtest/gtest.h>

#include "amr_api/behavior.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_navigation/navigator.hpp"
#include "amr_navigation/navigator_behavior.hpp"
#include "amr_planning/costmap.hpp"
#include "amr_planning/costmap_adapter.hpp"

namespace {
amr_core::OccupancyGrid free_grid() {
  amr_core::OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = 80;
  g.cols = 80;
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);
  return g;
}
}  // namespace

TEST(NavigatorBehavior, MatchesRaw) {
  amr_core::NavConfig ncfg;
  amr_core::AstarConfig acfg;
  amr_core::DwaConfig dcfg;
  amr_core::RobotConfig rob;
  amr_core::CostmapConfig ccfg;
  const auto grid = free_grid();
  amr_planning::Costmap raw_cm(grid, ccfg, rob.radius);
  auto view = amr_planning::make_costmap(grid, ccfg, rob.radius);

  amr_navigation::Navigator raw =
      amr_navigation::Navigator::make_default(ncfg, acfg, dcfg, rob);
  amr_navigation::NavigatorBehavior adp(ncfg, acfg, dcfg, rob);

  const amr_core::Pose2D goal{2.0, 2.0, 0.0};
  raw.set_goal(goal);
  adp.set_goal(goal);

  amr_core::Pose2D pose{0.5, 0.5, 0.0};
  amr_core::Twist2D vel{0.0, 0.0};
  for (int i = 0; i < 5; ++i) {
    const double now = 0.1 * i;
    const amr_core::Twist2D cr = raw.update(pose, vel, raw_cm, now);
    const auto out =
        adp.update(amr_api::BehaviorInput{pose, vel, view.get(), now});
    EXPECT_DOUBLE_EQ(cr.v, out.cmd.v);
    EXPECT_DOUBLE_EQ(cr.omega, out.cmd.omega);
    EXPECT_STREQ(amr_navigation::to_string(raw.state()),
                 amr_api::to_string(out.state));
    vel = cr;
  }
}

TEST(NavigatorBehavior, StateLabelsMatchNavState) {
  using amr_api::BehaviorState;
  using amr_navigation::NavState;
  EXPECT_STREQ(amr_navigation::to_string(NavState::IDLE),
               amr_api::to_string(BehaviorState::IDLE));
  EXPECT_STREQ(amr_navigation::to_string(NavState::PLANNING),
               amr_api::to_string(BehaviorState::PLANNING));
  EXPECT_STREQ(amr_navigation::to_string(NavState::FOLLOWING),
               amr_api::to_string(BehaviorState::FOLLOWING));
  EXPECT_STREQ(amr_navigation::to_string(NavState::RECOVERY),
               amr_api::to_string(BehaviorState::RECOVERY));
  EXPECT_STREQ(amr_navigation::to_string(NavState::SUCCEEDED),
               amr_api::to_string(BehaviorState::SUCCEEDED));
  EXPECT_STREQ(amr_navigation::to_string(NavState::FAILED),
               amr_api::to_string(BehaviorState::FAILED));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
