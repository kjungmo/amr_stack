// SPDX-License-Identifier: Apache-2.0
// Parity tests: the CostmapView / IGlobalPlanner / ILocalPlanner adapters must
// produce output identical to the raw amr_planning calls on the same inputs.
#include <array>

#include <gtest/gtest.h>

#include "amr_api/global_planner.hpp"
#include "amr_api/local_planner.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/astar.hpp"
#include "amr_planning/costmap.hpp"
#include "amr_planning/costmap_adapter.hpp"
#include "amr_planning/dwa.hpp"
#include "amr_planning/global_planner_adapter.hpp"
#include "amr_planning/local_planner_adapter.hpp"

namespace {
amr_core::OccupancyGrid free_grid() {
  amr_core::OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = 40;
  g.cols = 40;
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);
  return g;
}
}  // namespace

TEST(PlanningAdapters, CostmapViewMatchesRaw) {
  const auto grid = free_grid();
  amr_core::CostmapConfig cfg;
  const double r = 0.15;
  amr_planning::Costmap raw(grid, cfg, r);
  auto view = amr_planning::make_costmap(grid, cfg, r);

  ASSERT_EQ(view->rows(), raw.rows());
  ASSERT_EQ(view->cols(), raw.cols());
  for (int row = 0; row < raw.rows(); ++row) {
    for (int col = 0; col < raw.cols(); ++col) {
      EXPECT_FLOAT_EQ(view->cost_at(row, col), raw.cost_at(row, col));
    }
  }
}

TEST(PlanningAdapters, GlobalPlannerMatchesRaw) {
  const auto grid = free_grid();
  amr_core::CostmapConfig ccfg;
  amr_core::AstarConfig acfg;
  const double r = 0.15;
  amr_planning::Costmap raw(grid, ccfg, r);
  auto view = amr_planning::make_costmap(grid, ccfg, r);

  const std::array<double, 2> start{0.2, 0.2};
  const std::array<double, 2> goal{1.6, 1.6};

  const auto expected = amr_planning::plan_path(raw, start, goal, acfg);
  amr_planning::AstarGlobalPlanner planner(acfg);
  amr_api::GlobalPlanRequest req{start, goal, view.get()};
  const auto got = planner.plan(req);

  ASSERT_EQ(expected.has_value(), got.has_value());
  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(expected->size(), got->size());
  for (std::size_t i = 0; i < got->size(); ++i) {
    EXPECT_DOUBLE_EQ((*expected)[i][0], (*got)[i][0]);
    EXPECT_DOUBLE_EQ((*expected)[i][1], (*got)[i][1]);
  }
}

TEST(PlanningAdapters, LocalPlannerMatchesRaw) {
  const auto grid = free_grid();
  amr_core::CostmapConfig ccfg;
  amr_core::DwaConfig dcfg;
  amr_core::RobotConfig rob;
  const double r = rob.radius;
  amr_planning::Costmap raw(grid, ccfg, r);
  auto view = amr_planning::make_costmap(grid, ccfg, r);

  amr_core::Pose2D pose{0.2, 0.2, 0.0};
  amr_core::Twist2D vel{0.0, 0.0};
  amr_api::Path path{{0.2, 0.2}, {0.6, 0.2}, {1.0, 0.2}};

  amr_planning::DwaPlanner raw_planner(dcfg, rob);
  const auto expected = raw_planner.compute(pose, vel, path, raw);

  amr_planning::DwaLocalPlanner planner(dcfg, rob);
  amr_api::LocalPlanRequest req{pose, vel, &path, view.get()};
  const auto got = planner.compute(req);

  EXPECT_DOUBLE_EQ(expected.cmd.v, got.cmd.v);
  EXPECT_DOUBLE_EQ(expected.cmd.omega, got.cmd.omega);
  EXPECT_EQ(expected.blocked, got.blocked);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
