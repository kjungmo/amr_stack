// SPDX-License-Identifier: Apache-2.0
// Unit tests for amr_planning: costmap inflation, A*, DWA. Mirrors the Python
// unit tests in amr_stack/tests for planning.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/astar.hpp"
#include "amr_planning/costmap.hpp"
#include "amr_planning/dwa.hpp"

namespace {

using amr_core::OccupancyGrid;
using amr_planning::Costmap;

// Build an empty (all-free) grid with the given size and resolution.
OccupancyGrid make_free_grid(int rows, int cols, double res = 0.05) {
  OccupancyGrid g;
  g.resolution = res;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = rows;
  g.cols = cols;
  g.data.assign(static_cast<std::size_t>(rows) * cols, 0);  // 0 = free
  return g;
}

inline void set_occ(OccupancyGrid& g, int r, int c) {
  g.data[static_cast<std::size_t>(r) * g.cols + c] = 100;
}

}  // namespace

// ---------------------------------------------------------------------------
// Costmap: lethal core and decaying band.
// ---------------------------------------------------------------------------
TEST(Costmap, LethalCoreAndDecayBand) {
  // 40x40 grid at 0.05 m => 2.0 m square. One obstacle cell in the middle.
  OccupancyGrid g = make_free_grid(40, 40, 0.05);
  const int obs_r = 20, obs_c = 20;
  set_occ(g, obs_r, obs_c);

  amr_core::CostmapConfig cfg;  // robot is point; inflation handled by radius
  const double robot_radius = 0.18;
  Costmap cm(g, cfg, robot_radius);

  // The obstacle cell itself is lethal.
  EXPECT_TRUE(cm.is_lethal(obs_r, obs_c));
  EXPECT_FLOAT_EQ(cm.cost_at(obs_r, obs_c), 1.0f);

  // A cell within the inflated robot radius (one cell away, 0.05 m center
  // distance minus half-cell shift => ~0.025 m < 0.18 m) is lethal.
  EXPECT_TRUE(cm.is_lethal(obs_r, obs_c + 1));

  // A cell inside the decay band (between robot_radius and inflation_radius)
  // has a strictly-positive, sub-lethal cost.
  // inflation_radius default 0.45 m => ~9 cells. Pick ~6 cells out (0.30 m
  // center dist, ~0.275 m after shift) => band region.
  const int band_c = obs_c + 6;
  const float band_cost = cm.cost_at(obs_r, band_c);
  EXPECT_GT(band_cost, 0.0f);
  EXPECT_LT(band_cost, 0.99f);

  // Cost decays with distance within the band: a closer band cell costs more
  // than a farther one.
  const float near_band = cm.cost_at(obs_r, obs_c + 5);
  const float far_band = cm.cost_at(obs_r, obs_c + 8);
  EXPECT_GT(near_band, far_band);

  // Beyond the inflation radius the cost is exactly 0.
  EXPECT_FLOAT_EQ(cm.cost_at(obs_r, obs_c + 15), 0.0f);

  // Out-of-bounds cells are treated as lethal.
  EXPECT_TRUE(cm.is_lethal(-1, 0));
  EXPECT_TRUE(cm.is_lethal(0, g.cols));
}

// ---------------------------------------------------------------------------
// A*: finds a path around a wall and avoids lethal cells.
// ---------------------------------------------------------------------------
TEST(Astar, FindsPathAroundWall) {
  // 60x60 grid at 0.05 m => 3.0 m square.
  OccupancyGrid g = make_free_grid(60, 60, 0.05);
  // Vertical wall at col 30, from row 0 up to row 45 (leaves a gap near the
  // top so the planner must go around).
  const int wall_c = 30;
  for (int r = 0; r <= 45; ++r) {
    set_occ(g, r, wall_c);
  }

  amr_core::CostmapConfig ccfg;
  // Shrink inflation so the gap at the top stays passable on this small grid.
  ccfg.inflation_radius = 0.15;
  Costmap cm(g, ccfg, /*robot_radius=*/0.05);

  amr_core::AstarConfig acfg;  // simplify default true

  // Start left of the wall, goal right of the wall, both near the bottom.
  const std::array<double, 2> start{0.25, 0.25};
  const std::array<double, 2> goal{2.75, 0.25};

  auto path = amr_planning::plan_path(cm, start, goal, acfg);
  ASSERT_TRUE(path.has_value());
  ASSERT_GE(path->size(), 2u);

  // Endpoints are exact.
  EXPECT_NEAR((*path)[0][0], start[0], 1e-9);
  EXPECT_NEAR((*path)[0][1], start[1], 1e-9);
  EXPECT_NEAR(path->back()[0], goal[0], 1e-9);
  EXPECT_NEAR(path->back()[1], goal[1], 1e-9);

  // Every waypoint and every densely-sampled segment must be non-lethal.
  for (std::size_t i = 0; i + 1 < path->size(); ++i) {
    EXPECT_TRUE(amr_planning::has_line_of_sight(cm, (*path)[i][0], (*path)[i][1],
                                                (*path)[i + 1][0],
                                                (*path)[i + 1][1]));
  }

  // The path must detour upward (toward the gap) rather than tunneling the
  // wall: some waypoint has y well above the straight-line baseline of 0.25 m.
  double max_y = 0.0;
  for (const auto& p : *path) {
    max_y = std::max(max_y, p[1]);
  }
  EXPECT_GT(max_y, 1.5);
}

TEST(Astar, GoalInLethalReturnsNullopt) {
  OccupancyGrid g = make_free_grid(40, 40, 0.05);
  set_occ(g, 20, 20);
  amr_core::CostmapConfig ccfg;
  Costmap cm(g, ccfg, 0.18);
  amr_core::AstarConfig acfg;

  // Goal placed on the obstacle (lethal) -> no path.
  double gx, gy;
  cm.grid_to_world(20, 20, gx, gy);
  auto path = amr_planning::plan_path(cm, {0.1, 0.1}, {gx, gy}, acfg);
  EXPECT_FALSE(path.has_value());
}

TEST(Astar, StraightLineSimplifiesToTwoPoints) {
  OccupancyGrid g = make_free_grid(40, 40, 0.05);
  Costmap cm(g, amr_core::CostmapConfig{}, 0.05);
  amr_core::AstarConfig acfg;  // simplify true
  auto path = amr_planning::plan_path(cm, {0.25, 0.25}, {1.5, 0.25}, acfg);
  ASSERT_TRUE(path.has_value());
  // Open straight corridor: simplification collapses to just start + goal.
  EXPECT_EQ(path->size(), 2u);
}

// ---------------------------------------------------------------------------
// DWA: picks a sane v,w toward a straight-ahead carrot.
// ---------------------------------------------------------------------------
TEST(Dwa, DrivesTowardStraightAheadCarrot) {
  // Wide-open free grid, robot at origin facing +x, path straight ahead.
  OccupancyGrid g = make_free_grid(80, 80, 0.05);
  g.origin_x = -2.0;
  g.origin_y = -2.0;
  Costmap cm(g, amr_core::CostmapConfig{}, 0.05);

  amr_core::DwaConfig dcfg;
  amr_core::RobotConfig rcfg;
  amr_planning::DwaPlanner planner(dcfg, rcfg);

  amr_core::Pose2D pose{0.0, 0.0, 0.0};  // facing +x
  amr_core::Twist2D vel{0.0, 0.0};
  amr_planning::Path path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};

  auto res = planner.compute(pose, vel, path, cm);
  EXPECT_FALSE(res.blocked);
  // Forward velocity should be positive (drives toward the carrot).
  EXPECT_GT(res.cmd.v, 0.0);
  EXPECT_LE(res.cmd.v, rcfg.max_lin_vel + 1e-9);
  // Carrot is dead ahead: yaw rate should be near zero (no need to turn hard).
  EXPECT_LT(std::abs(res.cmd.omega), 0.6);
}

TEST(Dwa, TurnsTowardCarrotToTheLeft) {
  OccupancyGrid g = make_free_grid(120, 120, 0.05);
  g.origin_x = -3.0;
  g.origin_y = -3.0;
  Costmap cm(g, amr_core::CostmapConfig{}, 0.05);

  amr_planning::DwaPlanner planner(amr_core::DwaConfig{}, amr_core::RobotConfig{});

  amr_core::Pose2D pose{0.0, 0.0, 0.0};  // facing +x
  amr_core::Twist2D vel{0.0, 0.0};
  // Carrot is up and to the left -> expect a positive (CCW) yaw rate.
  amr_planning::Path path{{0.0, 0.0}, {0.5, 1.0}, {1.0, 2.0}};

  auto res = planner.compute(pose, vel, path, cm);
  EXPECT_FALSE(res.blocked);
  EXPECT_GT(res.cmd.omega, 0.0);
}

TEST(Dwa, CarrotPointAdvancesAlongPath) {
  amr_planning::Path path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  amr_core::Pose2D pose{0.0, 0.0, 0.0};
  auto c = amr_planning::carrot_point(path, pose, 0.8);
  EXPECT_NEAR(c[0], 0.8, 1e-9);
  EXPECT_NEAR(c[1], 0.0, 1e-9);
  // Lookahead past the end clamps to the final waypoint.
  auto cend = amr_planning::carrot_point(path, pose, 5.0);
  EXPECT_NEAR(cend[0], 2.0, 1e-9);
  EXPECT_NEAR(cend[1], 0.0, 1e-9);
}

TEST(Dwa, BlockedWhenAllRolloutsHitLethal) {
  // Fully occupied grid: every rollout pose (including the stay-put v=0,w=0
  // sample) lands on a lethal cell, so no rollout survives -> blocked.
  OccupancyGrid g = make_free_grid(40, 40, 0.05);
  g.origin_x = -1.0;
  g.origin_y = -1.0;
  for (auto& v : g.data) v = 100;  // 100 = occupied everywhere
  Costmap cm(g, amr_core::CostmapConfig{}, 0.05);

  amr_planning::DwaPlanner planner(amr_core::DwaConfig{}, amr_core::RobotConfig{});
  amr_core::Pose2D pose{0.0, 0.0, 0.0};
  amr_core::Twist2D vel{0.3, 0.0};  // already moving forward into the wall
  amr_planning::Path path{{0.0, 0.0}, {1.0, 0.0}};
  auto res = planner.compute(pose, vel, path, cm);
  EXPECT_TRUE(res.blocked);
  EXPECT_DOUBLE_EQ(res.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(res.cmd.omega, 0.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
