// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/geometry.hpp"
#include "amr_core/types.hpp"

using namespace amr_core;

TEST(Geometry, WrapAngle) {
  EXPECT_NEAR(wrap_angle(0.0), 0.0, 1e-9);
  EXPECT_NEAR(wrap_angle(M_PI), M_PI, 1e-9);   // (-pi, pi] includes +pi
  EXPECT_NEAR(wrap_angle(-M_PI), M_PI, 1e-9);  // -pi wraps to +pi
  EXPECT_NEAR(wrap_angle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(wrap_angle(M_PI / 2 + 2 * M_PI), M_PI / 2, 1e-9);
}

TEST(Geometry, PoseComposeBetweenRoundTrip) {
  const Pose2D a{1.0, 2.0, 0.5};
  const Pose2D b{0.3, -0.4, 0.2};
  const Pose2D c = pose_compose(a, b);
  const Pose2D rel = pose_between(a, c);
  EXPECT_NEAR(rel.x, b.x, 1e-9);
  EXPECT_NEAR(rel.y, b.y, 1e-9);
  EXPECT_NEAR(rel.theta, b.theta, 1e-9);
}

TEST(Types, GridRoundTrip) {
  OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = -1.0;
  g.origin_y = -2.0;
  g.rows = 100;
  g.cols = 200;
  int r = 0, c = 0;
  g.world_to_grid(0.0, 0.0, r, c);
  double x = 0.0, y = 0.0;
  g.grid_to_world(r, c, x, y);
  EXPECT_LE(std::abs(x - 0.0), g.resolution);
  EXPECT_LE(std::abs(y - 0.0), g.resolution);
  EXPECT_TRUE(g.in_bounds(r, c));
}

TEST(Geometry, DistanceField) {
  const int rows = 5, cols = 5;
  std::vector<uint8_t> occ(static_cast<size_t>(rows) * cols, 0);
  occ[2 * cols + 2] = 1;  // center occupied
  const auto d = distance_field(occ, rows, cols, 1.0, 10.0);
  EXPECT_NEAR(d[2 * cols + 2], 0.0f, 1e-6);
  EXPECT_NEAR(d[2 * cols + 0], 2.0f, 1e-6);              // two cells to the left
  EXPECT_NEAR(d[0 * cols + 0], std::sqrt(8.0f), 1e-5f);  // corner: 2 diagonals
}

TEST(Config, DefaultsAndOverride) {
  const AmrConfig cfg =
      load_config("", {"nav.goal_tol_xy=0.4", "localization.num_particles=300"});
  EXPECT_EQ(cfg.seed, 42);
  EXPECT_NEAR(cfg.robot.radius, 0.18, 1e-9);
  EXPECT_NEAR(cfg.nav.goal_tol_xy, 0.4, 1e-9);
  EXPECT_EQ(cfg.localization.num_particles, 300);
  // nested defaults survive an unrelated override
  EXPECT_NEAR(cfg.planning.dwa.sim_time, 1.5, 1e-9);
}

TEST(Config, UnknownKeyThrows) {
  EXPECT_THROW(load_config("", {"nav.bogus=1"}), ConfigError);
  EXPECT_THROW(load_config("", {"bogus.x=1"}), ConfigError);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
