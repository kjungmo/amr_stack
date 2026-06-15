// SPDX-License-Identifier: Apache-2.0
// Unit tests for the amr_sim library, mirroring the Python sim unit tests.
//   1. exact-arc kinematics for a known (v, omega, dt)
//   2. lidar range against a single known wall
//   3. collision flag when driven into a wall
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_sim/lidar.hpp"
#include "amr_sim/robot.hpp"
#include "amr_sim/simulator.hpp"
#include "amr_sim/world.hpp"

namespace {

// Build an empty res=0.05 world of (size_x, size_y) metres, spawn at origin.
amr_sim::World make_empty_world(double size_x, double size_y,
                                double res = 0.05) {
  const int rows = static_cast<int>(std::lround(size_y / res));
  const int cols = static_cast<int>(std::lround(size_x / res));
  amr_core::OccupancyGrid grid;
  grid.resolution = res;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.rows = rows;
  grid.cols = cols;
  grid.data.assign(static_cast<std::size_t>(rows) * cols, 0);
  return amr_sim::World(grid, amr_core::Pose2D{0.0, 0.0, 0.0}, {size_x, size_y});
}

}  // namespace

// --- 1. Exact-arc kinematics ----------------------------------------------

// arc_robot_frame: straight-line case (omega ~ 0) -> pure forward translation.
TEST(Kinematics, ArcStraightLine) {
  const amr_core::Pose2D d = amr_sim::arc_robot_frame(0.5, 0.0, 0.1);
  EXPECT_NEAR(d.x, 0.05, 1e-12);   // 0.5 m/s * 0.1 s
  EXPECT_NEAR(d.y, 0.0, 1e-12);
  EXPECT_NEAR(d.theta, 0.0, 1e-12);
}

// arc_robot_frame: a quarter turn. v=pi/2, w=pi/2, dt=1 -> R=1, dth=pi/2.
// dx = R sin(dth) = 1, dy = R (1 - cos(dth)) = 1, dth = pi/2.
TEST(Kinematics, ArcQuarterTurn) {
  const double w = M_PI / 2.0;
  const double v = M_PI / 2.0;  // R = v/w = 1
  const amr_core::Pose2D d = amr_sim::arc_robot_frame(v, w, 1.0);
  EXPECT_NEAR(d.x, 1.0, 1e-9);
  EXPECT_NEAR(d.y, 1.0, 1e-9);
  EXPECT_NEAR(d.theta, M_PI / 2.0, 1e-9);
}

// DiffDriveRobot.step world-frame exact arc for a known single step.
// Disable accel/vel clamping by giving generous limits so the commanded (v, w)
// is applied exactly in one step from rest.
TEST(Kinematics, RobotStepWorldArc) {
  amr_core::RobotConfig rcfg;
  rcfg.radius = 0.05;          // tiny footprint, stays free in a big world
  rcfg.max_lin_vel = 10.0;
  rcfg.max_ang_vel = 10.0;
  rcfg.max_lin_acc = 1000.0;   // dv_max huge -> command applied in full
  rcfg.max_ang_acc = 1000.0;

  amr_sim::World world = make_empty_world(20.0, 20.0);
  // Spawn in the open middle, heading +x.
  amr_sim::DiffDriveRobot robot(rcfg, amr_core::Pose2D{10.0, 10.0, 0.0});

  const double v = M_PI / 2.0;  // R = 1
  const double w = M_PI / 2.0;
  const double dt = 1.0;
  robot.step(amr_core::Twist2D{v, w}, dt, world);

  // World-frame closed form (th0 = 0): R=1, nth_raw = pi/2.
  // nx = x0 + R(sin(nth_raw) - sin(0)) = 10 + 1*(1 - 0) = 11
  // ny = y0 - R(cos(nth_raw) - cos(0)) = 10 - 1*(0 - 1) = 11
  EXPECT_FALSE(robot.collided());
  EXPECT_NEAR(robot.pose().x, 11.0, 1e-9);
  EXPECT_NEAR(robot.pose().y, 11.0, 1e-9);
  EXPECT_NEAR(robot.pose().theta, M_PI / 2.0, 1e-9);
  EXPECT_NEAR(robot.vel().v, v, 1e-9);
  EXPECT_NEAR(robot.vel().omega, w, 1e-9);
}

// --- 2. Lidar range against a single known wall ---------------------------

// A vertical wall at x in [5.0, 5.1). Robot at (1.0, 1.0) heading +x: the
// forward beam (angle 0) should report a range close to 5.0 - 1.0 = 4.0 m.
TEST(Lidar, RangeToKnownWall) {
  const double res = 0.05;
  // Large world so the world boundary is far from the robot; the only obstacle
  // is a single vertical wall in front.
  amr_sim::World base = make_empty_world(20.0, 20.0, res);
  amr_core::OccupancyGrid grid = base.grid();
  // Vertical wall: the occupied column whose cell covers world x in [9.0, 9.05).
  const int wall_col = static_cast<int>(std::floor(9.0 / res));  // x>=9.0
  for (int r = 0; r < grid.rows; ++r) {
    grid.at(r, wall_col) = 100;
  }
  amr_sim::World world(grid, amr_core::Pose2D{0.0, 0.0, 0.0}, {20.0, 20.0});

  amr_core::LidarConfig lcfg;
  lcfg.num_beams = 240;
  lcfg.angle_min = -M_PI;
  lcfg.angle_max = M_PI;
  lcfg.range_min = 0.12;
  lcfg.range_max = 5.0;   // shorter than the distance to any world boundary
  lcfg.noise_std = 0.0;   // deterministic for the assertion

  std::mt19937 rng(42);
  amr_sim::Lidar lidar(lcfg, rng);
  amr_core::Pose2D pose{6.0, 10.0, 0.0};  // 3.0 m in front of the wall, +x
  amr_core::LaserScan scan = lidar.scan(world, pose, 0.0);

  const double inc = (lcfg.angle_max - lcfg.angle_min) / lcfg.num_beams;

  // Forward beam (angle 0): the first occupied sample lands at ~9.0 - 6.0 = 3.0
  // m. Allow one sample step (res*0.5 = 0.025 m) of slack.
  const int fwd = static_cast<int>(std::lround(-lcfg.angle_min / inc));
  ASSERT_GE(fwd, 0);
  ASSERT_LT(fwd, lcfg.num_beams);
  EXPECT_NEAR(scan.ranges[fwd], 3.0, 0.05);

  // Backward beam (angle pi): clear space for 5.0 m (boundary is ~6 m away),
  // so no return -> range_max.
  const int back = static_cast<int>(std::lround((M_PI - lcfg.angle_min) / inc)) %
                   lcfg.num_beams;
  EXPECT_NEAR(scan.ranges[back], lcfg.range_max, 1e-9);
}

// --- 3. Collision flag when driven into a wall ----------------------------

TEST(Collision, DriveIntoWall) {
  const double res = 0.05;
  amr_sim::World base = make_empty_world(8.0, 4.0, res);
  // Occupied vertical wall band near x = 2.0.
  amr_core::OccupancyGrid grid = base.grid();
  for (int r = 0; r < grid.rows; ++r) {
    for (int c = 0; c < grid.cols; ++c) {
      double cx = (c + 0.5) * res;
      if (cx >= 2.0 && cx < 2.15) {
        grid.at(r, c) = 100;
      }
    }
  }
  amr_sim::World world(grid, amr_core::Pose2D{0.0, 0.0, 0.0}, {8.0, 4.0});

  amr_core::RobotConfig rcfg;
  rcfg.radius = 0.18;
  rcfg.max_lin_vel = 1.0;
  rcfg.max_ang_vel = 1.0;
  rcfg.max_lin_acc = 1000.0;
  rcfg.max_ang_acc = 1000.0;

  // Spawn just left of the wall, heading +x straight at it.
  amr_sim::DiffDriveRobot robot(rcfg, amr_core::Pose2D{1.0, 1.0, 0.0});

  bool collided = false;
  const double dt = 0.1;
  for (int i = 0; i < 200; ++i) {
    robot.step(amr_core::Twist2D{1.0, 0.0}, dt, world);
    if (robot.collided()) {
      collided = true;
      break;
    }
  }
  EXPECT_TRUE(collided);
  // On collision the robot is held short of the wall (center never enters it).
  EXPECT_LT(robot.pose().x, 2.0);
  EXPECT_DOUBLE_EQ(robot.vel().v, 0.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
