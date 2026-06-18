// SPDX-License-Identifier: Apache-2.0
// Reference core: the worked example for CORE_INTEGRATION. A minimal host
// orchestrator (CoreBus) drives the standardized module interfaces end to end.
// This file doubles as the integration guide's canonical example.
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "amr_api/behavior.hpp"
#include "amr_api/mapper.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_mapping/mapper_adapter.hpp"
#include "amr_navigation/navigator_behavior.hpp"
#include "amr_planning/costmap_adapter.hpp"
#include "amr_reference_core/core_bus.hpp"

namespace {
// A 4x4 m open arena at 0.05 m resolution, all free.
amr_api::OccupancyGrid open_arena() {
  amr_api::OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = 80;
  g.cols = 80;
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);
  return g;
}
}  // namespace

// Core owns the bus + clock + logger, builds a CostmapView via the planning
// backend, and drives the navigation behavior to a goal, advancing the pose by
// integrating the commanded twist (a stand-in for the simulator / real robot).
TEST(ReferenceCore, NavMissionReachesGoal) {
  amr_reference_core::CoreBus bus;
  bus.map = open_arena();
  bus.pose = amr_core::Pose2D{0.5, 0.5, 0.0};
  bus.vel = amr_core::Twist2D{0.0, 0.0};

  amr_core::RobotConfig robot;
  amr_core::CostmapConfig costmap_cfg;
  auto costmap = amr_planning::make_costmap(bus.map, costmap_cfg, robot.radius);

  // The "Planning + Navigation" module, behind amr_api::IBehavior.
  std::unique_ptr<amr_api::IBehavior> behavior =
      std::make_unique<amr_navigation::NavigatorBehavior>(
          amr_core::NavConfig{}, amr_core::AstarConfig{}, amr_core::DwaConfig{},
          robot);

  const amr_core::Pose2D goal{3.0, 3.0, 0.0};
  behavior->set_goal(goal);
  bus.logger.log(amr_api::Logger::Level::Info, "goal set (3.0, 3.0)");

  const double dt = 0.1;
  amr_api::BehaviorState state = amr_api::BehaviorState::IDLE;
  int ticks = 0;
  for (; ticks < 600; ++ticks) {
    amr_api::BehaviorInput in;
    in.pose = bus.pose;
    in.vel = bus.vel;
    in.costmap = costmap.get();
    in.now = bus.clock.now();

    const amr_api::BehaviorOutput out = behavior->update(in);
    state = out.state;
    bus.vel = out.cmd;
    if (out.plan.has_value()) {
      bus.plan = *out.plan;
    }

    // Advance the pose by integrating the command (unicycle); stands in for the
    // simulator / real robot closing the loop.
    bus.pose.x += out.cmd.v * std::cos(bus.pose.theta) * dt;
    bus.pose.y += out.cmd.v * std::sin(bus.pose.theta) * dt;
    bus.pose.theta += out.cmd.omega * dt;
    bus.clock.advance(dt);

    if (state == amr_api::BehaviorState::SUCCEEDED ||
        state == amr_api::BehaviorState::FAILED) {
      break;
    }
  }

  EXPECT_EQ(state, amr_api::BehaviorState::SUCCEEDED);
  const double err = std::hypot(bus.pose.x - goal.x, bus.pose.y - goal.y);
  EXPECT_LT(err, amr_core::NavConfig{}.goal_tol_xy + 0.05);
  EXPECT_GT(ticks, 0);
}

// Demonstrates the IMapper interface: integrate a scan and read back the grid.
TEST(ReferenceCore, MapperIntegratesScan) {
  amr_core::MappingConfig cfg;
  amr_mapping::OccupancyGridMapperAdapter mapper(cfg, {4.0, 4.0}, {0.0, 0.0});

  amr_core::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / 60.0;
  scan.range_min = 0.12;
  scan.range_max = 8.0;
  scan.ranges.assign(60, 1.0);

  amr_api::IMapper& iface = mapper;
  iface.integrate(amr_api::MapperInput{amr_core::Pose2D{2.0, 2.0, 0.0}, scan});
  const amr_api::OccupancyGrid g = iface.map();

  int occupied = 0;
  for (const auto v : g.data) {
    if (v == 100) ++occupied;
  }
  EXPECT_GT(occupied, 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
