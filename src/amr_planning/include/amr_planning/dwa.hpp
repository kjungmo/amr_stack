// SPDX-License-Identifier: Apache-2.0
// Dynamic Window Approach local planner. Ports amr/planning/dwa.py.
//
// Samples a grid of (v, omega) commands within the velocity/acceleration-limited
// dynamic window, rolls each out with a unicycle model, rejects rollouts that hit
// a lethal costmap cell, and scores the survivors against a carrot point taken a
// fixed arc length ahead on the global path.
#pragma once

#include <array>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/costmap.hpp"

namespace amr_planning {

/// A path is a sequence of world (x, y) waypoints.
using Path = std::vector<std::array<double, 2>>;

struct DwaResult {
  amr_core::Twist2D cmd{};
  std::vector<std::array<double, 3>> trajectory{};  // poses of chosen rollout
  bool blocked{false};  // True if every sampled rollout collides
};

/// Point ``lookahead`` metres of arc length beyond the projection of the pose
/// onto the path polyline; clamps to the final point (the goal).
std::array<double, 2> carrot_point(const Path& path,
                                   const amr_core::Pose2D& pose,
                                   double lookahead);

class DwaPlanner {
 public:
  DwaPlanner(const amr_core::DwaConfig& cfg, const amr_core::RobotConfig& robot)
      : cfg_(cfg), robot_(robot) {}

  DwaResult compute(const amr_core::Pose2D& pose, const amr_core::Twist2D& vel,
                    const Path& path, const Costmap& costmap) const;

 private:
  std::vector<std::array<double, 3>> rollout(const amr_core::Pose2D& pose,
                                             double v, double omega) const;

  amr_core::DwaConfig cfg_;
  amr_core::RobotConfig robot_;
};

}  // namespace amr_planning
