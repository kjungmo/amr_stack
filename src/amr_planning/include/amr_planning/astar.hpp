// SPDX-License-Identifier: Apache-2.0
// A* global planner over an inflated costmap, plus a line-of-sight helper.
// Ports amr/planning/astar.py.
//
// 8-connected search on grid cells. Lethal cells are obstacles; the step cost is
// scaled up by the (non-lethal) cell cost so the path is pushed away from
// obstacles. The heuristic is the octile distance, which is admissible because
// the step-cost multiplier ``1 + w_cost * cost`` is always >= 1.
#pragma once

#include <array>
#include <optional>
#include <utility>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_planning/costmap.hpp"

namespace amr_planning {

/// True iff every sample at resolution/2 spacing along p->q is non-lethal.
bool has_line_of_sight(const Costmap& costmap, double px, double py, double qx,
                       double qy);

/// Plan an 8-connected A* path; return world waypoints start->goal as a vector
/// of (x, y) pairs, or std::nullopt if unreachable.
std::optional<std::vector<std::array<double, 2>>> plan_path(
    const Costmap& costmap, const std::array<double, 2>& start_xy,
    const std::array<double, 2>& goal_xy, const amr_core::AstarConfig& cfg);

}  // namespace amr_planning
