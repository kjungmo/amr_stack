// SPDX-License-Identifier: Apache-2.0
// CostmapAdapter: wraps an amr_planning::Costmap as an amr_api::CostmapView so
// core can hold and inspect a costmap without depending on amr_planning. The
// reference planners recover the concrete Costmap via as_costmap(); the pairing
// of "reference planner <-> reference costmap" is a documented contract.
#pragma once

#include <memory>

#include "amr_api/costmap_view.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_planning/costmap.hpp"

namespace amr_planning {

class CostmapAdapter : public amr_api::CostmapView {
 public:
  CostmapAdapter(const amr_core::OccupancyGrid& grid,
                 const amr_core::CostmapConfig& cfg, double robot_radius)
      : costmap_(grid, cfg, robot_radius) {}

  int rows() const override { return costmap_.rows(); }
  int cols() const override { return costmap_.cols(); }
  double resolution() const override { return costmap_.resolution(); }
  void world_to_grid(double x, double y, int& row, int& col) const override {
    costmap_.world_to_grid(x, y, row, col);
  }
  void grid_to_world(int row, int col, double& x, double& y) const override {
    costmap_.grid_to_world(row, col, x, y);
  }
  bool in_bounds(int row, int col) const override {
    return costmap_.in_bounds(row, col);
  }
  float cost_at(int row, int col) const override {
    return costmap_.cost_at(row, col);
  }
  double cost_at_world(double x, double y) const override {
    return costmap_.cost_at_world(x, y);
  }
  bool is_lethal(int row, int col) const override {
    return costmap_.is_lethal(row, col);
  }

  /// Concrete costmap, for the reference planners.
  const Costmap& raw() const { return costmap_; }

 private:
  Costmap costmap_;
};

/// Build a CostmapView from an occupancy grid (runs chamfer inflation once).
std::unique_ptr<amr_api::CostmapView> make_costmap(
    const amr_core::OccupancyGrid& grid, const amr_core::CostmapConfig& cfg,
    double robot_radius);

/// Recover the concrete Costmap from a CostmapView produced by make_costmap.
/// Throws std::bad_cast if `view` is not a CostmapAdapter (paired contract).
const Costmap& as_costmap(const amr_api::CostmapView& view);

}  // namespace amr_planning
