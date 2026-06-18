// SPDX-License-Identifier: Apache-2.0
// Read-only costmap abstraction. Lets amr_api expose a costmap to planners and
// behaviors without depending on amr_planning (where the concrete Costmap and
// its inflation live). The reference backend implements this in amr_planning
// (CostmapAdapter + make_costmap).
#pragma once

namespace amr_api {

class CostmapView {
 public:
  static constexpr float LETHAL = 1.0f;

  virtual ~CostmapView() = default;

  virtual int rows() const = 0;
  virtual int cols() const = 0;
  virtual double resolution() const = 0;

  virtual void world_to_grid(double x, double y, int& row, int& col) const = 0;
  virtual void grid_to_world(int row, int col, double& x, double& y) const = 0;
  virtual bool in_bounds(int row, int col) const = 0;

  virtual float cost_at(int row, int col) const = 0;        // in [0, 1]
  virtual double cost_at_world(double x, double y) const = 0;
  virtual bool is_lethal(int row, int col) const = 0;
};

}  // namespace amr_api
