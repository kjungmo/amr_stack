// SPDX-License-Identifier: Apache-2.0
// gtests for amr_mapping::OccupancyGridMapper. Mirrors tests/test_mapper.py.
#include "amr_mapping/occupancy_grid_mapper.hpp"

#include <cmath>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "gtest/gtest.h"

namespace {

// LaserScan with n beams, all returning `dist`. Matches _scan_hit_at in the
// Python test: angle_min=-pi, angle_increment=2*pi/n, range_min=0.1, max=8.0.
amr_core::LaserScan scan_hit_at(double dist, int n = 8) {
  amr_core::LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / static_cast<double>(n);
  s.range_min = 0.1;
  s.range_max = 8.0;
  s.ranges.assign(static_cast<std::size_t>(n), dist);
  return s;
}

int8_t cell_at(const amr_core::OccupancyGrid& g, double x, double y) {
  int row = 0;
  int col = 0;
  g.world_to_grid(x, y, row, col);
  return g.at(row, col);
}

}  // namespace

// Mirrors test_single_scan_free_occ_unknown: a wall cell is marked occupied,
// the space before it free, and the space behind it unknown.
TEST(Mapper, SingleScanFreeOccUnknown) {
  amr_core::MappingConfig cfg;  // struct defaults
  cfg.beam_subsample = 1;       // every beam, matching the Python test

  amr_mapping::OccupancyGridMapper m(cfg, {6.0, 6.0});

  amr_core::Pose2D pose{3.0, 3.0, 0.0};
  for (int i = 0; i < 4; ++i) {  // strengthen evidence
    m.update(pose, scan_hit_at(2.0));
  }
  const amr_core::OccupancyGrid g = m.to_occupancy_grid();

  EXPECT_EQ(cell_at(g, 4.0, 3.0), 0);    // on the +x beam, before the hit
  EXPECT_EQ(cell_at(g, 5.0, 3.0), 100);  // the hit cell
  EXPECT_EQ(cell_at(g, 5.5, 3.0), -1);   // behind the hit (unobserved)
  EXPECT_EQ(cell_at(g, 3.0, 5.0), 100);  // +y beam hit
}
