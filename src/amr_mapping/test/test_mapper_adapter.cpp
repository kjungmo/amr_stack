// SPDX-License-Identifier: Apache-2.0
// Parity test: OccupancyGridMapperAdapter must produce the same grid as the raw
// OccupancyGridMapper for the same scan at the same pose.
#include <utility>

#include <gtest/gtest.h>

#include "amr_api/mapper.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_mapping/mapper_adapter.hpp"
#include "amr_mapping/occupancy_grid_mapper.hpp"

namespace {
amr_core::LaserScan ring_scan() {
  amr_core::LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 60.0;
  s.range_min = 0.12;
  s.range_max = 8.0;
  s.ranges.assign(60, 1.0);
  return s;
}
}  // namespace

TEST(MapperAdapter, MatchesRaw) {
  amr_core::MappingConfig cfg;
  const std::pair<double, double> size{4.0, 4.0};
  const std::pair<double, double> origin{0.0, 0.0};
  const amr_core::Pose2D pose{2.0, 2.0, 0.0};
  const auto scan = ring_scan();

  amr_mapping::OccupancyGridMapper raw(cfg, size, origin);
  raw.update(pose, scan);
  const auto g_raw = raw.to_occupancy_grid();

  amr_mapping::OccupancyGridMapperAdapter adp(cfg, size, origin);
  adp.integrate(amr_api::MapperInput{pose, scan});
  const auto g_adp = adp.map();

  ASSERT_EQ(g_raw.rows, g_adp.rows);
  ASSERT_EQ(g_raw.cols, g_adp.cols);
  ASSERT_EQ(g_raw.data.size(), g_adp.data.size());
  for (std::size_t i = 0; i < g_raw.data.size(); ++i) {
    EXPECT_EQ(g_raw.data[i], g_adp.data[i]);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
