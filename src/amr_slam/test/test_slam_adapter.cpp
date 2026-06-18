// SPDX-License-Identifier: Apache-2.0
// Parity test: ScanMatchingSlamAdapter == raw ScanMatchingSlam on identical
// input (pose at each tick + the final map).
#include <utility>

#include <gtest/gtest.h>

#include "amr_api/slam.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_slam/scan_matching_slam.hpp"
#include "amr_slam/slam_adapter.hpp"

namespace {
amr_core::LaserScan ring_scan() {
  amr_core::LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 60.0;
  s.range_min = 0.12;
  s.range_max = 8.0;
  s.ranges.assign(60, 3.0);
  return s;
}
}  // namespace

TEST(SlamAdapter, MatchesRaw) {
  amr_core::SlamConfig scfg;
  amr_core::MappingConfig mcfg;
  const std::pair<double, double> size{10.0, 10.0};
  const amr_core::Pose2D init{5.0, 5.0, 0.0};
  const auto scan = ring_scan();

  amr_slam::ScanMatchingSlam raw(scfg, mcfg, size, init);
  amr_slam::ScanMatchingSlamAdapter adp(scfg, mcfg, size, init);

  const amr_core::Pose2D delta{0.05, 0.0, 0.0};
  for (int i = 0; i < 3; ++i) {
    const amr_core::Pose2D pr = raw.process(delta, scan);
    const amr_core::Pose2D pa = adp.update(amr_api::SlamInput{delta, scan});
    EXPECT_DOUBLE_EQ(pr.x, pa.x);
    EXPECT_DOUBLE_EQ(pr.y, pa.y);
    EXPECT_DOUBLE_EQ(pr.theta, pa.theta);
  }
  const auto gr = raw.get_map();
  const auto ga = adp.map();
  ASSERT_EQ(gr.data.size(), ga.data.size());
  for (std::size_t i = 0; i < gr.data.size(); ++i) {
    EXPECT_EQ(gr.data[i], ga.data[i]);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
