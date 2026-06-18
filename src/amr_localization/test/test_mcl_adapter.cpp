// SPDX-License-Identifier: Apache-2.0
// Parity test: MclAdapter (own seeded RNG) == raw MonteCarloLocalizer seeded
// identically, over the same predict/correct sequence. Both engines start at the
// same seed and draw in the same order, so estimates are bit-identical.
#include <random>

#include <gtest/gtest.h>

#include "amr_api/localizer.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_localization/mcl.hpp"
#include "amr_localization/mcl_adapter.hpp"

namespace {
amr_core::OccupancyGrid free_grid() {
  amr_core::OccupancyGrid g;
  g.resolution = 0.05;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.rows = 80;
  g.cols = 80;
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);
  return g;
}
amr_core::LaserScan ring_scan() {
  amr_core::LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 40.0;
  s.range_min = 0.12;
  s.range_max = 8.0;
  s.ranges.assign(40, 2.0);
  return s;
}
}  // namespace

TEST(MclAdapter, MatchesRaw) {
  const auto grid = free_grid();
  amr_core::LocalizationConfig cfg;
  const unsigned int seed = 7;
  const amr_core::Pose2D init{2.0, 2.0, 0.0};

  std::mt19937 rng(seed);
  amr_localization::MonteCarloLocalizer raw(grid, cfg, rng, init);
  amr_localization::MclAdapter adp(grid, cfg, seed, init);

  const amr_core::Pose2D delta{0.05, 0.0, 0.01};
  const auto scan = ring_scan();
  for (int i = 0; i < 3; ++i) {
    raw.predict(delta);
    raw.correct(scan);
    const amr_core::Pose2D er = raw.estimate();

    const auto out = adp.update(amr_api::LocalizerInput{delta, scan});

    EXPECT_DOUBLE_EQ(er.x, out.pose.x);
    EXPECT_DOUBLE_EQ(er.y, out.pose.y);
    EXPECT_DOUBLE_EQ(er.theta, out.pose.theta);
  }
  EXPECT_EQ(raw.particles().size(), adp.raw().particles().size());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
