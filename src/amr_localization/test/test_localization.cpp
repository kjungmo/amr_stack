// SPDX-License-Identifier: Apache-2.0
// gtests for the amr_localization library: motion model, likelihood-field
// sensor model, and MCL convergence on a small known map. Mirrors the Python
// unit tests in amr/localization. The convergence test is marked `slow`.
#include <cmath>
#include <random>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/geometry.hpp"
#include "amr_core/types.hpp"
#include "amr_localization/mcl.hpp"
#include "amr_localization/motion_model.hpp"
#include "amr_localization/sensor_model.hpp"
#include "gtest/gtest.h"

namespace {

using amr_core::LaserScan;
using amr_core::OccupancyGrid;
using amr_core::Pose2D;

// Build a rectangular room: a free interior bordered by an occupied wall ring.
// width/height in metres, res in m/cell, origin at (0,0).
OccupancyGrid make_room(double width, double height, double res) {
  OccupancyGrid g;
  g.resolution = res;
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cols = static_cast<int>(std::round(width / res));
  g.rows = static_cast<int>(std::round(height / res));
  g.data.assign(static_cast<std::size_t>(g.rows) * g.cols, 0);  // all free
  for (int r = 0; r < g.rows; ++r) {
    for (int c = 0; c < g.cols; ++c) {
      const bool border =
          (r == 0 || r == g.rows - 1 || c == 0 || c == g.cols - 1);
      if (border) {
        g.at(r, c) = 100;  // occupied
      }
    }
  }
  return g;
}

// Synthesize a laser scan from a true pose by ray-casting against occupied
// cells in the grid (first-hit). Beams that hit nothing return range_max.
LaserScan synth_scan(const OccupancyGrid& g, const Pose2D& pose, int num_beams,
                     double range_max) {
  LaserScan s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / static_cast<double>(num_beams);
  s.range_min = 0.12;
  s.range_max = range_max;
  s.stamp = 0.0;
  s.ranges.resize(num_beams);

  const double step = g.resolution * 0.25;
  for (int i = 0; i < num_beams; ++i) {
    const double ang = s.angle_min + s.angle_increment * i;
    const double world_ang = pose.theta + ang;
    const double cx = std::cos(world_ang);
    const double cy = std::sin(world_ang);
    double rng = range_max;
    for (double d = s.range_min; d <= range_max; d += step) {
      const double wx = pose.x + cx * d;
      const double wy = pose.y + cy * d;
      int row, col;
      g.world_to_grid(wx, wy, row, col);
      if (!g.in_bounds(row, col)) {
        rng = range_max;
        break;
      }
      if (g.at(row, col) >= 65) {
        rng = d;
        break;
      }
    }
    s.ranges[i] = rng;
  }
  return s;
}

}  // namespace

// --- Motion model ----------------------------------------------------------

TEST(MotionModel, ZeroDeltaZeroNoiseIsIdentity) {
  std::vector<Pose2D> particles = {{1.0, 2.0, 0.5}, {-3.0, 0.0, -1.2}};
  std::mt19937 rng(42);
  // Zero alphas -> zero noise; zero delta -> no motion.
  const auto out = amr_localization::sample_motion(
      particles, Pose2D{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, rng);
  ASSERT_EQ(out.size(), particles.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_NEAR(out[i].x, particles[i].x, 1e-12);
    EXPECT_NEAR(out[i].y, particles[i].y, 1e-12);
    EXPECT_NEAR(out[i].theta, particles[i].theta, 1e-12);
  }
}

TEST(MotionModel, NoiselessForwardMoveAppliesInBodyFrame) {
  // A particle at the origin facing +x; pure forward delta of 1.0 m.
  std::vector<Pose2D> particles = {{0.0, 0.0, 0.0}};
  std::mt19937 rng(7);
  const auto out = amr_localization::sample_motion(
      particles, Pose2D{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, rng);
  EXPECT_NEAR(out[0].x, 1.0, 1e-9);
  EXPECT_NEAR(out[0].y, 0.0, 1e-9);
  EXPECT_NEAR(out[0].theta, 0.0, 1e-9);

  // Same delta but particle faces +y -> moves along world +y.
  std::vector<Pose2D> p2 = {{0.0, 0.0, M_PI / 2.0}};
  const auto out2 = amr_localization::sample_motion(
      p2, Pose2D{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, rng);
  EXPECT_NEAR(out2[0].x, 0.0, 1e-9);
  EXPECT_NEAR(out2[0].y, 1.0, 1e-9);
}

// --- Sensor model ----------------------------------------------------------

TEST(SensorModel, TruePoseScoresHigherThanWrongPose) {
  const OccupancyGrid g = make_room(4.0, 3.0, 0.05);
  amr_core::LikelihoodConfig lcfg;  // struct defaults
  amr_localization::LikelihoodField field(g, lcfg);

  const Pose2D truth{2.0, 1.5, 0.3};
  const LaserScan scan = synth_scan(g, truth, 240, 8.0);

  std::vector<Pose2D> candidates = {truth, {2.6, 1.5, 0.3}, {2.0, 0.9, -0.6}};
  const std::vector<double> w = field.weigh(candidates, scan);
  ASSERT_EQ(w.size(), candidates.size());
  EXPECT_GT(w[0], w[1]);
  EXPECT_GT(w[0], w[2]);
}

// --- MCL convergence (slow) ------------------------------------------------

TEST(MCLConvergence, ConvergesToKnownPose_slow) {
  const OccupancyGrid g = make_room(4.0, 3.0, 0.05);
  amr_core::LocalizationConfig cfg;  // 500 particles, struct defaults
  std::mt19937 rng(42);

  const Pose2D truth{2.0, 1.5, 0.4};

  // Seed Gaussian-near the truth (as the NAV pipeline would after an
  // initialpose), then feed several scans from the (stationary) true pose.
  amr_localization::MonteCarloLocalizer mcl(g, cfg, rng, truth);

  const LaserScan scan = synth_scan(g, truth, 240, 8.0);
  for (int iter = 0; iter < 12; ++iter) {
    mcl.predict(Pose2D{0.0, 0.0, 0.0});  // stationary
    mcl.correct(scan);
  }

  const Pose2D est = mcl.estimate();
  const double pos_err = std::hypot(est.x - truth.x, est.y - truth.y);
  const double ang_err = std::abs(amr_core::wrap_angle(est.theta - truth.theta));
  EXPECT_LT(pos_err, 0.15) << "pos_err=" << pos_err;
  EXPECT_LT(ang_err, 0.20) << "ang_err=" << ang_err;
}

TEST(MCLConvergence, GlobalInitRecoversWithMotion_slow) {
  const OccupancyGrid g = make_room(4.0, 3.0, 0.05);
  amr_core::LocalizationConfig cfg;
  std::mt19937 rng(123);

  const Pose2D truth{2.4, 1.2, 0.0};
  // Global (uniform) init; correct repeatedly from the true pose. With a
  // distinctive room the cloud should collapse near the truth.
  amr_localization::MonteCarloLocalizer mcl(g, cfg, rng, std::nullopt);

  const LaserScan scan = synth_scan(g, truth, 240, 8.0);
  for (int iter = 0; iter < 30; ++iter) {
    mcl.predict(Pose2D{0.0, 0.0, 0.0});
    mcl.correct(scan);
  }

  // The weighted-mean estimate must stay on-map.
  const Pose2D est = mcl.estimate();
  EXPECT_GE(est.x, 0.0);
  EXPECT_LE(est.x, 4.0);
  EXPECT_GE(est.y, 0.0);
  EXPECT_LE(est.y, 3.0);

  // The dominant (max-weight) particle should sit near the truth: this proves
  // the cloud collapsed onto the correct mode even if a mirror mode survives
  // and biases the weighted mean.
  const auto& particles = mcl.particles();
  const auto& weights = mcl.weights();
  std::size_t best = 0;
  for (std::size_t i = 1; i < weights.size(); ++i) {
    if (weights[i] > weights[best]) {
      best = i;
    }
  }
  const double best_err =
      std::hypot(particles[best].x - truth.x, particles[best].y - truth.y);
  EXPECT_LT(best_err, 0.5) << "best_err=" << best_err;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
