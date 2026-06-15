// SPDX-License-Identifier: Apache-2.0
// gtest (slow): the correlative scan matcher recovers a known small
// (dx, dy, dtheta) transform applied to a synthetic scan against a map.
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/geometry.hpp"
#include "amr_core/types.hpp"
#include "amr_slam/scan_matching_slam.hpp"
#include "gtest/gtest.h"

namespace {

// A simple square room: four axis-aligned walls. Synthetic 360-beam lidar
// ray-casts the nearest wall hit so the resulting map has real 2D structure.
struct Room {
  double x_min, x_max, y_min, y_max;
};

// Distance to the first wall along a ray from (px,py) heading `ang` (world).
double ray_cast(const Room& room, double px, double py, double ang,
                double range_max) {
  const double cx = std::cos(ang);
  const double sy = std::sin(ang);
  double best = range_max;
  // Vertical walls x = x_min, x = x_max.
  for (const double wx : {room.x_min, room.x_max}) {
    if (std::abs(cx) > 1e-9) {
      const double t = (wx - px) / cx;
      if (t > 1e-6 && t < best) {
        const double yhit = py + t * sy;
        if (yhit >= room.y_min - 1e-9 && yhit <= room.y_max + 1e-9) best = t;
      }
    }
  }
  // Horizontal walls y = y_min, y = y_max.
  for (const double wy : {room.y_min, room.y_max}) {
    if (std::abs(sy) > 1e-9) {
      const double t = (wy - py) / sy;
      if (t > 1e-6 && t < best) {
        const double xhit = px + t * cx;
        if (xhit >= room.x_min - 1e-9 && xhit <= room.x_max + 1e-9) best = t;
      }
    }
  }
  return best;
}

amr_core::LaserScan synth_scan(const Room& room, const amr_core::Pose2D& pose,
                               int num_beams = 240) {
  amr_core::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / static_cast<double>(num_beams);
  scan.range_min = 0.12;
  scan.range_max = 8.0;
  scan.ranges.resize(static_cast<std::size_t>(num_beams));
  for (int i = 0; i < num_beams; ++i) {
    const double beam = scan.angle_min + scan.angle_increment * i;
    const double world_ang = pose.theta + beam;  // beam 0 = robot forward
    const double r = ray_cast(room, pose.x, pose.y, world_ang, scan.range_max);
    scan.ranges[static_cast<std::size_t>(i)] = r;
  }
  return scan;
}

}  // namespace

TEST(ScanMatchingSlam, RecoversKnownSmallTransform) {
  amr_core::SlamConfig slam_cfg{};   // canonical defaults
  amr_core::MappingConfig map_cfg{};

  // Room comfortably inside a 10x8 m map (origin at world (0,0)).
  const Room room{1.0, 9.0, 1.0, 7.0};
  const amr_core::Pose2D truth{5.0, 4.0, 0.3};

  amr_slam::ScanMatchingSlam slam(slam_cfg, map_cfg, std::make_pair(10.0, 8.0),
                                  truth);

  // 1. Seed the map at the true pose with a dense synthetic scan. Repeat a few
  //    times with small keyframe-triggering nudges so the walls are well
  //    populated in the log-odds / score map.
  const amr_core::LaserScan seed = synth_scan(room, truth);
  slam.process(amr_core::Pose2D{0.0, 0.0, 0.0},
               std::optional<amr_core::LaserScan>(seed));

  // The matcher must already be operating near the true pose. Now inject a
  // known small odometry error: pretend odom reports a delta that overshoots
  // truth by (dx, dy, dtheta). The pose estimate jumps to truth (+) error,
  // but the (true) scan should pull it back toward truth.
  const amr_core::Pose2D err{0.08, -0.06, 0.04};  // within coarse window
  const amr_core::LaserScan obs = synth_scan(room, truth);

  // process() dead-reckons pose by `err`, then scan-matches against the map.
  const amr_core::Pose2D matched =
      slam.process(err, std::optional<amr_core::LaserScan>(obs));

  // Pose without correction would sit at truth (+) err.
  const amr_core::Pose2D predicted = amr_core::pose_compose(truth, err);
  const double pred_err =
      std::hypot(predicted.x - truth.x, predicted.y - truth.y);
  const double matched_err =
      std::hypot(matched.x - truth.x, matched.y - truth.y);

  // The match must pull the estimate meaningfully closer to truth than the
  // uncorrected odometry prediction, and land within ~one coarse step.
  EXPECT_LT(matched_err, pred_err)
      << "matched_err=" << matched_err << " pred_err=" << pred_err;
  EXPECT_LT(matched_err, slam_cfg.coarse_step_xy + 1e-9);

  const double dtheta =
      std::abs(amr_core::wrap_angle(matched.theta - truth.theta));
  EXPECT_LT(dtheta, slam_cfg.coarse_step_theta + 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
