// SPDX-License-Identifier: Apache-2.0
#include "amr_sim/lidar.hpp"

#include <cmath>
#include <vector>

namespace amr_sim {

amr_core::LaserScan Lidar::scan(const World& world, const amr_core::Pose2D& pose,
                                double stamp) {
  const amr_core::LidarConfig& cfg = cfg_;
  const amr_core::OccupancyGrid& grid = world.grid();
  const double res = grid.resolution;
  const int B = cfg.num_beams;
  const double increment = (cfg.angle_max - cfg.angle_min) / static_cast<double>(B);

  // Range samples along each beam: arange(range_min, range_max, res*0.5).
  const double sstep = res * 0.5;
  std::vector<double> S;
  for (double s = cfg.range_min; s < cfg.range_max; s += sstep) {
    S.push_back(s);
  }
  if (S.empty()) {
    S.push_back(cfg.range_min);
  }
  const int Snum = static_cast<int>(S.size());

  std::vector<double> ranges(static_cast<std::size_t>(B));

  for (int b = 0; b < B; ++b) {
    const double beam_angle = cfg.angle_min + increment * static_cast<double>(b);
    const double world_angle = pose.theta + beam_angle;
    const double cosw = std::cos(world_angle);
    const double sinw = std::sin(world_angle);

    double range = cfg.range_max;  // no-return default
    for (int k = 0; k < Snum; ++k) {
      const double sx = pose.x + cosw * S[k];
      const double sy = pose.y + sinw * S[k];
      const long row = static_cast<long>(std::floor((sy - grid.origin_y) / res));
      const long col = static_cast<long>(std::floor((sx - grid.origin_x) / res));
      const bool in_bounds = (row >= 0 && row < grid.rows &&
                              col >= 0 && col < grid.cols);
      bool hit;
      if (!in_bounds) {
        hit = true;  // out-of-bounds counts as a hit
      } else {
        hit = grid.at(static_cast<int>(row), static_cast<int>(col)) >= 50;
      }
      if (hit) {
        range = S[k];  // first occupied sample is the range
        break;
      }
    }
    ranges[static_cast<std::size_t>(b)] = range;
  }

  // Seeded Gaussian noise: one draw per beam in beam order.
  if (cfg.noise_std > 0.0) {
    std::normal_distribution<double> dist(0.0, cfg.noise_std);
    for (int b = 0; b < B; ++b) {
      ranges[static_cast<std::size_t>(b)] += dist(rng_);
    }
  }

  // Clip to [range_min, range_max].
  for (int b = 0; b < B; ++b) {
    double& r = ranges[static_cast<std::size_t>(b)];
    if (r < cfg.range_min) r = cfg.range_min;
    if (r > cfg.range_max) r = cfg.range_max;
  }

  amr_core::LaserScan ls;
  ls.angle_min = cfg.angle_min;
  ls.angle_increment = increment;
  ls.range_min = cfg.range_min;
  ls.range_max = cfg.range_max;
  ls.ranges = std::move(ranges);
  ls.stamp = stamp;
  return ls;
}

}  // namespace amr_sim
