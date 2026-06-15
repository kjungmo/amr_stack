// SPDX-License-Identifier: Apache-2.0
#include "amr_localization/sensor_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "amr_core/geometry.hpp"

namespace amr_localization {

LikelihoodField::LikelihoodField(const amr_core::OccupancyGrid& grid,
                                 const amr_core::LikelihoodConfig& cfg)
    : grid_(grid), cfg_(cfg) {
  const std::size_t n =
      static_cast<std::size_t>(grid_.rows) * static_cast<std::size_t>(grid_.cols);
  // occupied iff data >= 65 (matches Python `grid.data >= 65`).
  std::vector<uint8_t> occupied(n);
  unknown_.assign(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    occupied[i] = (grid_.data[i] >= 65) ? 1u : 0u;
    // Cells unknown in the source map (data < 0) contribute no hit likelihood.
    unknown_[i] = (grid_.data[i] < 0) ? 1 : 0;
  }
  field_ = amr_core::distance_field(occupied, grid_.rows, grid_.cols,
                                    grid_.resolution, cfg_.max_dist);
}

std::vector<double> LikelihoodField::weigh(
    const std::vector<amr_core::Pose2D>& particles,
    const amr_core::LaserScan& scan) const {
  const std::size_t n = particles.size();
  const double res = grid_.resolution;
  const double max_dist = cfg_.max_dist;
  const double sigma = cfg_.sigma_hit;

  const std::vector<char> valid = scan.valid_mask();
  const int sub = std::max(cfg_.beam_subsample, 1);
  const std::vector<double> angles = scan.angles();
  const std::vector<double>& ranges = scan.ranges;

  // Subsampled valid beams: take valid beams, then stride by `sub` over that
  // filtered sequence (matches Python angles[valid][::sub]).
  std::vector<double> a;
  std::vector<double> r;
  std::size_t valid_count = 0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    if (!valid[i]) {
      continue;
    }
    if (valid_count % static_cast<std::size_t>(sub) == 0) {
      a.push_back(angles[i]);
      r.push_back(ranges[i]);
    }
    ++valid_count;
  }

  if (a.empty()) {
    return std::vector<double>(n, 1.0);
  }

  const std::size_t b = a.size();
  // Beam endpoints in the robot frame.
  std::vector<double> bx(b);
  std::vector<double> by(b);
  for (std::size_t j = 0; j < b; ++j) {
    bx[j] = r[j] * std::cos(a[j]);
    by[j] = r[j] * std::sin(a[j]);
  }

  const double two_sigma2 = 2.0 * sigma * sigma;
  const double z_rand_term = cfg_.z_rand / scan.range_max;

  std::vector<double> w(n);
  for (std::size_t p = 0; p < n; ++p) {
    const double px = particles[p].x;
    const double py = particles[p].y;
    const double cos_t = std::cos(particles[p].theta);
    const double sin_t = std::sin(particles[p].theta);

    double log_sum = 0.0;
    for (std::size_t j = 0; j < b; ++j) {
      const double wx = px + cos_t * bx[j] - sin_t * by[j];
      const double wy = py + sin_t * bx[j] + cos_t * by[j];

      const long col = static_cast<long>(
          std::floor((wx - grid_.origin_x) / res));
      const long row = static_cast<long>(
          std::floor((wy - grid_.origin_y) / res));

      const bool in_bounds = (row >= 0 && row < grid_.rows && col >= 0 &&
                              col < grid_.cols);
      long rc = row;
      long cc = col;
      if (rc < 0) rc = 0;
      if (rc > grid_.rows - 1) rc = grid_.rows - 1;
      if (cc < 0) cc = 0;
      if (cc > grid_.cols - 1) cc = grid_.cols - 1;

      const std::size_t flat =
          static_cast<std::size_t>(rc) * grid_.cols + static_cast<std::size_t>(cc);
      double d;
      if (in_bounds && !unknown_[flat]) {
        d = static_cast<double>(field_[flat]);
      } else {
        // Out-of-bounds or unknown cells contribute the max distance.
        d = max_dist;
      }

      double q = cfg_.z_hit * std::exp(-(d * d) / two_sigma2) + z_rand_term;
      if (q < 1e-300) {
        q = 1e-300;
      }
      log_sum += std::log(q);
    }
    w[p] = std::exp(log_sum);
  }
  return w;
}

}  // namespace amr_localization
