// SPDX-License-Identifier: Apache-2.0
#include "amr_localization/mcl.hpp"

#include <algorithm>
#include <cmath>

#include "amr_localization/motion_model.hpp"

namespace amr_localization {

namespace {

// Low-variance (systematic) resampler. Mirrors the Python
// _low_variance_resample: stratified positions, searchsorted over the
// cumulative weights, index clamped to n-1.
std::vector<amr_core::Pose2D> low_variance_resample(
    const std::vector<amr_core::Pose2D>& particles,
    const std::vector<double>& weights, std::mt19937& rng) {
  const std::size_t n = weights.size();
  std::vector<double> cumsum(n);
  double acc = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    acc += weights[i];
    cumsum[i] = acc;
  }

  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const double r0 = unit(rng);

  std::vector<amr_core::Pose2D> out(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double position =
        (r0 + static_cast<double>(i)) / static_cast<double>(n);
    // np.searchsorted(cumsum, position): first index with cumsum[idx] >= pos.
    auto it = std::lower_bound(cumsum.begin(), cumsum.end(), position);
    std::size_t idx = static_cast<std::size_t>(it - cumsum.begin());
    if (idx > n - 1) {
      idx = n - 1;
    }
    out[i] = particles[idx];
  }
  return out;
}

}  // namespace

MonteCarloLocalizer::MonteCarloLocalizer(
    const amr_core::OccupancyGrid& grid,
    const amr_core::LocalizationConfig& cfg, std::mt19937& rng,
    std::optional<amr_core::Pose2D> initial_pose)
    : grid_(grid), cfg_(cfg), rng_(rng), n_(cfg.num_particles) {
  field_ = std::make_unique<LikelihoodField>(grid_, cfg_.likelihood);

  if (initial_pose.has_value()) {
    particles_ = gaussian_init(*initial_pose);
  } else {
    particles_ = global_init();
  }
  weights_.assign(n_, 1.0 / static_cast<double>(n_));
}

std::vector<amr_core::Pose2D> MonteCarloLocalizer::global_init() {
  // Free cells are data == 0; theta uniform over (-pi, pi).
  std::vector<std::pair<int, int>> free_cells;  // (row, col)
  for (int row = 0; row < grid_.rows; ++row) {
    for (int col = 0; col < grid_.cols; ++col) {
      if (grid_.at(row, col) == 0) {
        free_cells.emplace_back(row, col);
      }
    }
  }
  if (free_cells.empty()) {
    // Degenerate map: fall back to map centre.
    free_cells.emplace_back(grid_.rows / 2, grid_.cols / 2);
  }

  const double res = grid_.resolution;
  std::uniform_int_distribution<std::size_t> pick(0, free_cells.size() - 1);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> ang(-M_PI, M_PI);

  std::vector<amr_core::Pose2D> out(n_);
  for (int i = 0; i < n_; ++i) {
    const auto& cell = free_cells[pick(rng_)];
    const int row = cell.first;
    const int col = cell.second;
    out[i].x = grid_.origin_x + (static_cast<double>(col) + unit(rng_)) * res;
    out[i].y = grid_.origin_y + (static_cast<double>(row) + unit(rng_)) * res;
    out[i].theta = ang(rng_);
  }
  return out;
}

std::vector<amr_core::Pose2D> MonteCarloLocalizer::gaussian_init(
    const amr_core::Pose2D& pose) {
  const double sx = cfg_.init_std[0];
  const double sy = cfg_.init_std[1];
  const double sth = cfg_.init_std[2];
  std::normal_distribution<double> nx(0.0, sx);
  std::normal_distribution<double> ny(0.0, sy);
  std::normal_distribution<double> nth(0.0, sth);

  std::vector<amr_core::Pose2D> out(n_);
  for (int i = 0; i < n_; ++i) {
    out[i].x = pose.x + nx(rng_);
    out[i].y = pose.y + ny(rng_);
    out[i].theta = pose.theta + nth(rng_);
  }
  return out;
}

void MonteCarloLocalizer::predict(const amr_core::Pose2D& odom_delta) {
  particles_ = sample_motion(particles_, odom_delta, cfg_.alphas, rng_);
}

void MonteCarloLocalizer::correct(const amr_core::LaserScan& scan) {
  const std::vector<double> lik = field_->weigh(particles_, scan);

  std::vector<double> w(n_);
  double total = 0.0;
  for (int i = 0; i < n_; ++i) {
    w[i] = weights_[i] * lik[i];
    total += w[i];
  }

  if (!std::isfinite(total) || total <= 0.0) {
    // Underflow / all-zero guard: reset to a uniform cloud.
    const double u = 1.0 / static_cast<double>(n_);
    for (int i = 0; i < n_; ++i) {
      w[i] = u;
    }
  } else {
    for (int i = 0; i < n_; ++i) {
      w[i] /= total;
    }
  }
  weights_ = w;

  double sum_sq = 0.0;
  for (int i = 0; i < n_; ++i) {
    sum_sq += weights_[i] * weights_[i];
  }
  const double neff = 1.0 / sum_sq;
  if (neff < cfg_.resample_neff_frac * static_cast<double>(n_)) {
    particles_ = low_variance_resample(particles_, weights_, rng_);
    const double u = 1.0 / static_cast<double>(n_);
    for (int i = 0; i < n_; ++i) {
      weights_[i] = u;
    }
  }
}

amr_core::Pose2D MonteCarloLocalizer::estimate() const {
  double x = 0.0;
  double y = 0.0;
  double s = 0.0;
  double c = 0.0;
  for (int i = 0; i < n_; ++i) {
    const double w = weights_[i];
    x += w * particles_[i].x;
    y += w * particles_[i].y;
    s += w * std::sin(particles_[i].theta);
    c += w * std::cos(particles_[i].theta);
  }
  const double theta = std::atan2(s, c);
  return amr_core::Pose2D{x, y, theta};
}

void MonteCarloLocalizer::set_pose(const amr_core::Pose2D& pose) {
  particles_ = gaussian_init(pose);
  const double u = 1.0 / static_cast<double>(n_);
  weights_.assign(n_, u);
}

}  // namespace amr_localization
