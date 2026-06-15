// SPDX-License-Identifier: Apache-2.0
// Correlative scan-matching SLAM. Faithful C++ port of
// amr/slam/scan_matching_slam.py.
#include "amr_slam/scan_matching_slam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "amr_core/geometry.hpp"

namespace amr_slam {

ScanMatchingSlam::ScanMatchingSlam(const amr_core::SlamConfig& cfg,
                                   const amr_core::MappingConfig& mapping_cfg,
                                   std::pair<double, double> size_m,
                                   const amr_core::Pose2D& initial_pose)
    : cfg_(cfg),
      mapping_cfg_(mapping_cfg),
      resolution_(mapping_cfg.resolution),
      pose_(initial_pose),
      mapper_(mapping_cfg, size_m, {0.0, 0.0}) {
  rows_ = mapper_.rows();
  cols_ = mapper_.cols();
  last_kf_pose_ = initial_pose;
  last_proc_pose_ = initial_pose;
  // Precompute the separable Gaussian blur kernel.
  blur_kernel_ = gaussian_kernel(cfg_.blur_sigma_cells);
}

// ----------------------------------------------------------------------------
// Public interface
// ----------------------------------------------------------------------------

amr_core::Pose2D ScanMatchingSlam::process(
    const amr_core::Pose2D& odom_delta,
    const std::optional<amr_core::LaserScan>& scan) {
  // 1. Dead-reckon the estimate forward.
  pose_ = amr_core::pose_compose(pose_, odom_delta);

  if (!scan.has_value()) {
    return pose_;
  }
  const amr_core::LaserScan& s = *scan;

  // 2. First scan ever: seed the map, remember keyframe state, return.
  if (!have_first_) {
    mapper_.update(pose_, s);
    rebuild_score_map();
    last_kf_pose_ = pose_;
    last_proc_pose_ = pose_;
    have_first_ = true;
    return pose_;
  }

  // 3. Skip matching if displacement since last processed scan is tiny.
  auto [d_trans, d_rot] = motion_since(last_proc_pose_);
  if (d_trans < cfg_.min_motion && d_rot < cfg_.min_motion) {
    return pose_;
  }

  // 4/5/6. Correlative scan match (score map already current).
  const std::vector<double> pts = scan_points(s);
  if (!pts.empty() && have_score_map_) {
    const std::optional<amr_core::Pose2D> matched = match(pts);
    if (matched.has_value()) {
      pose_ = *matched;
    }
  }

  last_proc_pose_ = pose_;

  // 7. Keyframe map update on sufficient motion.
  auto [kf_trans, kf_rot] = motion_since(last_kf_pose_);
  if (kf_trans >= cfg_.keyframe_trans || kf_rot >= cfg_.keyframe_rot) {
    mapper_.update(pose_, s);
    rebuild_score_map();
    last_kf_pose_ = pose_;
  }

  return pose_;
}

// ----------------------------------------------------------------------------
// Score map construction
// ----------------------------------------------------------------------------

void ScanMatchingSlam::rebuild_score_map() {
  // p_occ = sigmoid(log_odds); score_src = (p_occ > 0.6) as float.
  std::vector<double> score_src(static_cast<std::size_t>(rows_) * cols_, 0.0);
  for (int r = 0; r < rows_; ++r) {
    for (int c = 0; c < cols_; ++c) {
      const double lo = static_cast<double>(mapper_.log_odds_at(r, c));
      const double p_occ = 1.0 - 1.0 / (1.0 + std::exp(lo));
      if (p_occ > 0.6) {
        score_src[static_cast<std::size_t>(r) * cols_ + c] = 1.0;
      }
    }
  }

  std::vector<double> blurred = separable_blur(score_src);
  double peak = 0.0;
  for (const double v : blurred) {
    if (v > peak) peak = v;
  }
  if (peak > 0.0) {
    for (double& v : blurred) v /= peak;
  }
  score_map_ = std::move(blurred);
  have_score_map_ = true;
}

std::vector<double> ScanMatchingSlam::gaussian_kernel(double sigma) {
  // 1D Gaussian kernel, half-width 3 sigma, normalized to sum 1.
  sigma = std::max(sigma, 1e-6);
  int half = std::max(static_cast<int>(std::ceil(3.0 * sigma)), 1);
  std::vector<double> k(static_cast<std::size_t>(2 * half + 1));
  double sum = 0.0;
  for (int i = -half; i <= half; ++i) {
    const double off = static_cast<double>(i);
    const double w = std::exp(-(off * off) / (2.0 * sigma * sigma));
    k[static_cast<std::size_t>(i + half)] = w;
    sum += w;
  }
  for (double& w : k) w /= sum;
  return k;
}

std::vector<double> ScanMatchingSlam::separable_blur(
    const std::vector<double>& src) const {
  // Separable convolution along both axes via weighted, zero-padded shifted
  // slices. Boundary = zero padding. Matches the Python implementation.
  const int ksize = static_cast<int>(blur_kernel_.size());
  const int half = (ksize - 1) / 2;
  const int rows = rows_;
  const int cols = cols_;
  const std::size_t n = src.size();

  // Pass 1: blur along columns (axis=1, the x axis).
  std::vector<double> tmp(n, 0.0);
  for (int k = 0; k < ksize; ++k) {
    const double w = blur_kernel_[static_cast<std::size_t>(k)];
    const int shift = k - half;  // negative => take from the right
    if (shift == 0) {
      for (std::size_t i = 0; i < n; ++i) tmp[i] += w * src[i];
    } else if (shift > 0) {
      // tmp[:, shift:] += w * src[:, :-shift]
      for (int r = 0; r < rows; ++r) {
        const std::size_t row = static_cast<std::size_t>(r) * cols;
        for (int c = shift; c < cols; ++c) {
          tmp[row + c] += w * src[row + (c - shift)];
        }
      }
    } else {
      const int sft = -shift;
      // tmp[:, :-sft] += w * src[:, sft:]
      for (int r = 0; r < rows; ++r) {
        const std::size_t row = static_cast<std::size_t>(r) * cols;
        for (int c = 0; c < cols - sft; ++c) {
          tmp[row + c] += w * src[row + (c + sft)];
        }
      }
    }
  }

  // Pass 2: blur along rows (axis=0, the y axis).
  std::vector<double> out(n, 0.0);
  for (int k = 0; k < ksize; ++k) {
    const double w = blur_kernel_[static_cast<std::size_t>(k)];
    const int shift = k - half;
    if (shift == 0) {
      for (std::size_t i = 0; i < n; ++i) out[i] += w * tmp[i];
    } else if (shift > 0) {
      // out[shift:, :] += w * tmp[:-shift, :]
      for (int r = shift; r < rows; ++r) {
        const std::size_t orow = static_cast<std::size_t>(r) * cols;
        const std::size_t irow = static_cast<std::size_t>(r - shift) * cols;
        for (int c = 0; c < cols; ++c) out[orow + c] += w * tmp[irow + c];
      }
    } else {
      const int sft = -shift;
      // out[:-sft, :] += w * tmp[sft:, :]
      for (int r = 0; r < rows - sft; ++r) {
        const std::size_t orow = static_cast<std::size_t>(r) * cols;
        const std::size_t irow = static_cast<std::size_t>(r + sft) * cols;
        for (int c = 0; c < cols; ++c) out[orow + c] += w * tmp[irow + c];
      }
    }
  }

  return out;
}

// ----------------------------------------------------------------------------
// Correlative scan matching
// ----------------------------------------------------------------------------

std::vector<double> ScanMatchingSlam::scan_points(
    const amr_core::LaserScan& scan) const {
  // `match_beams` evenly-subsampled valid beams as robot-frame [x,y] pairs.
  const std::vector<double> angles = scan.angles();
  const std::vector<char> valid = scan.valid_mask();

  std::vector<int> valid_idx;
  valid_idx.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < valid.size(); ++i) {
    if (valid[i] != 0) valid_idx.push_back(static_cast<int>(i));
  }
  if (valid_idx.empty()) {
    return {};
  }

  const int n_valid = static_cast<int>(valid_idx.size());
  const int n_want = cfg_.match_beams;
  if (n_want > 0 && n_valid > n_want) {
    // sel = unique(round(linspace(0, n_valid-1, n_want)))
    std::vector<int> sel;
    sel.reserve(static_cast<std::size_t>(n_want));
    int prev = std::numeric_limits<int>::min();
    for (int j = 0; j < n_want; ++j) {
      // np.linspace endpoints inclusive.
      const double t = static_cast<double>(j) *
                       (static_cast<double>(n_valid - 1) /
                        static_cast<double>(n_want - 1));
      const int idx = static_cast<int>(std::lround(t));
      if (idx != prev) {  // np.unique drops duplicates (already sorted)
        sel.push_back(idx);
        prev = idx;
      }
    }
    std::vector<int> sub;
    sub.reserve(sel.size());
    for (const int k : sel) sub.push_back(valid_idx[static_cast<std::size_t>(k)]);
    valid_idx = std::move(sub);
  }

  std::vector<double> pts;
  pts.reserve(valid_idx.size() * 2);
  for (const int i : valid_idx) {
    const double a = angles[static_cast<std::size_t>(i)];
    const double r = scan.ranges[static_cast<std::size_t>(i)];
    // Robot frame: x forward, y left; beam angle 0 = forward.
    pts.push_back(r * std::cos(a));
    pts.push_back(r * std::sin(a));
  }
  return pts;
}

std::optional<amr_core::Pose2D> ScanMatchingSlam::match(
    const std::vector<double>& pts) const {
  const amr_core::Pose2D& base = pose_;

  // Coarse stage centered on the predicted pose.
  auto [c_best, c_score] =
      search(pts, base, cfg_.coarse_window_xy, cfg_.coarse_step_xy,
             cfg_.coarse_window_theta, cfg_.coarse_step_theta);

  // Fine stage centered on the coarse winner.
  auto [f_best, f_score] =
      search(pts, c_best, cfg_.coarse_step_xy, cfg_.fine_step_xy,
             cfg_.coarse_step_theta, cfg_.fine_step_theta);

  amr_core::Pose2D best;
  double best_score;
  if (f_score >= c_score) {
    best = f_best;
    best_score = f_score;
  } else {
    best = c_best;
    best_score = c_score;
  }

  if (best_score >= cfg_.min_match_score) {
    return best;
  }
  return std::nullopt;
}

std::pair<amr_core::Pose2D, double> ScanMatchingSlam::search(
    const std::vector<double>& pts, const amr_core::Pose2D& center,
    double window_xy, double step_xy, double window_theta,
    double step_theta) const {
  const int rows = rows_;
  const int cols = cols_;
  const double res = resolution_;
  const double inv_res = 1.0 / res;
  const double ox = origin_x_;
  const double oy = origin_y_;

  const std::vector<double> offs_xy = linrange(window_xy, step_xy);
  const std::vector<double> offs_th = linrange(window_theta, step_theta);

  // Translation grid relative to the center. np.meshgrid(offs_xy, offs_xy)
  // then column-stack of (dxx.ravel(), dyy.ravel()): dxx varies fastest along
  // the inner (column) axis, dyy along the outer (row) axis.
  std::vector<double> trans_dx;
  std::vector<double> trans_dy;
  trans_dx.reserve(offs_xy.size() * offs_xy.size());
  trans_dy.reserve(offs_xy.size() * offs_xy.size());
  for (const double dy : offs_xy) {
    for (const double dx : offs_xy) {
      trans_dx.push_back(dx);
      trans_dy.push_back(dy);
    }
  }
  const std::size_t kxy = trans_dx.size();

  const std::size_t beams = pts.size() / 2;
  const double b = static_cast<double>(beams);

  double best_score = -1.0;
  double best_dx = 0.0;
  double best_dy = 0.0;
  double best_theta = center.theta;

  for (const double dth : offs_th) {
    const double theta = center.theta + dth;
    const double c = std::cos(theta);
    const double sn = std::sin(theta);

    // Rotate robot-frame points into a heading-aligned world offset.
    std::vector<double> rx(beams);
    std::vector<double> ry(beams);
    for (std::size_t j = 0; j < beams; ++j) {
      const double px = pts[2 * j];
      const double py = pts[2 * j + 1];
      rx[j] = px * c - py * sn;
      ry[j] = px * sn + py * c;
    }

    // Score every translation candidate by the mean score-map value at the
    // rotated scan endpoints (out-of-map contributes 0).
    for (std::size_t k = 0; k < kxy; ++k) {
      const double cand_x = center.x + trans_dx[k];
      const double cand_y = center.y + trans_dy[k];
      double acc = 0.0;
      for (std::size_t j = 0; j < beams; ++j) {
        const double ex = cand_x + rx[j];
        const double ey = cand_y + ry[j];
        const long cc =
            static_cast<long>(std::floor((ex - ox) * inv_res));
        const long cr =
            static_cast<long>(std::floor((ey - oy) * inv_res));
        if (cr >= 0 && cr < rows && cc >= 0 && cc < cols) {
          acc += score_map_[static_cast<std::size_t>(cr) * cols + cc];
        }
      }
      const double mean = acc / b;  // mean over beams
      if (mean > best_score) {
        best_score = mean;
        best_dx = trans_dx[k];
        best_dy = trans_dy[k];
        best_theta = theta;
      }
    }
  }

  amr_core::Pose2D best{center.x + best_dx, center.y + best_dy,
                        amr_core::wrap_angle(best_theta)};
  return {best, best_score};
}

std::vector<double> ScanMatchingSlam::linrange(double window, double step) {
  // Symmetric offsets in [-window, window] inclusive at `step` spacing.
  if (step <= 0.0 || window <= 0.0) {
    return {0.0};
  }
  const int n = static_cast<int>(std::lround(window / step));
  std::vector<double> out;
  out.reserve(static_cast<std::size_t>(2 * n + 1));
  for (int i = -n; i <= n; ++i) {
    out.push_back(static_cast<double>(i) * step);
  }
  return out;
}

// ----------------------------------------------------------------------------
// Small helpers
// ----------------------------------------------------------------------------

std::pair<double, double> ScanMatchingSlam::motion_since(
    const amr_core::Pose2D& ref) const {
  const double d_trans = std::hypot(pose_.x - ref.x, pose_.y - ref.y);
  const double d_rot = std::abs(amr_core::wrap_angle(pose_.theta - ref.theta));
  return {d_trans, d_rot};
}

}  // namespace amr_slam
