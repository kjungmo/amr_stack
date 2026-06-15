// SPDX-License-Identifier: Apache-2.0
// Correlative scan-matching SLAM. Faithful C++ port of
// amr/slam/scan_matching_slam.py.
//
// A lightweight 2D SLAM front-end that fuses noisy odometry increments with a
// correlative scan match against a blurred occupancy *score map* derived from
// the log-odds map maintained by amr_mapping::OccupancyGridMapper.
//
// Pipeline per process() call (mirrors the Python docstring step list):
//   1. Dead-reckon the estimate forward with the odometry increment
//      (pose = pose_compose(pose, odom_delta)). If there is no scan, return.
//   2. The very first scan seeds the map and keyframe bookkeeping, no matching.
//   3. Skip matching if the motion since the last processed scan is below
//      min_motion (both translation and |rotation|).
//   4. Build a blurred occupancy score map: p_occ = sigmoid(log_odds),
//      score_src = (p_occ > 0.6) as float, blurred with a separable Gaussian
//      (sigma = blur_sigma_cells, half-width 3 sigma) via shifted-slice sums,
//      peak-normalized to 1. Rebuilt only after a keyframe map update.
//   5. Two-stage correlative search (coarse then fine) over candidate
//      (dx, dy, dtheta) offsets around the predicted pose, scoring each
//      candidate by the mean score-map value at the scan endpoints.
//   6. Accept the best candidate iff its score >= min_match_score (otherwise
//      the early, thin map is not trustworthy -- keep the odometry prediction).
//   7. On keyframe motion (>= keyframe_trans or keyframe_rot since the last
//      keyframe), integrate the scan into the map and rebuild the score map.
//
// Stochastic-free; randomness lives upstream in the simulator. Coordinate
// conventions follow CONTRACT.md §2.
#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_mapping/occupancy_grid_mapper.hpp"

namespace amr_slam {

class ScanMatchingSlam {
 public:
  /// size_m = (width_x, height_y) in metres; the map origin is (0, 0).
  ScanMatchingSlam(const amr_core::SlamConfig& cfg,
                   const amr_core::MappingConfig& mapping_cfg,
                   std::pair<double, double> size_m,
                   const amr_core::Pose2D& initial_pose);

  /// Fuse one odometry increment and (optionally) one scan; return the pose.
  amr_core::Pose2D process(const amr_core::Pose2D& odom_delta,
                           const std::optional<amr_core::LaserScan>& scan);

  /// Current best pose estimate (map frame).
  const amr_core::Pose2D& pose() const { return pose_; }

  /// Snapshot the current map as a ROS-style OccupancyGrid.
  amr_core::OccupancyGrid get_map() const { return mapper_.to_occupancy_grid(); }

  /// Underlying mapper (for map_io / inspection).
  const amr_mapping::OccupancyGridMapper& mapper() const { return mapper_; }

 private:
  // -- Score map construction -------------------------------------------
  void rebuild_score_map();
  static std::vector<double> gaussian_kernel(double sigma);
  // Separable blur of a (rows x cols) row-major float field via shifted-slice
  // sums (zero padding at the boundary), matching the Python implementation.
  std::vector<double> separable_blur(const std::vector<double>& src) const;

  // -- Correlative scan matching ----------------------------------------
  // Returns subsampled valid beams as interleaved robot-frame [x0,y0,x1,y1,...].
  std::vector<double> scan_points(const amr_core::LaserScan& scan) const;
  // Two-stage search; returns the matched pose, or std::nullopt to keep odom.
  std::optional<amr_core::Pose2D> match(const std::vector<double>& pts) const;
  // Vectorized correlative search over a (dx, dy, dtheta) grid; returns the
  // best candidate pose and its mean score.
  std::pair<amr_core::Pose2D, double> search(const std::vector<double>& pts,
                                             const amr_core::Pose2D& center,
                                             double window_xy, double step_xy,
                                             double window_theta,
                                             double step_theta) const;
  static std::vector<double> linrange(double window, double step);

  // -- Small helpers -----------------------------------------------------
  std::pair<double, double> motion_since(const amr_core::Pose2D& ref) const;

  amr_core::SlamConfig cfg_;
  amr_core::MappingConfig mapping_cfg_;
  double resolution_;

  amr_core::Pose2D pose_;
  amr_mapping::OccupancyGridMapper mapper_;

  double origin_x_{0.0};
  double origin_y_{0.0};

  bool have_first_{false};
  amr_core::Pose2D last_kf_pose_{};
  amr_core::Pose2D last_proc_pose_{};

  // Blurred occupancy score map (row-major, rows_*cols_), rebuilt on keyframes.
  std::vector<double> score_map_;
  bool have_score_map_{false};
  int rows_{0};
  int cols_{0};

  std::vector<double> blur_kernel_;
};

}  // namespace amr_slam
