// SPDX-License-Identifier: Apache-2.0
// Geometry primitives shared by all subsystems. Mirrors amr/core/geometry.py.
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "amr_core/types.hpp"

namespace amr_core {

/// Wrap a scalar angle to (-pi, pi].
double wrap_angle(double a);

/// a (+) b: pose of frame b (expressed in a) in a's parent frame.
Pose2D pose_compose(const Pose2D& a, const Pose2D& b);

/// a (-) b: pose of b expressed in frame a, so pose_compose(a, result) == b.
Pose2D pose_between(const Pose2D& a, const Pose2D& b);

/// Transform interleaved [x0,y0,x1,y1,...] points from pose's frame to world.
std::vector<double> transform_points(const Pose2D& pose,
                                     const std::vector<double>& pts);

/// All integer grid cells on the line (r0,c0)->(r1,c1), inclusive.
std::vector<std::pair<int, int>> bresenham(int r0, int c0, int r1, int c1);

/// Chamfer distance (m) from each cell to the nearest occupied cell.
/// `occupied` is row-major (rows*cols); non-zero marks an occupied cell.
/// Iterative 8-neighbour relaxation, capped at max_dist. No external deps.
std::vector<float> distance_field(const std::vector<uint8_t>& occupied,
                                  int rows, int cols, double resolution,
                                  double max_dist);

}  // namespace amr_core
