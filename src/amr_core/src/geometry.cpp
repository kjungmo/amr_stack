// SPDX-License-Identifier: Apache-2.0
#include "amr_core/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace amr_core {

double wrap_angle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

Pose2D pose_compose(const Pose2D& a, const Pose2D& b) {
  const double c = std::cos(a.theta);
  const double s = std::sin(a.theta);
  return Pose2D{a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y,
                wrap_angle(a.theta + b.theta)};
}

Pose2D pose_between(const Pose2D& a, const Pose2D& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double c = std::cos(a.theta);
  const double s = std::sin(a.theta);
  return Pose2D{c * dx + s * dy, -s * dx + c * dy, wrap_angle(b.theta - a.theta)};
}

std::vector<double> transform_points(const Pose2D& pose,
                                     const std::vector<double>& pts) {
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  std::vector<double> out(pts.size());
  for (std::size_t i = 0; i + 1 < pts.size(); i += 2) {
    const double x = pts[i];
    const double y = pts[i + 1];
    out[i] = c * x - s * y + pose.x;
    out[i + 1] = s * x + c * y + pose.y;
  }
  return out;
}

std::vector<std::pair<int, int>> bresenham(int r0, int c0, int r1, int c1) {
  std::vector<std::pair<int, int>> cells;
  const int dr = std::abs(r1 - r0);
  const int dc = std::abs(c1 - c0);
  const int sr = (r1 >= r0) ? 1 : -1;
  const int sc = (c1 >= c0) ? 1 : -1;
  int err = dc - dr;
  int r = r0;
  int c = c0;
  while (true) {
    cells.emplace_back(r, c);
    if (r == r1 && c == c1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 > -dr) {
      err -= dr;
      c += sc;
    }
    if (e2 < dc) {
      err += dc;
      r += sr;
    }
  }
  return cells;
}

std::vector<float> distance_field(const std::vector<uint8_t>& occupied,
                                  int rows, int cols, double resolution,
                                  double max_dist) {
  const float big = static_cast<float>(max_dist / resolution + 2.0);
  const float sq2 = static_cast<float>(std::sqrt(2.0));
  const std::size_t n = static_cast<std::size_t>(rows) * cols;

  std::vector<float> d(n);
  for (std::size_t i = 0; i < n; ++i) {
    d[i] = occupied[i] ? 0.0f : big;
  }

  const int sweeps = static_cast<int>(std::ceil(max_dist / resolution)) + 1;
  auto idx = [cols](int r, int c) {
    return static_cast<std::size_t>(r) * cols + c;
  };

  std::vector<float> nd(n);
  for (int s = 0; s < sweeps; ++s) {
    nd = d;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        float best = d[idx(r, c)];
        if (r > 0) best = std::min(best, d[idx(r - 1, c)] + 1.0f);
        if (r < rows - 1) best = std::min(best, d[idx(r + 1, c)] + 1.0f);
        if (c > 0) best = std::min(best, d[idx(r, c - 1)] + 1.0f);
        if (c < cols - 1) best = std::min(best, d[idx(r, c + 1)] + 1.0f);
        if (r > 0 && c > 0) best = std::min(best, d[idx(r - 1, c - 1)] + sq2);
        if (r > 0 && c < cols - 1)
          best = std::min(best, d[idx(r - 1, c + 1)] + sq2);
        if (r < rows - 1 && c > 0)
          best = std::min(best, d[idx(r + 1, c - 1)] + sq2);
        if (r < rows - 1 && c < cols - 1)
          best = std::min(best, d[idx(r + 1, c + 1)] + sq2);
        nd[idx(r, c)] = best;
      }
    }
    if (nd == d) {
      break;
    }
    d.swap(nd);
  }

  std::vector<float> out(n);
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = static_cast<float>(
        std::min(static_cast<double>(d[i]) * resolution, max_dist));
  }
  return out;
}

}  // namespace amr_core
