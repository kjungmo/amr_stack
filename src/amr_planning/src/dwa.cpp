// SPDX-License-Identifier: Apache-2.0
#include "amr_planning/dwa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "amr_core/geometry.hpp"

namespace amr_planning {

std::array<double, 2> carrot_point(const Path& path,
                                   const amr_core::Pose2D& pose,
                                   double lookahead) {
  if (path.size() == 1) {
    return path[0];
  }
  const double px = pose.x;
  const double py = pose.y;

  // Find the closest point on any segment; record its cumulative arc length.
  double best_d2 = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  double arc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double ax = path[i][0];
    const double ay = path[i][1];
    const double bx = path[i + 1][0];
    const double by = path[i + 1][1];
    const double segx = bx - ax;
    const double segy = by - ay;
    const double seg_len = std::hypot(segx, segy);
    if (seg_len < 1e-12) {
      continue;
    }
    double t = ((px - ax) * segx + (py - ay) * segy) / (seg_len * seg_len);
    t = std::min(1.0, std::max(0.0, t));
    const double projx = ax + t * segx;
    const double projy = ay + t * segy;
    const double d2 = (px - projx) * (px - projx) + (py - projy) * (py - projy);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_arc = arc + t * seg_len;
    }
    arc += seg_len;
  }

  const double target_arc = best_arc + lookahead;

  // Walk forward along the polyline to the target arc length.
  arc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double ax = path[i][0];
    const double ay = path[i][1];
    const double bx = path[i + 1][0];
    const double by = path[i + 1][1];
    const double segx = bx - ax;
    const double segy = by - ay;
    const double seg_len = std::hypot(segx, segy);
    if (seg_len < 1e-12) {
      continue;
    }
    if (arc + seg_len >= target_arc) {
      double t = (target_arc - arc) / seg_len;
      t = std::min(1.0, std::max(0.0, t));
      return {ax + t * segx, ay + t * segy};
    }
    arc += seg_len;
  }
  // Past the end of the path -> clamp to the final point.
  return path.back();
}

std::vector<std::array<double, 3>> DwaPlanner::rollout(
    const amr_core::Pose2D& pose, double v, double omega) const {
  const int n = static_cast<int>(std::lround(cfg_.sim_time / cfg_.sim_dt));
  double x = pose.x;
  double y = pose.y;
  double th = pose.theta;
  const double dt = cfg_.sim_dt;
  std::vector<std::array<double, 3>> out;
  out.reserve(static_cast<std::size_t>(std::max(0, n)));
  for (int k = 0; k < n; ++k) {
    x += v * std::cos(th) * dt;
    y += v * std::sin(th) * dt;
    th = amr_core::wrap_angle(th + omega * dt);
    out.push_back({x, y, th});
  }
  return out;
}

DwaResult DwaPlanner::compute(const amr_core::Pose2D& pose,
                              const amr_core::Twist2D& vel, const Path& path,
                              const Costmap& costmap) const {
  const double T = cfg_.sim_time;
  const double v_max = robot_.max_lin_vel;
  const double w_max = robot_.max_ang_vel;

  // Dynamic window (sim_time used as the reachability horizon).
  double v_lo = std::max(0.0, vel.v - robot_.max_lin_acc * T);
  double v_hi = std::min(v_max, vel.v + robot_.max_lin_acc * T);
  double w_lo = std::max(-w_max, vel.omega - robot_.max_ang_acc * T);
  double w_hi = std::min(w_max, vel.omega + robot_.max_ang_acc * T);

  const std::array<double, 2> carrot = carrot_point(path, pose, cfg_.lookahead);
  const std::array<double, 2> goal = path.back();
  const double dist_to_goal =
      std::hypot(goal[0] - pose.x, goal[1] - pose.y);

  // Near-goal slowdown: cap sampled v.
  if (dist_to_goal < cfg_.lookahead) {
    v_hi = std::min(v_hi, std::max(0.1, 0.7 * dist_to_goal));
    v_hi = std::max(v_hi, v_lo);
  }

  // np.linspace semantics: endpoints inclusive; a single sample yields v_lo.
  auto linspace = [](double lo, double hi, int num) {
    std::vector<double> g(static_cast<std::size_t>(std::max(0, num)));
    if (num == 1) {
      g[0] = lo;
      return g;
    }
    for (int i = 0; i < num; ++i) {
      g[static_cast<std::size_t>(i)] =
          lo + (hi - lo) * static_cast<double>(i) / static_cast<double>(num - 1);
    }
    return g;
  };

  const std::vector<double> v_grid = linspace(v_lo, v_hi, cfg_.v_samples);
  const std::vector<double> w_grid = linspace(w_lo, w_hi, cfg_.w_samples);

  // Collect surviving rollouts and their raw score components.
  std::vector<std::vector<std::array<double, 3>>> trajs;
  std::vector<std::array<double, 2>> vels;  // (v, w)
  std::vector<double> progress, heading, clearance, velocity;

  for (double v : v_grid) {
    for (double w : w_grid) {
      auto traj = rollout(pose, v, w);
      // Per-pose cost; reject if any pose is lethal.
      bool lethal = false;
      double min_one_minus_cost = std::numeric_limits<double>::infinity();
      for (const auto& p : traj) {
        const double cck = costmap.cost_at_world(p[0], p[1]);
        if (cck >= 0.99) {
          lethal = true;
          break;
        }
        min_one_minus_cost = std::min(min_one_minus_cost, 1.0 - cck);
      }
      if (lethal) {
        continue;
      }

      const auto& end = traj.back();
      const double d_end = std::hypot(end[0] - carrot[0], end[1] - carrot[1]);
      const double bearing =
          std::atan2(carrot[1] - end[1], carrot[0] - end[0]);
      const double head_err = std::abs(amr_core::wrap_angle(bearing - end[2]));
      const double clear = min_one_minus_cost;

      trajs.push_back(std::move(traj));
      vels.push_back({v, w});
      progress.push_back(-d_end);
      heading.push_back(-head_err);
      clearance.push_back(clear);
      velocity.push_back(v_max > 0.0 ? v / v_max : 0.0);
    }
  }

  if (trajs.empty()) {
    DwaResult r;
    r.cmd = amr_core::Twist2D{0.0, 0.0};
    r.blocked = true;
    return r;
  }

  // Min-max normalisation: constant component -> all ones (matches Python).
  auto norm = [](const std::vector<double>& a) {
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (double x : a) {
      lo = std::min(lo, x);
      hi = std::max(hi, x);
    }
    std::vector<double> out(a.size());
    if (hi - lo < 1e-12) {
      std::fill(out.begin(), out.end(), 1.0);
      return out;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
      out[i] = (a[i] - lo) / (hi - lo);
    }
    return out;
  };

  const std::vector<double> np_progress = norm(progress);
  const std::vector<double> np_heading = norm(heading);
  const std::vector<double> np_clearance = norm(clearance);
  const std::vector<double> np_velocity = norm(velocity);

  int best = 0;
  double best_total = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trajs.size(); ++i) {
    const double total = cfg_.w_progress * np_progress[i] +
                         cfg_.w_heading * np_heading[i] +
                         cfg_.w_clearance * np_clearance[i] +
                         cfg_.w_velocity * np_velocity[i];
    if (total > best_total) {
      best_total = total;
      best = static_cast<int>(i);
    }
  }

  DwaResult r;
  r.cmd = amr_core::Twist2D{vels[static_cast<std::size_t>(best)][0],
                            vels[static_cast<std::size_t>(best)][1]};
  r.trajectory = trajs[static_cast<std::size_t>(best)];
  r.blocked = false;
  return r;
}

}  // namespace amr_planning
