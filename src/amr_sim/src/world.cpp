// SPDX-License-Identifier: Apache-2.0
#include "amr_sim/world.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace amr_sim {

World World::from_yaml(const std::string& path) {
  // Fail with a clear, actionable message rather than an opaque YAML::BadFile
  // (which, if uncaught, aborts the node). The world path must be absolute or
  // resolvable from the launching process's cwd.
  {
    std::ifstream probe(path);
    if (!probe.good()) {
      throw std::runtime_error(
          "amr_sim: world file not found or unreadable: '" + path +
          "'. Pass an absolute path via the 'world_file' parameter (the launch "
          "files resolve it from amr_bringup/share/config/office.world.yaml).");
    }
  }
  const YAML::Node d = YAML::LoadFile(path);

  const double size_x = d["size"][0].as<double>();
  const double size_y = d["size"][1].as<double>();
  const double res = d["resolution"].as<double>();
  const int rows = static_cast<int>(std::lround(size_y / res));
  const int cols = static_cast<int>(std::lround(size_x / res));

  amr_core::OccupancyGrid grid;
  grid.resolution = res;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.rows = rows;
  grid.cols = cols;
  grid.data.assign(static_cast<std::size_t>(rows) * cols, 0);

  // Cell-center world coordinates (col -> x, row -> y).
  std::vector<double> xs(cols);
  for (int c = 0; c < cols; ++c) {
    xs[c] = (static_cast<double>(c) + 0.5) * res;
  }
  std::vector<double> ys(rows);
  for (int r = 0; r < rows; ++r) {
    ys[r] = (static_cast<double>(r) + 0.5) * res;
  }

  if (d["obstacles"]) {
    for (const auto& obs : d["obstacles"]) {
      const std::string otype = obs["type"].as<std::string>();
      if (otype == "rect") {
        const double ox = obs["x"].as<double>();
        const double oy = obs["y"].as<double>();
        const double ow = obs["w"].as<double>();
        const double oh = obs["h"].as<double>();
        for (int r = 0; r < rows; ++r) {
          const double cy = ys[r];
          if (cy < oy || cy >= oy + oh) {
            continue;
          }
          for (int c = 0; c < cols; ++c) {
            const double cx = xs[c];
            if (cx >= ox && cx < ox + ow) {
              grid.at(r, c) = 100;
            }
          }
        }
      } else if (otype == "circle") {
        const double ox = obs["x"].as<double>();
        const double oy = obs["y"].as<double>();
        const double rad = obs["r"].as<double>();
        const double r2 = rad * rad;
        for (int r = 0; r < rows; ++r) {
          const double dy = ys[r] - oy;
          for (int c = 0; c < cols; ++c) {
            const double dx = xs[c] - ox;
            if (dx * dx + dy * dy <= r2) {
              grid.at(r, c) = 100;
            }
          }
        }
      } else {
        throw std::runtime_error("unknown obstacle type '" + otype + "'");
      }
    }
  }

  const auto& sp = d["spawn"];
  amr_core::Pose2D spawn;
  spawn.x = sp["x"].as<double>();
  spawn.y = sp["y"].as<double>();
  spawn.theta = sp["theta"] ? sp["theta"].as<double>() : 0.0;

  return World(std::move(grid), spawn, {size_x, size_y});
}

bool World::is_occupied_world(double x, double y) const {
  int row, col;
  grid_.world_to_grid(x, y, row, col);
  if (!grid_.in_bounds(row, col)) {
    return true;
  }
  return grid_.at(row, col) >= 50;
}

}  // namespace amr_sim
