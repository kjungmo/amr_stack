// SPDX-License-Identifier: Apache-2.0
// ROS-compatible map I/O. Faithful C++ port of amr/mapping/map_io.py.
#include "amr_mapping/map_io.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <ios>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace amr_mapping {

namespace {

// Basename / dirname helpers (POSIX-style '/' separators, matching os.path use).
std::string basename_of(const std::string& path) {
  const auto pos = path.find_last_of('/');
  return (pos == std::string::npos) ? path : path.substr(pos + 1);
}

std::string dirname_of(const std::string& path) {
  const auto pos = path.find_last_of('/');
  return (pos == std::string::npos) ? std::string() : path.substr(0, pos);
}

bool is_absolute(const std::string& path) {
  return !path.empty() && path.front() == '/';
}

}  // namespace

std::pair<std::string, std::string> save_map(const amr_core::OccupancyGrid& grid,
                                             const std::string& path_stem) {
  const std::string pgm_path = path_stem + ".pgm";
  const std::string yaml_path = path_stem + ".yaml";

  const int rows = grid.rows;
  const int cols = grid.cols;

  // Map int8 occupancy values to PGM pixel values.
  //   occupied (100) -> 0 ; free (0) -> 254 ; everything else -> 205 (unknown).
  std::vector<uint8_t> pixels(static_cast<std::size_t>(rows) * cols, 205);
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    const int8_t v = grid.data[i];
    if (v == 100) {
      pixels[i] = 0;
    } else if (v == 0) {
      pixels[i] = 254;
    } else {
      pixels[i] = 205;
    }
  }

  // Flip vertically so image row 0 = top (north): image row i = grid row
  // (rows-1-i).
  std::vector<uint8_t> pixels_img(pixels.size());
  for (int r = 0; r < rows; ++r) {
    const int src = rows - 1 - r;
    std::copy(pixels.begin() + static_cast<std::size_t>(src) * cols,
              pixels.begin() + static_cast<std::size_t>(src + 1) * cols,
              pixels_img.begin() + static_cast<std::size_t>(r) * cols);
  }

  // Write PGM P5 (binary). Header: "P5\n<cols> <rows>\n255\n".
  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("save_map: cannot open " + pgm_path);
  }
  pgm << "P5\n" << cols << ' ' << rows << "\n255\n";
  pgm.write(reinterpret_cast<const char*>(pixels_img.data()),
            static_cast<std::streamsize>(pixels_img.size()));
  pgm.close();

  // Write YAML sidecar (block style, matching yaml.dump default_flow_style).
  std::ofstream yml(yaml_path);
  if (!yml) {
    throw std::runtime_error("save_map: cannot open " + yaml_path);
  }
  // Full round-trip precision so load_map recovers the exact origin/resolution.
  yml.precision(std::numeric_limits<double>::max_digits10);
  yml << "free_thresh: 0.25\n";
  yml << "image: " << basename_of(pgm_path) << "\n";
  yml << "negate: 0\n";
  yml << "occupied_thresh: 0.65\n";
  yml << "origin:\n";
  yml << "- " << grid.origin_x << "\n";
  yml << "- " << grid.origin_y << "\n";
  yml << "- 0.0\n";
  yml << "resolution: " << grid.resolution << "\n";
  yml.close();

  return {pgm_path, yaml_path};
}

amr_core::OccupancyGrid load_map(const std::string& yaml_path) {
  std::ifstream yml(yaml_path);
  if (!yml) {
    throw std::runtime_error("load_map: cannot open " + yaml_path);
  }

  // Minimal YAML reader for the fixed schema emitted by save_map (and the
  // equivalent Python yaml.dump output). Parses scalars + the 3-element origin
  // sequence regardless of flow/block style.
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::string image;

  std::string line;
  while (std::getline(yml, line)) {
    // Strip trailing CR.
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    // Block-style origin sequence entries ("- <num>") are consumed inline while
    // handling the `origin` key below; here we only dispatch on "key: value".
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    std::string key = line.substr(0, colon);
    // Trim key whitespace.
    const auto ks = key.find_first_not_of(" \t");
    const auto ke = key.find_last_not_of(" \t");
    if (ks == std::string::npos) {
      continue;
    }
    key = key.substr(ks, ke - ks + 1);
    std::string val = line.substr(colon + 1);
    const auto vs = val.find_first_not_of(" \t");
    std::string vtrim = (vs == std::string::npos) ? std::string()
                                                  : val.substr(vs);

    if (key == "resolution") {
      resolution = std::stod(vtrim);
    } else if (key == "image") {
      image = vtrim;
    } else if (key == "origin") {
      if (!vtrim.empty() && vtrim.front() == '[') {
        // Flow style: [x, y, z].
        std::string inner = vtrim.substr(1);
        const auto close = inner.find(']');
        if (close != std::string::npos) {
          inner = inner.substr(0, close);
        }
        std::stringstream ss(inner);
        std::string tok;
        std::vector<double> vals;
        while (std::getline(ss, tok, ',')) {
          const auto ts = tok.find_first_not_of(" \t");
          if (ts != std::string::npos) {
            vals.push_back(std::stod(tok.substr(ts)));
          }
        }
        if (vals.size() >= 2) {
          origin_x = vals[0];
          origin_y = vals[1];
        }
      } else {
        // Block style: read the next two "- <num>" entries.
        std::vector<double> vals;
        std::streampos save_pos = yml.tellg();
        std::string sline;
        while (vals.size() < 3 && std::getline(yml, sline)) {
          if (!sline.empty() && sline.back() == '\r') {
            sline.pop_back();
          }
          const auto dash = sline.find_first_not_of(" \t");
          if (dash != std::string::npos && sline[dash] == '-') {
            std::string num = sline.substr(dash + 1);
            const auto ns = num.find_first_not_of(" \t");
            if (ns != std::string::npos) {
              vals.push_back(std::stod(num.substr(ns)));
            }
            save_pos = yml.tellg();
          } else {
            // Not a sequence entry; rewind so the outer loop re-reads it.
            yml.seekg(save_pos);
            break;
          }
        }
        if (vals.size() >= 2) {
          origin_x = vals[0];
          origin_y = vals[1];
        }
      }
    }
  }
  yml.close();

  if (image.empty()) {
    throw std::runtime_error("load_map: missing 'image' key in " + yaml_path);
  }

  // Resolve PGM path relative to the YAML directory.
  std::string pgm_path;
  if (is_absolute(image)) {
    pgm_path = image;
  } else {
    const std::string dir = dirname_of(yaml_path);
    pgm_path = dir.empty() ? image : (dir + "/" + image);
  }

  std::ifstream pgm(pgm_path, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("load_map: cannot open " + pgm_path);
  }
  std::vector<char> raw((std::istreambuf_iterator<char>(pgm)),
                        std::istreambuf_iterator<char>());
  pgm.close();

  // Parse header tokens: "P5", cols, rows, maxval — skipping whitespace and
  // '#' comments, exactly like the Python parser.
  std::size_t pos = 0;
  auto next_token = [&raw, &pos]() -> std::string {
    while (pos < raw.size()) {
      const char ch = raw[pos];
      if (ch == '#') {
        while (pos < raw.size() && raw[pos] != '\n') {
          ++pos;
        }
      } else if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
        ++pos;
      } else {
        break;
      }
    }
    const std::size_t start = pos;
    while (pos < raw.size()) {
      const char ch = raw[pos];
      if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
        break;
      }
      ++pos;
    }
    return std::string(raw.begin() + start, raw.begin() + pos);
  };

  const std::string magic = next_token();
  if (magic != "P5") {
    throw std::runtime_error("load_map: expected P5 PGM, got " + magic);
  }
  const int cols = std::stoi(next_token());
  const int rows = std::stoi(next_token());
  (void)next_token();  // maxval, ignored.
  ++pos;               // single whitespace byte separating header from data.

  const std::size_t n = static_cast<std::size_t>(rows) * cols;
  if (pos + n > raw.size()) {
    throw std::runtime_error("load_map: truncated PGM data in " + pgm_path);
  }

  // pixels_img[row, col]; reverse the flip applied on save:
  //   grid row r  <-  image row (rows-1-r).
  // Map pixel -> ROS occupancy: pixel<=50 -> 100, pixel>=250 -> 0, else -1.
  amr_core::OccupancyGrid grid;
  grid.resolution = resolution;
  grid.origin_x = origin_x;
  grid.origin_y = origin_y;
  grid.rows = rows;
  grid.cols = cols;
  grid.data.assign(n, -1);

  for (int r = 0; r < rows; ++r) {
    const int img_row = rows - 1 - r;  // flipud
    for (int c = 0; c < cols; ++c) {
      const uint8_t px = static_cast<uint8_t>(
          raw[pos + static_cast<std::size_t>(img_row) * cols + c]);
      int8_t occ = -1;
      if (px <= 50) {
        occ = 100;
      } else if (px >= 250) {
        occ = 0;
      }
      grid.data[static_cast<std::size_t>(r) * cols + c] = occ;
    }
  }

  return grid;
}

}  // namespace amr_mapping
