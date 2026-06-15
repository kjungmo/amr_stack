// SPDX-License-Identifier: Apache-2.0
// gtests for amr_mapping::{save_map, load_map}. Mirrors tests/test_map_io.py.
#include "amr_mapping/map_io.hpp"

#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "amr_core/types.hpp"
#include "gtest/gtest.h"

namespace {

// A unique writable stem under the system temp directory.
std::string temp_stem(const std::string& tag) {
  std::string base = "/tmp";
  if (const char* t = std::getenv("TMPDIR")) {
    base = t;
  }
  return base + "/amr_mapping_test_" + tag + "_" +
         std::to_string(static_cast<long>(::getpid()));
}

amr_core::OccupancyGrid make_grid(int rows, int cols, double res, double ox,
                                  double oy) {
  amr_core::OccupancyGrid g;
  g.resolution = res;
  g.origin_x = ox;
  g.origin_y = oy;
  g.rows = rows;
  g.cols = cols;
  g.data.assign(static_cast<std::size_t>(rows) * cols, static_cast<int8_t>(-1));
  return g;
}

}  // namespace

// Mirrors test_roundtrip: write then read preserves cells, resolution, origin.
TEST(MapIo, Roundtrip) {
  amr_core::OccupancyGrid g = make_grid(5, 7, 0.05, -1.0, 2.0);
  g.data[0 * 7 + 0] = 0;     // (row 0, col 0)
  g.data[4 * 7 + 6] = 100;   // (row 4, col 6)
  g.data[2 * 7 + 3] = 0;     // (row 2, col 3)

  const std::string stem = temp_stem("roundtrip");
  const auto paths = amr_mapping::save_map(g, stem);

  amr_core::OccupancyGrid g2 = amr_mapping::load_map(paths.second);

  EXPECT_EQ(g2.rows, g.rows);
  EXPECT_EQ(g2.cols, g.cols);
  EXPECT_EQ(g2.data, g.data);
  EXPECT_DOUBLE_EQ(g2.resolution, g.resolution);
  EXPECT_DOUBLE_EQ(g2.origin_x, g.origin_x);
  EXPECT_DOUBLE_EQ(g2.origin_y, g.origin_y);

  std::remove(paths.first.c_str());
  std::remove(paths.second.c_str());
}

// Mirrors test_pgm_pixel_convention: occupied->0, free->254, unknown->205.
TEST(MapIo, PgmPixelConvention) {
  // Single row [100, 0, -1]; one row means flip is a no-op.
  amr_core::OccupancyGrid g = make_grid(1, 3, 0.1, 0.0, 0.0);
  g.data[0] = 100;
  g.data[1] = 0;
  g.data[2] = -1;

  const std::string stem = temp_stem("pixel");
  const auto paths = amr_mapping::save_map(g, stem);

  std::ifstream f(paths.first, std::ios::binary);
  ASSERT_TRUE(f.good());
  std::vector<char> raw((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());
  f.close();

  // Find the maxval "255" then skip the single whitespace delimiter.
  const std::string token = "255";
  std::size_t p = std::string(raw.begin(), raw.end()).find(token);
  ASSERT_NE(p, std::string::npos);
  p += token.size();
  while (p < raw.size() &&
         (raw[p] == '\n' || raw[p] == ' ' || raw[p] == '\t' || raw[p] == '\r')) {
    ++p;
  }
  ASSERT_LE(p + 3, raw.size());
  EXPECT_EQ(static_cast<uint8_t>(raw[p + 0]), 0);
  EXPECT_EQ(static_cast<uint8_t>(raw[p + 1]), 254);
  EXPECT_EQ(static_cast<uint8_t>(raw[p + 2]), 205);

  std::remove(paths.first.c_str());
  std::remove(paths.second.c_str());
}
