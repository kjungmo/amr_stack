// SPDX-License-Identifier: Apache-2.0
// ROS-compatible map I/O: save/load PGM+YAML map-file pairs.
// Faithful C++ port of amr/mapping/map_io.py.
//
// Pixel convention (CONTRACT §2 item 6):
//   occupied (100) -> pixel 0   (black)
//   free     (0)   -> pixel 254 (white)
//   unknown  (-1)  -> pixel 205 (grey)
//
// Image rows are flipped on write (image row 0 = top / north) while grid row 0
// = south (minimum y). Load reverses the flip.
#pragma once

#include <string>
#include <utility>

#include "amr_core/types.hpp"

namespace amr_mapping {

/// Write <stem>.pgm (P5 binary) and <stem>.yaml. Returns (pgm_path, yaml_path).
std::pair<std::string, std::string> save_map(const amr_core::OccupancyGrid& grid,
                                             const std::string& path_stem);

/// Load a PGM+YAML map pair. The PGM image is resolved relative to the YAML.
amr_core::OccupancyGrid load_map(const std::string& yaml_path);

}  // namespace amr_mapping
