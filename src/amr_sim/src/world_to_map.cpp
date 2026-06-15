// SPDX-License-Identifier: Apache-2.0
// world_to_map: render a world YAML to a ROS map_server PGM + YAML pair.
//
// Offline utility. It loads the ground-truth world (the same World::from_yaml
// the simulator uses) and writes a fully-known occupancy map (occupied/free,
// no unknown cells) via amr_mapping::save_map. This gives the NAV stack
// (map_publisher + MCL + navigator) a deterministic, complete map to localise
// and plan on, independent of live-SLAM map quality.
//
// Usage:  world_to_map <world.yaml> <out_stem>
//   writes <out_stem>.pgm and <out_stem>.yaml
#include <cstdio>
#include <exception>
#include <string>

#include "amr_mapping/map_io.hpp"
#include "amr_sim/world.hpp"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: world_to_map <world.yaml> <out_stem>\n");
    return 2;
  }
  const std::string world_path = argv[1];
  const std::string out_stem = argv[2];
  try {
    const amr_sim::World world = amr_sim::World::from_yaml(world_path);
    const auto paths = amr_mapping::save_map(world.grid(), out_stem);
    std::printf("world_to_map: wrote %s and %s (%dx%d @ %.3f m/cell)\n",
                paths.first.c_str(), paths.second.c_str(), world.grid().cols,
                world.grid().rows, world.grid().resolution);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "world_to_map: failed: %s\n", e.what());
    return 1;
  }
  return 0;
}
