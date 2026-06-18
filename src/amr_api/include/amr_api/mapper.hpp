// SPDX-License-Identifier: Apache-2.0
// IMapper: standardized occupancy mapping interface. Integrates scans taken at a
// known pose and exposes the resulting grid.
#pragma once

#include "amr_api/types.hpp"

namespace amr_api {

struct MapperInput {
  Pose2D pose;
  LaserScan scan;
};

class IMapper {
 public:
  virtual ~IMapper() = default;

  /// Integrate one scan taken at `in.pose`.
  virtual void integrate(const MapperInput& in) = 0;

  /// Snapshot the occupancy map.
  virtual OccupancyGrid map() const = 0;
};

}  // namespace amr_api
