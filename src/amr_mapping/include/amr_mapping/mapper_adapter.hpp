// SPDX-License-Identifier: Apache-2.0
// OccupancyGridMapperAdapter: implements amr_api::IMapper by delegating to
// amr_mapping::OccupancyGridMapper.
#pragma once

#include <utility>

#include "amr_api/mapper.hpp"
#include "amr_core/config.hpp"
#include "amr_mapping/occupancy_grid_mapper.hpp"

namespace amr_mapping {

class OccupancyGridMapperAdapter : public amr_api::IMapper {
 public:
  OccupancyGridMapperAdapter(const amr_core::MappingConfig& cfg,
                             std::pair<double, double> size_m,
                             std::pair<double, double> origin_xy = {0.0, 0.0})
      : mapper_(cfg, size_m, origin_xy) {}

  void integrate(const amr_api::MapperInput& in) override {
    mapper_.update(in.pose, in.scan);
  }
  amr_api::OccupancyGrid map() const override {
    return mapper_.to_occupancy_grid();
  }

  const OccupancyGridMapper& raw() const { return mapper_; }

 private:
  OccupancyGridMapper mapper_;
};

}  // namespace amr_mapping
