// SPDX-License-Identifier: Apache-2.0
// ScanMatchingSlamAdapter: implements amr_api::ISlam by delegating to
// amr_slam::ScanMatchingSlam.
#pragma once

#include <utility>

#include "amr_api/slam.hpp"
#include "amr_core/config.hpp"
#include "amr_slam/scan_matching_slam.hpp"

namespace amr_slam {

class ScanMatchingSlamAdapter : public amr_api::ISlam {
 public:
  ScanMatchingSlamAdapter(const amr_core::SlamConfig& cfg,
                          const amr_core::MappingConfig& mapping_cfg,
                          std::pair<double, double> size_m,
                          const amr_core::Pose2D& initial_pose)
      : slam_(cfg, mapping_cfg, size_m, initial_pose) {}

  amr_api::Pose2D update(const amr_api::SlamInput& in) override {
    return slam_.process(in.odom_delta, in.scan);
  }
  amr_api::Pose2D pose() const override { return slam_.pose(); }
  amr_api::OccupancyGrid map() const override { return slam_.get_map(); }

  const ScanMatchingSlam& raw() const { return slam_; }

 private:
  ScanMatchingSlam slam_;
};

}  // namespace amr_slam
