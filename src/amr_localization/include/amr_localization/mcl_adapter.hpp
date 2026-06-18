// SPDX-License-Identifier: Apache-2.0
// MclAdapter: implements amr_api::ILocalizer by delegating to
// amr_localization::MonteCarloLocalizer. Owns the seeded RNG (the interface is
// pure-data; the adapter holds the determinism seam) and bridges the
// predict/correct/estimate filter to a single update() tick.
//
// Non-copyable / non-movable: MonteCarloLocalizer stores a reference to this
// object's rng_ member, so the adapter must keep a stable address. Hold it via
// std::unique_ptr at call sites.
#pragma once

#include <optional>
#include <random>

#include "amr_api/localizer.hpp"
#include "amr_core/config.hpp"
#include "amr_core/types.hpp"
#include "amr_localization/mcl.hpp"

namespace amr_localization {

class MclAdapter : public amr_api::ILocalizer {
 public:
  MclAdapter(const amr_core::OccupancyGrid& grid,
             const amr_core::LocalizationConfig& cfg, unsigned int seed,
             std::optional<amr_core::Pose2D> initial_pose = std::nullopt)
      : rng_(seed), mcl_(grid, cfg, rng_, initial_pose) {}

  MclAdapter(const MclAdapter&) = delete;
  MclAdapter& operator=(const MclAdapter&) = delete;
  MclAdapter(MclAdapter&&) = delete;
  MclAdapter& operator=(MclAdapter&&) = delete;

  amr_api::LocalizerOutput update(const amr_api::LocalizerInput& in) override {
    mcl_.predict(in.odom_delta);
    if (in.scan.has_value()) {
      mcl_.correct(*in.scan);
    }
    amr_api::LocalizerOutput out;
    out.pose = mcl_.estimate();
    out.cloud.poses = mcl_.particles();
    out.cloud.weights = mcl_.weights();
    return out;
  }
  amr_api::Pose2D estimate() const override { return mcl_.estimate(); }
  void set_pose(const amr_api::Pose2D& pose) override { mcl_.set_pose(pose); }

  const MonteCarloLocalizer& raw() const { return mcl_; }

 private:
  std::mt19937 rng_;            // declared before mcl_ so it outlives the ref
  MonteCarloLocalizer mcl_;
};

}  // namespace amr_localization
