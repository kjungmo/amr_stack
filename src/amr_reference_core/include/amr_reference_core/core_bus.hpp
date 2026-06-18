// SPDX-License-Identifier: Apache-2.0
// CoreBus: the minimal shared-memory blackboard a host orchestrator owns. It
// holds the latest map / pose / velocity / plan / particle cloud, and provides a
// concrete Logger (stderr) and Clock (monotonic sim seconds) implementing the
// amr_api diagnostics seams. This is the reference template for the private core.
#pragma once

#include <cstdio>
#include <string>

#include "amr_api/diagnostics.hpp"
#include "amr_api/types.hpp"

namespace amr_reference_core {

class StderrLogger : public amr_api::Logger {
 public:
  void log(Level level, const std::string& msg) override {
    const char* tag = "INFO";
    switch (level) {
      case Level::Debug: tag = "DEBUG"; break;
      case Level::Info: tag = "INFO"; break;
      case Level::Warn: tag = "WARN"; break;
      case Level::Error: tag = "ERROR"; break;
    }
    std::fprintf(stderr, "[core][%s] %s\n", tag, msg.c_str());
  }
};

/// Monotonic sim clock advanced by the orchestrator (no wall-clock dependency,
/// so runs are deterministic and replayable).
class SimClock : public amr_api::Clock {
 public:
  double now() const override { return t_; }
  void advance(double dt) { t_ += dt; }
  void set(double t) { t_ = t; }

 private:
  double t_{0.0};
};

/// The shared blackboard: every module reads/writes plain data here.
struct CoreBus {
  amr_api::OccupancyGrid map;
  amr_api::Pose2D pose;
  amr_api::Twist2D vel;
  amr_api::Path plan;
  amr_api::ParticleCloud particles;
  StderrLogger logger;
  SimClock clock;
};

}  // namespace amr_reference_core
