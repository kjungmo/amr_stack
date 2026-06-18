// SPDX-License-Identifier: Apache-2.0
#include "amr_api/behavior.hpp"

namespace amr_api {

const char* to_string(BehaviorState s) {
  switch (s) {
    case BehaviorState::IDLE: return "IDLE";
    case BehaviorState::PLANNING: return "PLANNING";
    case BehaviorState::FOLLOWING: return "FOLLOWING";
    case BehaviorState::RECOVERY: return "RECOVERY";
    case BehaviorState::SUCCEEDED: return "SUCCEEDED";
    case BehaviorState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace amr_api
