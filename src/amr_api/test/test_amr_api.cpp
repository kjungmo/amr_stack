// SPDX-License-Identifier: Apache-2.0
// Smoke tests for the amr_api interface layer: version constants, BehaviorState
// labels, the no-op logger, and that the pure-data structs default sanely.
#include <gtest/gtest.h>

#include "amr_api/behavior.hpp"
#include "amr_api/diagnostics.hpp"
#include "amr_api/localizer.hpp"
#include "amr_api/version.hpp"

TEST(AmrApi, VersionStrings) {
  EXPECT_STREQ(amr_api::VERSION, "0.1.0");
  EXPECT_STREQ(amr_api::CONTRACT_VERSION, "1.0");
  EXPECT_EQ(amr_api::VERSION_MAJOR, 0);
}

TEST(AmrApi, BehaviorStateLabels) {
  EXPECT_STREQ(amr_api::to_string(amr_api::BehaviorState::FOLLOWING), "FOLLOWING");
  EXPECT_STREQ(amr_api::to_string(amr_api::BehaviorState::SUCCEEDED), "SUCCEEDED");
}

TEST(AmrApi, NullLoggerIsSilent) {
  amr_api::NullLogger log;
  log.log(amr_api::Logger::Level::Info, "ignored");  // must not throw
  SUCCEED();
}

TEST(AmrApi, LocalizerOutputDefaults) {
  amr_api::LocalizerOutput out;
  EXPECT_TRUE(out.cloud.poses.empty());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
