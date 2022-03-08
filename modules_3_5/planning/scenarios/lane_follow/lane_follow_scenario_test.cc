/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/
#define protected public
#define private public
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace lane_follow {

class LaneFollowScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<LaneFollowScenario> scenario_;
};

TEST_F(LaneFollowScenarioTest, VerifyConf) {
  FLAGS_scenario_lane_follow_config_file =
      "/apollo/modules/planning/conf/scenario_lane_follow_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_side_pass_config_file, &config));
}

TEST_F(LaneFollowScenarioTest, Init) {
  FLAGS_scenario_lane_follow_config_file =
      "/apollo/modules/planning/testdata/conf/"
      "scenario_lane_follow_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_lane_follow_config_file, &config));
  ScenarioContext context;
  scenario_.reset(new LaneFollowScenario(config, &context));
  EXPECT_EQ(scenario_->scenario_type(), ScenarioConfig::LANE_FOLLOW);
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
