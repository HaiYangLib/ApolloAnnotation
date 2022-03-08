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
#include "modules/planning/scenarios/traffic_light/right_turn_unprotected/traffic_light_right_turn_unprotected_scenario.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

class TrafficLightRightTurnUnprotectedScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<TrafficLightRightTurnUnprotectedScenario> scenario_;
};

TEST_F(TrafficLightRightTurnUnprotectedScenarioTest, VerifyConf) {
  FLAGS_scenario_stop_sign_unprotected_config_file =
      "/apollo/modules/planning/conf/"
      "scenario_traffic_light_right_turn_unprotected_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_traffic_light_right_turn_unprotected_config_file,
      &config));
}

TEST_F(TrafficLightRightTurnUnprotectedScenarioTest, Init) {
  FLAGS_scenario_stop_sign_unprotected_config_file =
      "/apollo/modules/planning/testdata/conf/"
      "scenario_traffic_light_right_turn_unprotected_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_traffic_light_right_turn_unprotected_config_file,
      &config));

  ScenarioContext context;
  scenario_.reset(new TrafficLightRightTurnUnprotectedScenario(config,
                                                               &context));
  EXPECT_EQ(scenario_->scenario_type(),
            ScenarioConfig::TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED);
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
