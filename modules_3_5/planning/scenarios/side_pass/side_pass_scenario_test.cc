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
#include "modules/planning/scenarios/side_pass/side_pass_scenario.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

class SidePassScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<SidePassScenario> scenario_;
};

TEST_F(SidePassScenarioTest, VerifyConf) {
  FLAGS_scenario_side_pass_config_file =
      "/apollo/modules/planning/conf/scenario_side_pass_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_side_pass_config_file, &config));
}

TEST_F(SidePassScenarioTest, Init) {
  FLAGS_scenario_side_pass_config_file =
      "/apollo/modules/planning/testdata/conf/scenario_side_pass_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_side_pass_config_file, &config));
  ScenarioContext context;
  scenario_.reset(new SidePassScenario(config, &context));
  EXPECT_EQ(scenario_->scenario_type(), ScenarioConfig::SIDE_PASS);
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
