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

#pragma once

#include <string>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/scenarios/stage.h"
#include "modules/planning/scenarios/traffic_light/right_turn_unprotected/traffic_light_right_turn_unprotected_scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

struct TrafficLightRightTurnUnprotectedContext;

class TrafficLightRightTurnUnprotectedStop : public Stage {
 public:
  explicit TrafficLightRightTurnUnprotectedStop(
      const ScenarioConfig::StageConfig& config) : Stage(config) {}

 private:
  Stage::StageStatus Process(
      const common::TrajectoryPoint& planning_init_point,
      Frame* frame) override;
  TrafficLightRightTurnUnprotectedContext* GetContext() {
    return GetContextAs<TrafficLightRightTurnUnprotectedContext>();
  }

 private:
  ScenarioTrafficLightRightTurnUnprotectedConfig scenario_config_;
};

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
