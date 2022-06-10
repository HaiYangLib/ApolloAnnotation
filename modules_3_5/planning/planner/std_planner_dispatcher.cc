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

#include "modules/planning/planner/std_planner_dispatcher.h"

#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

std::unique_ptr<Planner> StdPlannerDispatcher::DispatchPlanner() {
  PlanningConfig planning_config;
  /**
   * DEFINE_string(planning_config_file,
              "/apollo/modules/planning/conf/planning_config.pb.txt",
              "planning config file");
   * **/
  apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                         &planning_config);
  /**
   * DEFINE_bool(open_space_planner_switchable, false,
            "true for std planning being able to switch to open space planner "
            "when close enough to target parking spot");
   * **/   

  /**
   * standard_planning_config {
      planner_type: PUBLIC_ROAD
      planner_type: OPEN_SPACE
      planner_public_road_config {
        scenario_type: LANE_FOLLOW
        scenario_type: SIDE_PASS
        scenario_type: STOP_SIGN_UNPROTECTED
      }
    }
   * **/                                    
  if (FLAGS_open_space_planner_switchable) {
    return planner_factory_.CreateObject(
        planning_config.standard_planning_config().planner_type(1));
  }
  return planner_factory_.CreateObject(
      planning_config.standard_planning_config().planner_type(0));
}

}  // namespace planning
}  // namespace apollo
