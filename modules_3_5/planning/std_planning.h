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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/common/frame.h"
#include "modules/planning/planner/std_planner_dispatcher.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/tasks/smoothers/smoother.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class StdPlanning : public PlanningBase {
 public:
  StdPlanning() {
    planner_dispatcher_ = std::make_unique<StdPlannerDispatcher>();
  }
  virtual ~StdPlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const trajectory_pb) override;

  apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

 private:
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const double start_time,
                           const common::VehicleState& vehicle_state,
                           ADCTrajectory* output_trajectory);
  void ExportReferenceLineDebug(planning_internal::Debug* debug);
  bool CheckPlanningConfig(const PlanningConfig& config);
  void GenerateStopTrajectory(ADCTrajectory* trajectory_pb);

 private:
  routing::RoutingResponse last_routing_;
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  Smoother planning_smoother_;
};

}  // namespace planning
}  // namespace apollo
