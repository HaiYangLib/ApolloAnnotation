/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file dp_st_graph.h
 **/

#pragma once

#include <memory>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/tasks/optimizers/dp_st_speed/dp_st_cost.h"
#include "modules/planning/tasks/optimizers/dp_st_speed/st_graph_point.h"
#include "modules/planning/tasks/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class DpStGraph {
 public:
  DpStGraph(const StGraphData& st_graph_data, const DpStSpeedConfig& dp_config,
            const std::vector<const Obstacle*>& obstacles,
            const common::TrajectoryPoint& init_point,
            const SLBoundary& adc_sl_boundary);

  apollo::common::Status Search(SpeedData* const speed_data);

 private:
  apollo::common::Status InitCostTable();

  apollo::common::Status RetrieveSpeedProfile(SpeedData* const speed_data);

  apollo::common::Status CalculateTotalCost();

  // defined for cyber task
  struct StGraphMessage {
    StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
    uint32_t c;
    uint32_t r;
  };
  void CalculateCostAt(const std::shared_ptr<StGraphMessage>& msg);

  double CalculateEdgeCost(const STPoint& first, const STPoint& second,
                          const STPoint& third, const STPoint& forth,
                          const double speed_limit);
  double CalculateEdgeCostForSecondCol(const uint32_t row,
                                      const double speed_limit);
  double CalculateEdgeCostForThirdCol(const uint32_t curr_r,
                                     const uint32_t pre_r,
                                     const double speed_limit);

  void GetRowRange(const StGraphPoint& point, size_t* highest_row,
                   size_t* lowest_row);

 private:
  const StGraphData& st_graph_data_;

  // dp st configuration
  DpStSpeedConfig dp_st_speed_config_;

  // obstacles based on the current reference line
  const std::vector<const Obstacle*>& obstacles_;

  // vehicle configuration parameter
  const common::VehicleParam& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // initial status
  common::TrajectoryPoint init_point_;

  // cost utility with configuration;
  DpStCost dp_st_cost_;

  const SLBoundary& adc_sl_boundary_;

  double unit_s_ = 0.0;
  double unit_t_ = 0.0;

  // cost_table_[t][s]
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<StGraphPoint>> cost_table_;
};

}  // namespace planning
}  // namespace apollo
