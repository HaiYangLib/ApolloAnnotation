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
 * @file
 **/

#include "modules/planning/tasks/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/optimizers/qp_spline_st_speed/qp_piecewise_st_graph.h"
#include "modules/planning/tasks/optimizers/qp_spline_st_speed/qp_spline_st_graph.h"
#include "modules/planning/tasks/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::STGraphDebug;

QpSplineStSpeedOptimizer::QpSplineStSpeedOptimizer(const TaskConfig& config)
    : SpeedOptimizer(config) {
  SetName("QpSplineStSpeedOptimizer");
  CHECK(config.has_qp_st_speed_config());
  qp_st_speed_config_ = config.qp_st_speed_config();
  st_boundary_config_ = qp_st_speed_config_.st_boundary_config();
  std::vector<double> init_knots;

  if (FLAGS_use_osqp_optimizer_for_qp_st) {
    spline_solver_.reset(new OsqpSpline1dSolver(init_knots, 5));
  } else {
    spline_solver_.reset(new ActiveSetSpline1dSolver(init_knots, 5));
  }
}

Status QpSplineStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                         const PathData& path_data,
                                         const TrajectoryPoint& init_point,
                                         const ReferenceLine& reference_line,
                                         const SpeedData& reference_speed_data,
                                         PathDecision* const path_decision,
                                         SpeedData* const speed_data) {
  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }

  if (path_data.discretized_path().empty()) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  StBoundaryMapper boundary_mapper(
      adc_sl_boundary, st_boundary_config_, reference_line, path_data,
      qp_st_speed_config_.total_path_length(), qp_st_speed_config_.total_time(),
      reference_line_info_->IsChangeLanePath());

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    DCHECK(obstacle->HasLongitudinalDecision());
  }
  // step 1 get boundaries
  path_decision->EraseStBoundaries();
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Mapping obstacle for qp st speed optimizer failed!");
  }

  std::vector<const StBoundary*> boundaries;
  for (auto* obstacle : path_decision->obstacles().Items()) {
    auto id = obstacle->Id();
    if (!obstacle->st_boundary().IsEmpty()) {
      path_decision->Find(id)->SetBlockingObstacle(true);
      boundaries.push_back(&obstacle->st_boundary());
    }
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        reference_line, path_data);
  SpeedLimit speed_limits;
  if (speed_limit_decider.GetSpeedLimits(path_decision->obstacles(),
                                         &speed_limits) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "GetSpeedLimits for qp st speed optimizer failed!");
  }

  // step 2 perform graph search
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  QpSplineStGraph st_graph(spline_solver_.get(), qp_st_speed_config_, veh_param,
                           reference_line_info_->IsChangeLanePath());

  StGraphData st_graph_data(boundaries, init_point, speed_limits,
                            path_data.discretized_path().Length());

  STGraphDebug* st_graph_debug = reference_line_info_->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();

  std::pair<double, double> accel_bound = {
      qp_st_speed_config_.preferred_min_deceleration(),
      qp_st_speed_config_.preferred_max_acceleration()};
  st_graph.SetDebugLogger(st_graph_debug);
  auto ret = st_graph.Search(st_graph_data, accel_bound, reference_speed_data,
                             speed_data);
  if (ret != Status::OK()) {
    AERROR << "Failed to solve with ideal acceleration conditions. Use "
              "secondary choice instead.";

    accel_bound.first = qp_st_speed_config_.min_deceleration();
    accel_bound.second = qp_st_speed_config_.max_acceleration();
    ret = st_graph.Search(st_graph_data, accel_bound, reference_speed_data,
                          speed_data);

    // backup plan: use piecewise_st_graph
    if (ret != Status::OK()) {
      AERROR << "Spline QP speed solver Failed. "
             << "Using finite difference method.";
      QpPiecewiseStGraph piecewise_st_graph(qp_st_speed_config_);
      ret = piecewise_st_graph.Search(st_graph_data, speed_data, accel_bound);

      if (ret != Status::OK()) {
        std::string msg = common::util::StrCat(
            Name(), ": Failed to search graph with quadratic programming!");
        AERROR << msg;
        RecordSTGraphDebug(st_graph_data, st_graph_debug);
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    }
  }

  // record debug info
  RecordSTGraphDebug(st_graph_data, st_graph_debug);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
