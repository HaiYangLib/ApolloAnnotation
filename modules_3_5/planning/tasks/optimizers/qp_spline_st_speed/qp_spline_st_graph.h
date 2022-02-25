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

#pragma once

#include <utility>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/qp_st_speed_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/math/smoothing_spline/active_set_spline_1d_solver.h"
#include "modules/planning/tasks/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class QpSplineStGraph {
 public:
  QpSplineStGraph(Spline1dSolver* spline_solver,
                  const QpStSpeedConfig& qp_st_speed_config,
                  const apollo::common::VehicleParam& veh_param,
                  const bool is_change_lane);

  void SetDebugLogger(planning_internal::STGraphDebug* st_graph_debug);

  common::Status Search(const StGraphData& st_graph_data,
                        const std::pair<double, double>& accel_bound,
                        const SpeedData& reference_speed_data,
                        SpeedData* const speed_data);

 private:
  void Init();

  // Add st graph constraint
  common::Status AddConstraint(const common::TrajectoryPoint& init_point,
                               const SpeedLimit& speed_limit,
                               const std::vector<const StBoundary*>& boundaries,
                               const std::pair<double, double>& accel_bound);

  // Add objective function
  common::Status AddKernel(const std::vector<const StBoundary*>& boundaries,
                           const SpeedLimit& speed_limit);

  // solve
  common::Status Solve();

  // extract upper lower bound for constraint;
  common::Status GetSConstraintByTime(
      const std::vector<const StBoundary*>& boundaries, const double time,
      const double total_path_s, double* const s_upper_bound,
      double* const s_lower_bound) const;

  // reference line kernel is a constant s line at s = 250m
  common::Status AddCruiseReferenceLineKernel(const double weight);

  // follow line kernel
  common::Status AddFollowReferenceLineKernel(
      const std::vector<const StBoundary*>& boundaries, const double weight);

  // yield line kernel
  common::Status AddYieldReferenceLineKernel(
      const std::vector<const StBoundary*>& boundaries, const double weight);

  const SpeedData GetHistorySpeed() const;
  common::Status EstimateSpeedUpperBound(
      const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
      std::vector<double>* speed_upper_bound) const;

  bool AddDpStReferenceKernel(const double weight) const;

 private:
  // solver
  Spline1dSolver* spline_solver_ = nullptr;

  // qp st configuration
  const QpStSpeedConfig qp_st_speed_config_;

  // initial status
  common::TrajectoryPoint init_point_;

  // is change lane
  bool is_change_lane_ = false;

  // t knots resolution
  double t_knots_resolution_ = 0.0;

  // knots
  std::vector<double> t_knots_;

  // evaluated t resolution
  double t_evaluated_resolution_ = 0.0;

  // evaluated points
  std::vector<double> t_evaluated_;

  // reference line kernel
  std::vector<double> cruise_;

  // reference st points from dp optimizer
  SpeedData reference_dp_speed_points_;

  planning_internal::STGraphDebug* st_graph_debug_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
