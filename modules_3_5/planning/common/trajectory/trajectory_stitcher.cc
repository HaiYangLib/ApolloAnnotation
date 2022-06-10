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

#include "modules/planning/common/trajectory/trajectory_stitcher.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const VehicleState& vehicle_state) {
  TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_s(0.0);
  init_point.mutable_path_point()->set_x(vehicle_state.x());
  init_point.mutable_path_point()->set_y(vehicle_state.y());
  init_point.mutable_path_point()->set_z(vehicle_state.z());
  init_point.mutable_path_point()->set_theta(vehicle_state.heading());
  init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(0.0);

  return std::vector<TrajectoryPoint>(1, init_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new = common::math::WrapAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory, std::string* replan_reason) {
  // DEFINE_bool(enable_trajectory_stitcher, true, 
  //                            "enable stitching trajectory");
  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));

  auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
  auto lat_diff = frenet_sd.second;

  ADEBUG << "Control lateral diff: " << lat_diff
         << ", longitudinal diff: " << lon_diff;

  /**
   * DEFINE_double(replan_lateral_distance_threshold, 0.5,
              "The lateral distance threshold of replan");
   * **/
  if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
    std::string msg(
        "the distance between matched point and actual position is too "
        "large. Replan is triggered. lat_diff = " +
        std::to_string(lat_diff));
    AERROR << msg;
    *replan_reason = msg;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  /**
   * DEFINE_double(replan_longitudinal_distance_threshold, 2.5,
              "The longitudinal distance threshold of replan");
   * **/
  if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
    std::string msg(
        "the distance between matched point and actual position is too "
        "large. Replan is triggered. lon_diff = " +
        std::to_string(lon_diff));
    AERROR << msg;
    *replan_reason = msg;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  double forward_rel_time =
      prev_trajectory
          ->TrajectoryPointAt(static_cast<uint32_t>(time_matched_index))
          .relative_time() +
      planning_cycle_time;

  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;

  auto matched_index = std::min(time_matched_index, position_matched_index);

  constexpr size_t kNumPreCyclePoint = 20;
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - kNumPreCyclePoint)),
      prev_trajectory->begin() + forward_time_index + 1);

  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();

  const double zero_s = stitching_trajectory.back().path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    // tp.relative_time() + prev_trajectory->header_time()=绝对时间
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(common::math::cos(
              common::math::Angle16::from_rad(p.path_point().theta())),
          common::math::sin(
              common::math::Angle16::from_rad(p.path_point().theta())));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
