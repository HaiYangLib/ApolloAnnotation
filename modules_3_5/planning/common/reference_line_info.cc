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

#include "modules/planning/common/reference_line_info.h"

#include "cyber/task/task.h"
#include "modules/planning/proto/sl_boundary.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::common::EngageAdvice;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleSignal;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& segments)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),
      lanes_(segments) {}

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
  // stitching point
  const auto& path_point = adc_planning_point_.path_point();
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), param.length(), param.width());
  // realtime vehicle position
  Vec2d vehicle_position(vehicle_state_.x(), vehicle_state_.y());
  Vec2d vehicle_center(vehicle_position +
                       vec_to_center.rotate(vehicle_state_.heading()));
  Box2d vehicle_box(vehicle_center, vehicle_state_.heading(), param.length(),
                    param.width());

  if (!reference_line_.GetSLBoundary(vehicle_box,
                                     &sl_boundary_info_.vehicle_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from vehicle_box(realtime position "
              "of the car): "
           << box.DebugString();
    return false;
  }

  if (!reference_line_.GetSLBoundary(box,
                                     &sl_boundary_info_.adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }

  InitFirstOverlaps();

  if (sl_boundary_info_.adc_sl_boundary_.end_s() < 0 ||
      sl_boundary_info_.adc_sl_boundary_.start_s() > reference_line_.Length()) {
    AWARN << "Vehicle SL "
          << sl_boundary_info_.adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.Length()
          << "]";
  }
  constexpr double kOutOfReferenceLineL = 10.0;  // in meters
  if (sl_boundary_info_.adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      sl_boundary_info_.adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Ego vehicle is too far away from reference line.";
    return false;
  }
  is_on_reference_line_ =
      reference_line_.IsOnLane(sl_boundary_info_.adc_sl_boundary_);
  if (!AddObstacles(obstacles)) {
    AERROR << "Failed to add obstacles to reference line";
    return false;
  }

  const auto& map_path = reference_line_.map_path();
  for (const auto& speed_bump : map_path.speed_bump_overlaps()) {
    // -1 and + 1.0 are added to make sure it can be sampled.
    reference_line_.AddSpeedLimit(speed_bump.start_s - 1.0,
                                  speed_bump.end_s + 1.0,
                                  FLAGS_speed_bump_speed_limit);
  }

  // set lattice planning target speed limit;
  SetCruiseSpeed(FLAGS_default_cruise_speed);
  is_safe_to_change_lane_ = CheckChangeLane();
  is_inited_ = true;
  return true;
}

bool ReferenceLineInfo::GetFirstOverlap(
    const std::vector<hdmap::PathOverlap>& path_overlaps,
    hdmap::PathOverlap* path_overlap) {
  CHECK_NOTNULL(path_overlap);
  const double start_s = sl_boundary_info_.adc_sl_boundary_.end_s();
  constexpr double kMaxOverlapRange = 500.0;
  double overlap_min_s = kMaxOverlapRange;

  for (const auto& overlap : path_overlaps) {
    if (overlap.end_s < start_s) {
      continue;
    }
    if (overlap_min_s > overlap.start_s) {
      *path_overlap = overlap;
      overlap_min_s = overlap.start_s;
    }
  }
  return overlap_min_s < kMaxOverlapRange;
}

void ReferenceLineInfo::InitFirstOverlaps() {
  const auto& map_path = reference_line_.map_path();

  // crosswalk
  hdmap::PathOverlap crosswalk_overlap;
  if (GetFirstOverlap(map_path.crosswalk_overlaps(), &crosswalk_overlap)) {
    first_encounter_overlaps_.push_back({CROSSWALK, crosswalk_overlap});
  }

  // signal
  hdmap::PathOverlap signal_overlap;
  if (GetFirstOverlap(map_path.signal_overlaps(), &signal_overlap)) {
    first_encounter_overlaps_.push_back({SIGNAL, signal_overlap});
  }

  // stop_sign
  hdmap::PathOverlap stop_sign_overlap;
  if (GetFirstOverlap(map_path.stop_sign_overlaps(), &stop_sign_overlap)) {
    first_encounter_overlaps_.push_back({STOP_SIGN, stop_sign_overlap});
  }

  // clear_zone
  hdmap::PathOverlap clear_area_overlap;
  if (GetFirstOverlap(map_path.clear_area_overlaps(), &clear_area_overlap)) {
    first_encounter_overlaps_.push_back({CLEAR_AREA, clear_area_overlap});
  }

  // pnc_junction
  hdmap::PathOverlap pnc_junction_overlap;
  if (GetFirstOverlap(map_path.pnc_junction_overlaps(),
                      &pnc_junction_overlap)) {
    first_encounter_overlaps_.push_back({PNC_JUNCTION, pnc_junction_overlap});
  }
}

bool ReferenceLineInfo::IsInited() const { return is_inited_; }

bool WithinOverlap(const hdmap::PathOverlap& overlap, double s) {
  constexpr double kEpsilon = 1e-2;
  return overlap.start_s - kEpsilon <= s && s <= overlap.end_s + kEpsilon;
}

void ReferenceLineInfo::SetJunctionRightOfWay(double junction_s,
                                              bool is_protected) {
  auto* right_of_way =
      PlanningContext::MutablePlanningStatus()->mutable_right_of_way();
  auto* junction_right_of_way = right_of_way->mutable_junction();
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (WithinOverlap(overlap, junction_s)) {
      (*junction_right_of_way)[overlap.object_id] = is_protected;
    }
  }
}

ADCTrajectory::RightOfWayStatus ReferenceLineInfo::GetRightOfWayStatus() const {
  auto* right_of_way =
      PlanningContext::MutablePlanningStatus()->mutable_right_of_way();
  auto* junction_right_of_way = right_of_way->mutable_junction();
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (overlap.end_s < sl_boundary_info_.adc_sl_boundary_.start_s()) {
      junction_right_of_way->erase(overlap.object_id);
    } else if (WithinOverlap(overlap,
                             sl_boundary_info_.adc_sl_boundary_.end_s())) {
      auto is_protected = (*junction_right_of_way)[overlap.object_id];
      if (is_protected) {
        return ADCTrajectory::PROTECTED;
      }
    }
  }
  return ADCTrajectory::UNPROTECTED;
}

bool ReferenceLineInfo::CheckChangeLane() const {
  if (!IsChangeLanePath()) {
    ADEBUG << "Not a change lane path.";
    return false;
  }

  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& sl_boundary = obstacle->PerceptionSLBoundary();

    constexpr float kLateralShift = 2.5f;
    if (sl_boundary.start_l() < -kLateralShift ||
        sl_boundary.end_l() > kLateralShift) {
      continue;
    }
    constexpr double kChangeLaneIgnoreDistance = 50.0;
    if ((obstacle->IsVirtual() || obstacle->IsStatic()) &&
        sl_boundary.start_s() < sl_boundary_info_.adc_sl_boundary_.end_s() +
                                    kChangeLaneIgnoreDistance &&
        sl_boundary.start_s() > sl_boundary_info_.adc_sl_boundary_.end_s()) {
      return false;
    }

    constexpr float kSafeTime = 3.0f;
    constexpr float kForwardMinSafeDistance = 6.0f;
    constexpr float kBackwardMinSafeDistance = 8.0f;

    const float kForwardSafeDistance = std::max(
        kForwardMinSafeDistance,
        static_cast<float>((adc_planning_point_.v() - obstacle->speed()) *
                           kSafeTime));
    const float kBackwardSafeDistance = std::max(
        kBackwardMinSafeDistance,
        static_cast<float>((obstacle->speed() - adc_planning_point_.v()) *
                           kSafeTime));
    if (sl_boundary.end_s() > sl_boundary_info_.adc_sl_boundary_.start_s() -
                                  kBackwardSafeDistance &&
        sl_boundary.start_s() <
            sl_boundary_info_.adc_sl_boundary_.end_s() + kForwardSafeDistance) {
      return false;
    }
  }
  return true;
}

const hdmap::RouteSegments& ReferenceLineInfo::Lanes() const { return lanes_; }

const std::list<hdmap::Id> ReferenceLineInfo::TargetLaneId() const {
  std::list<hdmap::Id> lane_ids;
  for (const auto& lane_seg : lanes_) {
    lane_ids.push_back(lane_seg.lane->id());
  }
  return lane_ids;
}

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return sl_boundary_info_.adc_sl_boundary_;
}

const SLBoundary& ReferenceLineInfo::VehicleSlBoundary() const {
  return sl_boundary_info_.vehicle_sl_boundary_;
}

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

bool ReferenceLineInfo::AddObstacleHelper(
    const std::shared_ptr<Obstacle>& obstacle) {
  return AddObstacle(obstacle.get()) != nullptr;
}

// AddObstacle is thread safe
Obstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    AERROR << "The provided obstacle is empty";
    return nullptr;
  }
  auto* mutable_obstacle = path_decision_.AddObstacle(*obstacle);
  if (!mutable_obstacle) {
    AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),
                                     &perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
    return mutable_obstacle;
  }
  mutable_obstacle->SetPerceptionSlBoundary(perception_sl);
  mutable_obstacle->CheckLaneBlocking(reference_line_);
  if (obstacle->IsLaneBlocking()) {
    ADEBUG << "obstacle [" << obstacle->Id() << "] is lane blocking.";
  } else {
    ADEBUG << "obstacle [" << obstacle->Id() << "] is NOT lane blocking.";
  }

  if (IsUnrelaventObstacle(mutable_obstacle)) {
    ObjectDecisionType ignore;
    ignore.mutable_ignore();
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",
                                           obstacle->Id(), ignore);
    ADEBUG << "NO build reference line st boundary. id:" << obstacle->Id();
  } else {
    ADEBUG << "build reference line st boundary. id:" << obstacle->Id();
    mutable_obstacle->BuildReferenceLineStBoundary(
        reference_line_, sl_boundary_info_.adc_sl_boundary_.start_s());

    ADEBUG << "reference line st boundary: t["
           << mutable_obstacle->reference_line_st_boundary().min_t() << ", "
           << mutable_obstacle->reference_line_st_boundary().max_t() << "] s["
           << mutable_obstacle->reference_line_st_boundary().min_s() << ", "
           << mutable_obstacle->reference_line_st_boundary().max_s() << "]";
  }
  return mutable_obstacle;
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  if (FLAGS_use_multi_thread_to_add_obstacles) {
    std::vector<std::future<Obstacle*>> results;
    for (const auto* obstacle : obstacles) {
      results.push_back(
          cyber::Async(&ReferenceLineInfo::AddObstacle, this, obstacle));
    }
    for (auto& result : results) {
      if (!result.get()) {
        AERROR << "Fail to add obstacles.";
        return false;
      }
    }
  } else {
    for (const auto* obstacle : obstacles) {
      if (!AddObstacle(obstacle)) {
        AERROR << "Failed to add obstacle " << obstacle->Id();
        return false;
      }
    }
  }

  return true;
}

bool ReferenceLineInfo::IsUnrelaventObstacle(const Obstacle* obstacle) {
  // if adc is on the road, and obstacle behind adc, ignore
  if (obstacle->PerceptionSLBoundary().end_s() > reference_line_.Length()) {
    return true;
  }
  if (is_on_reference_line_ &&
      obstacle->PerceptionSLBoundary().end_s() <
          sl_boundary_info_.adc_sl_boundary_.end_s() &&
      reference_line_.IsOnLane(obstacle->PerceptionSLBoundary())) {
    return true;
  }
  return false;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

double ReferenceLineInfo::TrajectoryLength() const {
  if (discretized_trajectory_.empty()) {
    return 0.0;
  }
  return discretized_trajectory_.back().path_point().s();
}

void ReferenceLineInfo::SetStopPoint(const StopPoint& stop_point) {
  planning_target_.mutable_stop_point()->CopyFrom(stop_point);
}

void ReferenceLineInfo::SetCruiseSpeed(double speed) {
  planning_target_.set_cruise_speed(speed);
}

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.IsOnLane(sl_point);
}

const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

const RSSInfo& ReferenceLineInfo::rss_info() const { return rss_info_; }

RSSInfo* ReferenceLineInfo::mutable_rss_info() { return &rss_info_; }

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  CHECK(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;
  if (path_data_.discretized_path().size() == 0) {
    AWARN << "path data is empty";
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;
    }
    common::PathPoint path_point;
    if (!path_data_.GetPathPointWithPathS(speed_point.s(), &path_point)) {
      AERROR << "Fail to get path data with s " << speed_point.s()
             << "path total length " << path_data_.discretized_path().Length();
      return false;
    }
    path_point.set_s(path_point.s() + start_s);

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }

bool ReferenceLineInfo::IsChangeLanePath() const {
  return !Lanes().IsOnSegment();
}

bool ReferenceLineInfo::IsNeighborLanePath() const {
  return Lanes().IsNeighborSegment();
}

std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return apollo::common::util::StrCat("path_data:", path_data_.DebugString(),
                                      "speed_data:", speed_data_.DebugString());
}

void ReferenceLineInfo::ExportTurnSignal(VehicleSignal* signal) const {
  // set vehicle change lane signal
  CHECK_NOTNULL(signal);

  signal->Clear();
  signal->set_turn_signal(VehicleSignal::TURN_NONE);
  if (IsChangeLanePath()) {
    if (Lanes().PreviousAction() == routing::ChangeLaneType::LEFT) {
      signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    } else if (Lanes().PreviousAction() == routing::ChangeLaneType::RIGHT) {
      signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    }
    return;
  }
  // check lane's turn type
  double route_s = 0.0;
  const double adc_s = sl_boundary_info_.adc_sl_boundary_.end_s();
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + FLAGS_turn_signal_distance) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::LEFT_TURN) {
      signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      break;
    } else if (turn == hdmap::Lane::RIGHT_TURN) {
      signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      break;
    } else if (turn == hdmap::Lane::U_TURN) {
      // check left or right by geometry.
      auto start_xy =
          common::util::MakeVec2d(seg.lane->GetSmoothPoint(seg.start_s));
      auto middle_xy = common::util::MakeVec2d(
          seg.lane->GetSmoothPoint((seg.start_s + seg.end_s) / 2.0));
      auto end_xy =
          common::util::MakeVec2d(seg.lane->GetSmoothPoint(seg.end_s));
      auto start_to_middle = middle_xy - start_xy;
      auto start_to_end = end_xy - start_xy;
      if (start_to_middle.CrossProd(start_to_end) < 0) {
        signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      } else {
        signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      }
      break;
    }
  }
}

bool ReferenceLineInfo::IsRightTurnPath(const double forward_buffer) const {
  double route_s = 0.0;
  const double adc_s = sl_boundary_info_.adc_sl_boundary_.end_s();
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + forward_buffer) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::RIGHT_TURN) {
      return true;
    }
  }
  return false;
}

bool ReferenceLineInfo::ReachedDestination() const {
  constexpr double kDestinationDeltaS = 0.05;
  return SDistanceToDestination() <= kDestinationDeltaS;
}

double ReferenceLineInfo::SDistanceToDestination() const {
  double res = std::numeric_limits<double>::max();
  const auto* dest_ptr = path_decision_.Find(FLAGS_destination_obstacle_id);
  if (!dest_ptr) {
    return res;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {
    return res;
  }
  if (!reference_line_.IsOnLane(dest_ptr->PerceptionBoundingBox().center())) {
    return res;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +
                        dest_ptr->LongitudinalDecision().stop().distance_s();
  return stop_s - sl_boundary_info_.adc_sl_boundary_.end_s();
}

void ReferenceLineInfo::ExportDecision(DecisionResult* decision_result) const {
  MakeDecision(decision_result);
  ExportTurnSignal(decision_result->mutable_vehicle_signal());
  auto* main_decision = decision_result->mutable_main_decision();
  if (main_decision->has_stop()) {
    main_decision->mutable_stop()->set_change_lane_type(
        Lanes().PreviousAction());
  } else if (main_decision->has_cruise()) {
    main_decision->mutable_cruise()->set_change_lane_type(
        Lanes().PreviousAction());
  }
}

void ReferenceLineInfo::MakeDecision(DecisionResult* decision_result) const {
  CHECK_NOTNULL(decision_result);
  decision_result->Clear();

  // cruise by default
  decision_result->mutable_main_decision()->mutable_cruise();

  // check stop decision
  int error_code = MakeMainStopDecision(decision_result);
  if (error_code < 0) {
    MakeEStopDecision(decision_result);
  }
  MakeMainMissionCompleteDecision(decision_result);
  SetObjectDecisions(decision_result->mutable_object_decision());
}

void ReferenceLineInfo::MakeMainMissionCompleteDecision(
    DecisionResult* decision_result) const {
  if (!decision_result->main_decision().has_stop()) {
    return;
  }
  auto main_stop = decision_result->main_decision().stop();
  if (main_stop.reason_code() != STOP_REASON_DESTINATION) {
    return;
  }
  const auto& adc_pos = adc_planning_point_.path_point();
  if (common::util::DistanceXY(adc_pos, main_stop.stop_point()) >
      FLAGS_destination_check_distance) {
    return;
  }

  auto mission_complete =
      decision_result->mutable_main_decision()->mutable_mission_complete();
  if (ReachedDestination()) {
    PlanningContext::MutablePlanningStatus()
        ->mutable_destination()
        ->set_has_passed_destination(true);
  } else {
    mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
    mission_complete->set_stop_heading(main_stop.stop_heading());
  }
}

int ReferenceLineInfo::MakeMainStopDecision(
    DecisionResult* decision_result) const {
  double min_stop_line_s = std::numeric_limits<double>::infinity();
  const Obstacle* stop_obstacle = nullptr;
  const ObjectStop* stop_decision = nullptr;

  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }

    apollo::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL({stop_point.x(), stop_point.y()}, &stop_line_sl);

    double stop_line_s = stop_line_sl.s();
    if (stop_line_s < 0 || stop_line_s > reference_line_.Length()) {
      AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line_.Length()
             << "]";
      continue;
    }

    // check stop_line_s vs adc_s
    if (stop_line_s < min_stop_line_s) {
      min_stop_line_s = stop_line_s;
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop());
    }
  }

  if (stop_obstacle != nullptr) {
    MainStop* main_stop =
        decision_result->mutable_main_decision()->mutable_stop();
    main_stop->set_reason_code(stop_decision->reason_code());
    main_stop->set_reason("stop by " + stop_obstacle->Id());
    main_stop->mutable_stop_point()->set_x(stop_decision->stop_point().x());
    main_stop->mutable_stop_point()->set_y(stop_decision->stop_point().y());
    main_stop->set_stop_heading(stop_decision->stop_heading());

    ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point().x() << stop_decision->stop_point().y()
           << " ) stop_heading: " << stop_decision->stop_heading();

    return 1;
  }

  return 0;
}

void ReferenceLineInfo::SetObjectDecisions(
    ObjectDecisions* object_decisions) const {
  for (const auto obstacle : path_decision_.obstacles().Items()) {
    if (!obstacle->HasNonIgnoreDecision()) {
      continue;
    }
    auto* object_decision = object_decisions->add_decision();

    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    if (obstacle->HasLateralDecision() && !obstacle->IsLateralIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LateralDecision());
    }
    if (obstacle->HasLongitudinalDecision() &&
        !obstacle->IsLongitudinalIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LongitudinalDecision());
    }
  }
}

void ReferenceLineInfo::ExportEngageAdvice(EngageAdvice* engage_advice) const {
  constexpr double kMaxAngleDiff = M_PI / 6.0;
  auto* prev_advice =
      PlanningContext::MutablePlanningStatus()->mutable_engage_advice();
  if (!prev_advice->has_advice()) {
    prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
  }
  if (!IsDrivable()) {
    if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
    } else {
      prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
    prev_advice->set_reason("Reference line not drivable");
  } else if (!is_on_reference_line_) {
    if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
    } else {
      prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
    prev_advice->set_reason("Not on reference line");
  } else {
    // check heading
    auto ref_point = reference_line_.GetReferencePoint(
        sl_boundary_info_.adc_sl_boundary_.end_s());
    if (common::math::AngleDiff(vehicle_state_.heading(), ref_point.heading()) >
        kMaxAngleDiff) {
      if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
        prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
      } else {
        prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
      }
      prev_advice->set_reason("Vehicle heading is not aligned");
    } else {
      if (vehicle_state_.driving_mode() !=
          Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
        prev_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
      } else {
        prev_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
      }
      prev_advice->clear_reason();
    }
  }
  engage_advice->CopyFrom(*prev_advice);
}

void ReferenceLineInfo::MakeEStopDecision(
    DecisionResult* decision_result) const {
  decision_result->Clear();

  MainEmergencyStop* main_estop =
      decision_result->mutable_main_decision()->mutable_estop();
  main_estop->set_reason_code(MainEmergencyStop::ESTOP_REASON_INTERNAL_ERR);
  main_estop->set_reason("estop reason to be added");
  main_estop->mutable_cruise_to_stop();

  // set object decisions
  ObjectDecisions* object_decisions =
      decision_result->mutable_object_decision();
  for (const auto obstacle : path_decision_.obstacles().Items()) {
    auto* object_decision = object_decisions->add_decision();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    object_decision->add_object_decision()->mutable_avoid();
  }
}
}  // namespace planning
}  // namespace apollo
