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

#include "modules/planning/traffic_rules/signal_light.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/time/time.h"
#include "modules/common/util/map_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/traffic_rules/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::time::Clock;
using apollo::common::util::WithinBound;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

SignalLight::SignalLight(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

Status SignalLight::ApplyRule(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  if (!FindValidSignalLight(reference_line_info)) {
    return Status::OK();
  }
  ReadSignals(frame->local_view().traffic_light);
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void SignalLight::ReadSignals(
    const std::shared_ptr<TrafficLightDetection>& traffic_light) {
  detected_signals_.clear();
  if (traffic_light == nullptr) {
    return;
  }
  const double delay =
      traffic_light->header().timestamp_sec() - Clock::NowInSeconds();
  if (delay > config_.signal_light().signal_expire_time_sec()) {
    ADEBUG << "traffic signals msg is expired, delay = " << delay
           << " seconds.";
    return;
  }
  for (int j = 0; j < traffic_light->traffic_light_size(); j++) {
    const TrafficLight& signal = traffic_light->traffic_light(j);
    detected_signals_[signal.id()] = &signal;
  }
}

bool SignalLight::FindValidSignalLight(
    ReferenceLineInfo* const reference_line_info) {
  const std::vector<hdmap::PathOverlap>& signal_lights =
      reference_line_info->reference_line().map_path().signal_overlaps();
  if (signal_lights.size() <= 0) {
    ADEBUG << "No signal lights from reference line.";
    return false;
  }
  signal_lights_from_path_.clear();
  for (const hdmap::PathOverlap& signal_light : signal_lights) {
    if (signal_light.start_s + config_.signal_light().min_pass_s_distance() >
        reference_line_info->AdcSlBoundary().end_s()) {
      signal_lights_from_path_.push_back(signal_light);
    }
  }
  return signal_lights_from_path_.size() > 0;
}

void SignalLight::MakeDecisions(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  planning_internal::SignalLightDebug* signal_light_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_signal_light();
  const double adc_front_edge_s =
      reference_line_info->AdcSlBoundary().end_s();
  signal_light_debug->set_adc_front_s(adc_front_edge_s);
  signal_light_debug->set_adc_speed(
      common::VehicleStateProvider::Instance()->linear_velocity());

  bool has_stop = false;
  for (auto& signal_light : signal_lights_from_path_) {
    const TrafficLight signal = GetSignal(signal_light.object_id);

    double stop_deceleration = util::GetADCStopDeceleration(
        reference_line_info, signal_light.start_s,
        config_.signal_light().min_pass_s_distance());

    // work around incorrect s-projection along round routing
    const double kTrafficLightLookForwardDistanceConfGuard = 135.0;
    const double kTrafficLightDistance = 50.0;
    const double monitor_forward_distance = std::max(
        kTrafficLightLookForwardDistanceConfGuard,
        config_.signal_light().max_monitor_forward_distance());
    if (signal_light.start_s - adc_front_edge_s > monitor_forward_distance) {
      const auto& reference_line = reference_line_info->reference_line();
      common::SLPoint signal_light_sl;
      signal_light_sl.set_s(signal_light.start_s);
      signal_light_sl.set_l(0);
      common::math::Vec2d signal_light_point;
      reference_line.SLToXY(signal_light_sl, &signal_light_point);

      common::math::Vec2d adc_position = {
          common::VehicleStateProvider::Instance()->x(),
          common::VehicleStateProvider::Instance()->y()};

      const double distance = common::util::DistanceXY(
          signal_light_point, adc_position);
      ADEBUG << "traffic_light[" << signal_light.object_id
          << "] start_s[" << signal_light.start_s
          << "] actual_distance[" << distance << "]";
      if (distance < kTrafficLightDistance) {
        ADEBUG << "SKIP traffic_light[" << signal_light.object_id
            << "]. too far away along reference line";
        continue;
      }
    }

    planning_internal::SignalLightDebug::SignalDebug* signal_debug =
        signal_light_debug->add_signal();
    signal_debug->set_adc_stop_deacceleration(stop_deceleration);
    signal_debug->set_color(signal.color());
    signal_debug->set_light_id(signal_light.object_id);
    signal_debug->set_light_stop_s(signal_light.start_s);

    ADEBUG << "traffic_light[" << signal_light.object_id
        << "] start_s[" << signal_light.start_s
        << "] color[" << signal.color()
        << "] stop_deceleration[" << stop_deceleration << "]";
    if ((signal.color() == TrafficLight::RED &&
         stop_deceleration < config_.signal_light().max_stop_deceleration()) ||
        (signal.color() == TrafficLight::UNKNOWN &&
         stop_deceleration < config_.signal_light().max_stop_deceleration()) ||
        (signal.color() == TrafficLight::YELLOW &&
         stop_deceleration <
             config_.signal_light().max_stop_deacceleration_yellow_light())) {
      const double forward_buffer = 1.0;
      if (config_.signal_light().righ_turn_creep().enabled() &&
          reference_line_info->IsRightTurnPath(forward_buffer)) {
        SetCreepForwardSignalDecision(reference_line_info, &signal_light);
      }
      if (BuildStopDecision(frame, reference_line_info, &signal_light)) {
        has_stop = true;
        signal_debug->set_is_stop_wall_created(true);
      }
    }

    // set right_of_way_status
    bool is_protected = true;
    if (has_stop) {
      is_protected = false;
      // protected for YELLOW light with STOP decision
      // at beginning and then passing junction
      if (adc_front_edge_s - signal_light.start_s < 5.0 &&
          signal.color() == TrafficLight::YELLOW) {
        is_protected = true;
      }
    } else {
      is_protected = true;
    }
    reference_line_info->SetJunctionRightOfWay(signal_light.start_s,
                                               is_protected);
  }
}

void SignalLight::SetCreepForwardSignalDecision(
    ReferenceLineInfo* const reference_line_info,
    hdmap::PathOverlap* const signal_light) const {
  CHECK_NOTNULL(signal_light);

  if (EgoInfo::Instance()->start_point().v() >
      config_.signal_light().righ_turn_creep().speed_limit()) {
    ADEBUG << "Do not creep forward due to large speed.";
    return;
  }

  const double creep_s_buffer =
      config_.signal_light().righ_turn_creep().min_boundary_s();
  const auto& path_decision = reference_line_info->path_decision();
  for (const auto& obstacle : path_decision->obstacles().Items()) {
    const auto& st_boundary = obstacle->reference_line_st_boundary();
    const double stop_s =
        signal_light->start_s - config_.signal_light().stop_distance();
    if (reference_line_info->AdcSlBoundary().end_s() + st_boundary.min_s() <
        stop_s + creep_s_buffer) {
      AERROR << "Do not creep forward because obstacles are close.";
      return;
    }
  }
  signal_light->start_s = reference_line_info->AdcSlBoundary().end_s() +
                          config_.signal_light().stop_distance() +
                          creep_s_buffer;
  ADEBUG << "Creep forward s = " << signal_light->start_s;
}

TrafficLight SignalLight::GetSignal(const std::string& signal_id) {
  const auto* result =
      apollo::common::util::FindPtrOrNull(detected_signals_, signal_id);
  if (result == nullptr) {
    TrafficLight traffic_light;
    traffic_light.set_id(signal_id);
    traffic_light.set_color(TrafficLight::UNKNOWN);
    traffic_light.set_confidence(0.0);
    traffic_light.set_tracking_time(0.0);
    return traffic_light;
  }
  return *result;
}

bool SignalLight::BuildStopDecision(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    hdmap::PathOverlap* const signal_light) {
  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), signal_light->start_s)) {
    ADEBUG << "signal_light " << signal_light->object_id
           << " is not on reference line";
    return true;
  }

  // create virtual stop wall
  std::string virtual_obstacle_id =
      SIGNAL_LIGHT_VO_ID_PREFIX + signal_light->object_id;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, signal_light->start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
    return false;
  }
  Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for " << virtual_obstacle_id;
    return false;
  }

  // build stop decision
  const double stop_s =
      signal_light->start_s - config_.signal_light().stop_distance();
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  auto signal_color = GetSignal(signal_light->object_id).color();
  if (signal_color == TrafficLight::YELLOW) {
    stop_decision->set_reason_code(StopReasonCode::STOP_REASON_YELLOW_SIGNAL);
  } else {
    stop_decision->set_reason_code(StopReasonCode::STOP_REASON_SIGNAL);
  }
  stop_decision->set_distance_s(-config_.signal_light().stop_distance());
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
