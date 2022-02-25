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

#include "modules/planning/traffic_rules/crosswalk.h"

#include <limits>
#include <unordered_map>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/proto/planning_status.pb.h"
#include "modules/planning/traffic_rules/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::common::util::WithinBound;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;
using apollo::perception::PerceptionObstacle;
using CrosswalkToStop =
    std::vector<std::pair<const hdmap::PathOverlap*, std::vector<std::string>>>;
using CrosswalkStopTimer =
    std::unordered_map<std::string, std::unordered_map<std::string, double>>;

Crosswalk::Crosswalk(const TrafficRuleConfig& config) : TrafficRule(config) {}

Status Crosswalk::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FindCrosswalks(reference_line_info)) {
    PlanningContext::MutablePlanningStatus()->clear_crosswalk();
    return Status::OK();
  }

  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void Crosswalk::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  auto* mutable_crosswalk_status =
      PlanningContext::MutablePlanningStatus()->mutable_crosswalk();

  auto* path_decision = reference_line_info->path_decision();
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  CrosswalkToStop crosswalks_to_stop;

  // read crosswalk_stop_timer from saved status
  CrosswalkStopTimer crosswalk_stop_timer;
  std::unordered_map<std::string, double> stop_times;
  for (int i = 0; i < mutable_crosswalk_status->stop_time_size(); ++i) {
    stop_times.insert(
        {mutable_crosswalk_status->stop_time(i).obstacle_id(),
         mutable_crosswalk_status->stop_time(i).obstacle_stop_timestamp()});
  }
  crosswalk_stop_timer.insert(
      {mutable_crosswalk_status->crosswalk_id(), stop_times});

  std::vector<std::string> finished_crosswalks;
  for (int i = 0; i < mutable_crosswalk_status->finished_crosswalk_size();
      i++) {
    finished_crosswalks.push_back(
        mutable_crosswalk_status->finished_crosswalk(i));
  }

  const auto& reference_line = reference_line_info->reference_line();
  for (auto crosswalk_overlap : crosswalk_overlaps_) {
    auto crosswalk_ptr = HDMapUtil::BaseMap().GetCrosswalkById(
        hdmap::MakeMapId(crosswalk_overlap->object_id));
    std::string crosswalk_id = crosswalk_ptr->id().id();

    // skip crosswalk if master vehicle body already passes the stop line
    if (adc_front_edge_s - crosswalk_overlap->end_s >
        config_.crosswalk().min_pass_s_distance()) {
      if (mutable_crosswalk_status->has_crosswalk_id() &&
          mutable_crosswalk_status->crosswalk_id() == crosswalk_id) {
        mutable_crosswalk_status->clear_crosswalk_id();
        mutable_crosswalk_status->clear_stop_time();
      }

      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id
          << "] crosswalk_overlap_end_s[" << crosswalk_overlap->end_s
          << "] adc_front_edge_s[" << adc_front_edge_s
          << "]. adc_front_edge passes crosswalk_end_s + buffer.";
      continue;
    }

    // check if crosswalk already finished
    if (finished_crosswalks.end() !=
        std::find(finished_crosswalks.begin(), finished_crosswalks.end(),
                  crosswalk_id)) {
      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id
          << "] crosswalk_end_s[" << crosswalk_overlap->end_s
          << "] finished already";
      continue;
    }

    std::vector<std::string> pedestrians;
    for (const auto* obstacle : path_decision->obstacles().Items()) {
      bool stop = CheckStopForObstacle(reference_line_info,
                                       crosswalk_ptr,
                                       *obstacle);

      const std::string& obstacle_id = obstacle->Id();
      const PerceptionObstacle& perception_obstacle = obstacle->Perception();
      PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);

      // update stop timestamp on static pedestrian for watch timer
      const bool is_on_lane =
          reference_line.IsOnLane(obstacle->PerceptionSLBoundary());
      const double kStartWatchTimerDistance = 40.0;
      if (stop && !is_on_lane &&
          crosswalk_overlap->start_s - adc_front_edge_s
              <= kStartWatchTimerDistance) {
        // check on stop timer for static pedestrians/bicycles
        // if NOT on_lane ahead of adc
        const double kMaxStopSpeed = 0.3;
        auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                         perception_obstacle.velocity().y());
        if (obstacle_speed <= kMaxStopSpeed) {
          if (crosswalk_stop_timer[crosswalk_id].count(obstacle_id) < 1) {
            // add timestamp
            ADEBUG << "add timestamp: obstacle_id[" << obstacle_id
                   << "] timestamp[" << Clock::NowInSeconds() << "]";
            crosswalk_stop_timer[crosswalk_id].insert(
                {obstacle_id, Clock::NowInSeconds()});
          } else {
            double stop_time = Clock::NowInSeconds() -
                crosswalk_stop_timer[crosswalk_id][obstacle_id];
            ADEBUG << "stop_time: obstacle_id[" << obstacle_id
                << "] stop_time[" << stop_time << "]";
            if (stop_time >= config_.crosswalk().stop_timeout()) {
              stop = false;
            }
          }
        }
      }

      if (stop) {
        pedestrians.push_back(obstacle_id);
        ADEBUG << "wait for: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      } else {
        ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      }
    }

    if (!pedestrians.empty()) {
      // stop decision
      double stop_deceleration = util::GetADCStopDeceleration(
          reference_line_info, crosswalk_overlap->start_s,
          config_.crosswalk().min_pass_s_distance());
      if (stop_deceleration < config_.crosswalk().max_stop_deceleration()) {
        crosswalks_to_stop.push_back(
            std::make_pair(crosswalk_overlap, pedestrians));
        ADEBUG << "crosswalk_id[" << crosswalk_id << "] STOP";
      }
    }
  }

  double min_s = std::numeric_limits<double>::max();
  hdmap::PathOverlap* firsts_crosswalk_to_stop = nullptr;
  for (auto crosswalk_to_stop : crosswalks_to_stop) {
    // build stop decision
    BuildStopDecision(frame, reference_line_info,
                      const_cast<hdmap::PathOverlap*>(crosswalk_to_stop.first),
                      crosswalk_to_stop.second);

    if (crosswalk_to_stop.first->start_s < min_s) {
      firsts_crosswalk_to_stop =
          const_cast<PathOverlap*>(crosswalk_to_stop.first);
      min_s = crosswalk_to_stop.first->start_s;
    }
  }

  if (firsts_crosswalk_to_stop) {
    // update CrosswalkStatus
    std::string crosswalk = firsts_crosswalk_to_stop->object_id;
    mutable_crosswalk_status->set_crosswalk_id(crosswalk);
    mutable_crosswalk_status->clear_stop_time();
    for (auto it = crosswalk_stop_timer[crosswalk].begin();
        it != crosswalk_stop_timer[crosswalk].end(); ++it) {
      auto* stop_time = mutable_crosswalk_status->add_stop_time();
      stop_time->set_obstacle_id(it->first);
      stop_time->set_obstacle_stop_timestamp(it->second);
      ADEBUG << "UPDATE stop_time: id[" << crosswalk
          << "] obstacle_id[" << it->first
          << "] stop_timestamp[" << it->second << "]";
    }

    // update CrosswalkStatus.finished_crosswalk
    mutable_crosswalk_status->clear_finished_crosswalk();
    for (auto crosswalk_overlap : crosswalk_overlaps_) {
      if (crosswalk_overlap->start_s < firsts_crosswalk_to_stop->start_s) {
        mutable_crosswalk_status->add_finished_crosswalk(
            crosswalk_overlap->object_id);
        ADEBUG << "UPDATE finished_crosswalk: "
            << crosswalk_overlap->object_id;
      }
    }
  }

  ADEBUG << "crosswalk_status: " << mutable_crosswalk_status->DebugString();
}

bool Crosswalk::FindCrosswalks(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  crosswalk_overlaps_.clear();
  const std::vector<hdmap::PathOverlap>& crosswalk_overlaps =
      reference_line_info->reference_line().map_path().crosswalk_overlaps();
  for (const hdmap::PathOverlap& crosswalk_overlap : crosswalk_overlaps) {
    crosswalk_overlaps_.push_back(&crosswalk_overlap);
  }
  return crosswalk_overlaps_.size() > 0;
}

bool Crosswalk::CheckStopForObstacle(
    ReferenceLineInfo* const reference_line_info,
    const CrosswalkInfoConstPtr crosswalk_ptr,
    const Obstacle& obstacle) {
  CHECK_NOTNULL(reference_line_info);

  std::string crosswalk_id = crosswalk_ptr->id().id();

  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  const std::string& obstacle_id = obstacle.Id();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name =
      PerceptionObstacle_Type_Name(obstacle_type);
  double adc_end_edge_s = reference_line_info->AdcSlBoundary().start_s();

  // check type
  if (obstacle_type != PerceptionObstacle::PEDESTRIAN &&
      obstacle_type != PerceptionObstacle::BICYCLE &&
      obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
      obstacle_type != PerceptionObstacle::UNKNOWN) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type["
           << obstacle_type_name << "]. skip";
    return false;
  }

  // expand crosswalk polygon
  // note: crosswalk expanded area will include sideway area
  Vec2d point(perception_obstacle.position().x(),
              perception_obstacle.position().y());
  const Polygon2d crosswalk_exp_poly =
      crosswalk_ptr->polygon().ExpandByDistance(
          config_.crosswalk().expand_s_distance());
  bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);

  if (!in_expanded_crosswalk) {
    ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
        << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
        << "]: not in crosswalk expanded area";
    return false;
  }

  const auto& reference_line = reference_line_info->reference_line();

  common::SLPoint obstacle_sl_point;
  reference_line.XYToSL({perception_obstacle.position().x(),
                         perception_obstacle.position().y()},
                        &obstacle_sl_point);
  auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  const double obstacle_l_distance = std::min(
      std::fabs(obstacle_sl_boundary.start_l()),
      std::fabs(obstacle_sl_boundary.end_l()));

  const bool is_on_lane =
      reference_line.IsOnLane(obstacle.PerceptionSLBoundary());
  const bool is_on_road =
      reference_line.IsOnRoad(obstacle.PerceptionSLBoundary());
  const bool is_path_cross =
      !obstacle.reference_line_st_boundary().IsEmpty();

  ADEBUG << "obstacle_id[" << obstacle_id
      << "] type[" << obstacle_type_name
      << "] crosswalk_id[" << crosswalk_id
      << "] obstacle_l[" << obstacle_sl_point.l()
      << "] within_expanded_crosswalk_area[" << in_expanded_crosswalk
      << "] obstacle_l_distance[" << obstacle_l_distance
      << "] on_lane[" << is_on_lane
      << "] is_on_road[" << is_on_road
      << "] is_path_cross[" << is_path_cross << "]";

  bool stop = false;
  if (obstacle_l_distance >= config_.crosswalk().stop_loose_l_distance()) {
    // (1) when obstacle_l_distance is big enough(>= loose_l_distance),
    //     STOP only if paths crosses
    if (is_path_cross) {
      stop = true;
      ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle_id << "] type["
             << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
             << "]";
    }
  } else if (obstacle_l_distance <=
             config_.crosswalk().stop_strick_l_distance()) {
    if (is_on_road) {
      // (2) when l_distance <= strick_l_distance + on_road
      //     always STOP
      if (obstacle_sl_point.s() > adc_end_edge_s) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id
            << "] type[" << obstacle_type_name << "] s["
            << obstacle_sl_point.s() << "] adc_end_edge_s[ "
            << adc_end_edge_s << "] crosswalk_id[" << crosswalk_id
            << "] ON_ROAD";
      }
    } else {
      // (3) when l_distance <= strick_l_distance
      //     + NOT on_road(i.e. on crosswalk/median etc)
      //     STOP if paths cross
      if (is_path_cross) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id
               << "] type[" << obstacle_type_name << "] crosswalk_id["
               << crosswalk_id << "] PATH_CRSOSS";
      } else {
        // (4) when l_distance <= strick_l_distance
        //     + NOT on_road(i.e. on crosswalk/median etc)
        //     STOP if he pedestrian is moving toward the ego vehicle
        const auto obstacle_v = Vec2d(perception_obstacle.velocity().x(),
                                      perception_obstacle.velocity().y());
        const auto adc_path_point =
            Vec2d(EgoInfo::Instance()->start_point().path_point().x(),
                  EgoInfo::Instance()->start_point().path_point().y());
        const auto ovstacle_position =
            Vec2d(perception_obstacle.position().x(),
                  perception_obstacle.position().y());
        auto obs_to_adc = adc_path_point - ovstacle_position;
        const double kEpsilon = 1e-6;
        if (obstacle_v.InnerProd(obs_to_adc) > kEpsilon) {
          stop = true;
          ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id
              << "] type[" << obstacle_type_name << "] crosswalk_id["
              << crosswalk_id << "] MOVING_TOWARD_ADC";
        }
      }
    }
  } else {
    // (4) when l_distance is between loose_l and strick_l
    //     use history decision of this crosswalk to smooth unsteadiness

    // TODO(all): replace this temp implementation
    if (is_path_cross) {
      stop = true;
    }
    ADEBUG << "need_stop(between l1 & l2): obstacle_id[" << obstacle_id
        << "] type[" << obstacle_type_name << "] obstacle_l_distance["
        << obstacle_l_distance << "] crosswalk_id[" << crosswalk_id
        << "] USE_PREVIOUS_DECISION";
  }

  return stop;
}

int Crosswalk::BuildStopDecision(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info,
                                 hdmap::PathOverlap* const crosswalk_overlap,
                                 std::vector<std::string> pedestrians) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(crosswalk_overlap);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), crosswalk_overlap->start_s)) {
    ADEBUG << "crosswalk [" << crosswalk_overlap->object_id
           << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  std::string virtual_obstacle_id =
      CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, crosswalk_overlap->start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
    return -1;
  }
  Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << virtual_obstacle_id;
    return -1;
  }

  // build stop decision
  const double stop_s =
      crosswalk_overlap->start_s - config_.crosswalk().stop_distance();
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_CROSSWALK);
  stop_decision->set_distance_s(-config_.crosswalk().stop_distance());
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (auto pedestrian : pedestrians) {
    stop_decision->add_wait_for_obstacle(pedestrian);
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return 0;
}

}  // namespace planning
}  // namespace apollo
