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
#include "modules/planning/planning_component.h"

#include "modules/common/adapters/adapter_gflags.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;

bool PlanningComponent::Init() {
  if (FLAGS_open_space_planner_switchable) {
    planning_base_ = std::unique_ptr<PlanningBase>(new OpenSpacePlanning());
  } else {
    planning_base_ = std::unique_ptr<PlanningBase>(new StdPlanning());
  }
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                               &config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;
  planning_base_->Init(config_);

  if (FLAGS_use_sim_time) {
    Clock::SetMode(Clock::MOCK);
  }
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      FLAGS_routing_response_topic,
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });
  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      FLAGS_traffic_light_detection_topic,
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  if (FLAGS_use_navigation_mode) {
    pad_message_reader_ = node_->CreateReader<PadMessage>(
        FLAGS_planning_pad_topic,
        [this](const std::shared_ptr<PadMessage>& pad_message) {
          ADEBUG << "Received pad data: run pad callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          pad_message_.CopyFrom(*pad_message);
        });
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        FLAGS_relative_map_topic,
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }
  planning_writer_ =
      node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);

  rerouting_writer_ =
      node_->CreateWriter<RoutingRequest>(FLAGS_routing_request_topic);

  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  CHECK(prediction_obstacles != nullptr);

  if (FLAGS_use_sim_time) {
    Clock::SetNowInSeconds(localization_estimate->header().timestamp_sec());
  }
  // check and process possible rerouting request
  CheckRerouting();

  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
      local_view_.is_new_routing = true;
    } else {
      local_view_.is_new_routing = false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
  }

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajecotry relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  planning_writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));
  return true;
}

void PlanningComponent::CheckRerouting() {
  auto* rerouting =
      PlanningContext::MutablePlanningStatus()->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(
      std::make_shared<RoutingRequest>(rerouting->routing_request()));
}

bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (!local_view_.routing->has_header()) {
    not_ready->set_reason("routing not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(std::make_shared<ADCTrajectory>(trajectory_pb));
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
