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
/******************************************************************************
 * AnnotationAuthor  : HaiYang
 * Email   : hanhy20@mails.jlu.edu.cn
 * Desc    : annotation for apollo
 ******************************************************************************/

#include "modules/planning/planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"

namespace apollo {
namespace planning {

using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;

/**
 *
 * 
 * **/
bool PlanningComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();

  /**
   * modules/common/configs/config_gflags.cc
   *
   * DEFINE_bool(use_navigation_mode, false,
   *         "Use relative position in navigation mode");
   * **/
  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>(injector_);
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_);
  }

  /**
   * 该模块对应的配置文件
   * modules/planning/conf/planning_config.pb.txt
   * **/
  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  /**
   * modules/planning/common/planning_gflags.cc
   *
   * DEFINE_bool(planning_offline_learning, false,
   *         "offline learning. read record files and dump learning_data");
   *
   * learning_mode: NO_LEARNING
   * **/
  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";
      return false;
    }
  }

  planning_base_->Init(config_);

  //  routing_response_topic: "/apollo/routing_response"
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      config_.topic_config().routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  //  traffic_light_detection_topic: "/apollo/perception/traffic_light"
  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  //  planning_pad_topic: "/apollo/planning/pad"
  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  // story_telling_topic: "/apollo/storytelling"
  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });

  /**
   * modules/common/configs/config_gflags.cc
   *
   * DEFINE_bool(use_navigation_mode, false,
   *         "Use relative position in navigation mode");
   * **/
  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        config_.topic_config().relative_map_topic(),
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }

  // planning_trajectory_topic: "/apollo/planning"
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  // routing_request_topic: "/apollo/routing_request"
  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(
      config_.topic_config().routing_request_topic());

  // planning_learning_data_topic: "/apollo/planning/learning_data"
  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());

  return true;
}

/**
 * PlanningComponent 对应dag文件：
 * modules/planning/dag/planning.dag
 * 
 * flag_file_path:  "/apollo/modules/planning/conf/planning.conf"
 * 该模块订阅三个toppic:
 *
 * channel: "/apollo/prediction"
 * channel: "/apollo/canbus/chassis"
 * channel: "/apollo/localization/pose"
 * 
 * 步骤1: 检查是否需要重新规划线路。
 * 步骤2: 根据输入信息,为成员变量local_view_(LocalView)赋值,并做检查
 * 步骤3: 进行一次规划
 * 步骤4: 发布规划结果adc_trajectory_pb(ADCTrajectory)
 * 步骤5: 将规划结果adc_trajectory_pb加入到历史记录
 * **/
bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  /**
   * 检查是否需要重新规划线路。
   * **/
  // 步骤1 
  CheckRerouting();

  // process fused input data

  /**
   * struct LocalView {
   * std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
   * std::shared_ptr<canbus::Chassis> chassis;
   * std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
   * std::shared_ptr<perception::TrafficLightDetection>traffic_light;
   * std::shared_ptr<routing::RoutingResponse> routing;
   * std::shared_ptr<relative_map::MapMsg> relative_map;
   * std::shared_ptr<PadMessage> pad_msg;
   * std::shared_ptr<storytelling::Stories> stories;
   * };
   *
   * LocalView会保存着Planning所需要的信息
   * **/
  // 步骤2
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  // learning_mode: NO_LEARNING
  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);
    message_process_.OnPrediction(*local_view_.prediction_obstacles);
    message_process_.OnRoutingResponse(*local_view_.routing);
    message_process_.OnStoryTelling(*local_view_.stories);
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
    message_process_.OnLocalization(*local_view_.localization_estimate);
  }

  // publish learning data frame for RL test
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    PlanningLearningData planning_learning_data;
    LearningDataFrame* learning_data_frame =
        injector_->learning_based_data()->GetLatestLearningDataFrame();
    if (learning_data_frame) {
      planning_learning_data.mutable_learning_data_frame()->CopyFrom(
          *learning_data_frame);
      common::util::FillHeader(node_->Name(), &planning_learning_data);
      planning_learning_data_writer_->Write(planning_learning_data);
    } else {
      AERROR << "fail to generate learning data frame";
      return false;
    }
    return true;
  }

  /**
   * ADCTrajectory在下面文件中定义：
   * modules/planning/proto/planning.proto
   * 
   * RunOnce是规划模块的核心代码
   * 根据local_view_保存的信息，生成一条adc_trajectory_pb
   * **/
  // 步骤3 
  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }

  // 步骤4
  planning_writer_->Write(adc_trajectory_pb);

  // record in history

  // 步骤5
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;
}

/**
 * rerouting对应的proto
 * modules/planning/proto/planning_status.proto
 * message ReroutingStatus {
 *  optional double last_rerouting_time = 1;
 *  optional bool need_rerouting = 2 [default = false];
 *  optional apollo.routing.RoutingRequest routing_request = 3;
 * }
 *  **/
void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();

  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(rerouting->routing_request());
}

bool PlanningComponent::CheckInput() {
  /**
   * ADCTrajectory在下面文件中定义：
   * modules/planning/proto/planning.proto
   * **/
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } else {
    // nothing
  }

  /**
   * modules/common/configs/config_gflags.cc
   *
   * DEFINE_bool(use_navigation_mode, false,
   *         "Use relative position in navigation mode");
   * **/
  if (FLAGS_use_navigation_mode) {
    if (!local_view_.relative_map->has_header()) {
      not_ready->set_reason("relative map not ready");
    }
  } else {
    if (!local_view_.routing->has_header()) {
      not_ready->set_reason("routing not ready");
    }
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
