/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/******************************************************************************
* AnnotationAuthor  : HaiYang
* Email   : hanhy20@mails.jlu.edu.cn
* Desc    : annotation for apollo
******************************************************************************/


#include "modules/perception/radar/lib/preprocessor/conti_ars_preprocessor/conti_ars_preprocessor.h"
#include "modules/common/util/perf_util.h"

namespace apollo {
namespace perception {
namespace radar {

int ContiArsPreprocessor::current_idx_ = 0;
std::unordered_map<int, int> ContiArsPreprocessor::local2global_;

bool ContiArsPreprocessor::Init() {
  std::string model_name = "ContiArsPreprocessor";
  const lib::ModelConfig* model_config = nullptr;
  /**
   * 详情参见对ConfigManager的注释
   * modules/perception/lib/config_manager/config_manager.cc
   * 显然ConfigManager是单例
   * 
   * 变量model_config对应的配置文件
   * modules/perception/production/conf/perception/radar/modules/
   *                        conti_ars_preprocessor.config
   * 内容：
   * model_configs {
   * name: "ContiArsPreprocessor"
   *   version: "1.0.0"
   *   float_params {
   *     name: "delay_time"
   *     value: 0.07
   *   }
   * }

   * **/
  ACHECK(lib::ConfigManager::Instance()->GetModelConfig(model_name,
                                                        &model_config));
  // delay_time_=0.07                                                 
  ACHECK(model_config->get_value("delay_time", &delay_time_));
  return true;
}

/**
 * 步骤1：筛选掉时间上不符合的点
 * 步骤2：重新分配各个点的id
 * 步骤3：矫正该帧时间
 * **/
bool ContiArsPreprocessor::Preprocess(
    const drivers::ContiRadar& raw_obstacles,
    const PreprocessorOptions& options,
    drivers::ContiRadar* corrected_obstacles) {
  PERF_FUNCTION();
 
  // 步骤1
  SkipObjects(raw_obstacles, corrected_obstacles);
  // 步骤2
  ExpandIds(corrected_obstacles);
  // 步骤3
  CorrectTime(corrected_obstacles);
  return true;
}

std::string ContiArsPreprocessor::Name() const {
  return "ContiArsPreprocessor";
}

// 根据时间戳筛选
void ContiArsPreprocessor::SkipObjects(
    const drivers::ContiRadar& raw_obstacles,
    drivers::ContiRadar* corrected_obstacles) {
  corrected_obstacles->mutable_header()->CopyFrom(raw_obstacles.header());

  double timestamp = raw_obstacles.header().timestamp_sec() - 1e-6;
  /**
   * 筛选掉时间上不符合的点
   * **/
  for (const auto& contiobs : raw_obstacles.contiobs()) {
    double object_timestamp = contiobs.header().timestamp_sec();
    if (object_timestamp > timestamp &&
        object_timestamp < timestamp + CONTI_ARS_INTERVAL) {
      drivers::ContiRadarObs* obs = corrected_obstacles->add_contiobs();
      *obs = contiobs;
    }
  }

  if (raw_obstacles.contiobs_size() > corrected_obstacles->contiobs_size()) {
    AINFO << "skip objects: " << raw_obstacles.contiobs_size() << "-> "
          << corrected_obstacles->contiobs_size();
  }
}

void ContiArsPreprocessor::ExpandIds(drivers::ContiRadar* corrected_obstacles) {
  for (int iobj = 0; iobj < corrected_obstacles->contiobs_size(); ++iobj) {
    const auto& contiobs = corrected_obstacles->contiobs(iobj);
    int id = contiobs.obstacle_id();
    int corrected_id = 0;
    auto ob = local2global_.find(id);
    if (ob != local2global_.end()) {
      
      /**
       *  The following is only valid for the track object message
       *  0 = deleted, 1 = new, 
       *  2 = measured, 
       *  3 = predicted, 
       *  4 = deleted for,
       *  5 = new from merge optional int32 meas_state = 15;
       * **/
      
      if (CONTI_NEW == contiobs.meas_state()) {
        corrected_id = GetNextId();
        ob->second = corrected_id;
      } else {
        corrected_id = ob->second;
      }
    } else {
      corrected_id = GetNextId();
      local2global_.insert({id, corrected_id});
    }
    corrected_obstacles->mutable_contiobs(iobj)->set_obstacle_id(corrected_id);
  }
}

void ContiArsPreprocessor::CorrectTime(
    drivers::ContiRadar* corrected_obstacles) {
  // delay_time_=0.07      
  double correct_timestamp =
      corrected_obstacles->header().timestamp_sec() - delay_time_;
  corrected_obstacles->mutable_header()->set_timestamp_sec(correct_timestamp);
}

int ContiArsPreprocessor::GetNextId() {
  ++current_idx_;
  if (MAX_RADAR_IDX == current_idx_) {
    current_idx_ = 1;
  }
  return current_idx_;
}

PERCEPTION_REGISTER_PREPROCESSOR(ContiArsPreprocessor);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
