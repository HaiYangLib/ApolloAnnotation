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

#include "modules/perception/onboard/component/radar_detection_component.h"

#include "cyber/time/clock.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

using Clock = apollo::cyber::Clock;

namespace apollo {
namespace perception {
namespace onboard {

bool RadarDetectionComponent::Init() {
  /**
   * 由配置文件front_radar_component_conf.pb.txt或
   * rear_radar_component_conf.pb.txt 进行配置
   *
   * 以front_radar_component_conf.pb.txt为例
   * radar_name: "radar_front"
   * tf_child_frame_id: "radar_front"
   * radar_forward_distance: 200.0
   * radar_preprocessor_method: "ContiArsPreprocessor"
   * radar_perception_method: "RadarObstaclePerception"
   * radar_pipeline_name: "FrontRadarPipeline"
   * odometry_channel_name: "/apollo/localization/pose"
   * output_channel_name: "/perception/inner/PrefusedObjects"
   *
   * **/
  RadarComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }

  AINFO << "Radar Component Configs: " << comp_config.DebugString();

  // To load component configs
  // tf_child_frame_id: "radar_front"
  tf_child_frame_id_ = comp_config.tf_child_frame_id();

  // radar_forward_distance: 200.0
  radar_forward_distance_ = comp_config.radar_forward_distance();

  // radar_preprocessor_method: "ContiArsPreprocessor"
  preprocessor_method_ = comp_config.radar_preprocessor_method();

  // adar_perception_method: "RadarObstaclePerception"
  perception_method_ = comp_config.radar_perception_method();

  // radar_pipeline_name: "FrontRadarPipeline"
  pipeline_name_ = comp_config.radar_pipeline_name();

  /**
   * Radar子模块需要订阅话题:"/apollo/localization/pose"
   * 用来获取世界坐标系的经纬度
   * odometry_channel_name: "/apollo/localization/pose"
   * **/
  odometry_channel_name_ = comp_config.odometry_channel_name();

  /**
   * comp_config.radar_name()="radar_front"
   * 填充radar_info_
   * **/
  if (!common::SensorManager::Instance()->GetSensorInfo(
          comp_config.radar_name(), &radar_info_)) {
    AERROR << "Failed to get sensor info, sensor name: "
           << comp_config.radar_name();
    return false;
  }

  /**
   *
   * SensorFrameMessage是输出的结果类型
   * 在下面文件定义：
   * modules/perception/onboard/inner_component_messages/
   *        inner_component_messages.h
   *
   * class SensorFrameMessage {
   * public:
   *   std::string sensor_id_;
   *   double timestamp_ = 0.0;
   *   uint64_t lidar_timestamp_ = 0;
   *   uint32_t seq_num_ = 0;
   *   std::string type_name_;
   *   base::HdmapStructConstPtr hdmap_;
   *   base::FramePtr frame_;
   *   ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
   * };
   *
   * Radar子模块的输出Topic
   * output_channel_name: "/perception/inner/PrefusedObjects"
   * **/
  writer_ = node_->CreateWriter<SensorFrameMessage>(
      comp_config.output_channel_name());

  // Init algorithm plugin
  /**
   * 根据配置内容，初始化相应的算法实例
   *
   * 几乎所有的模块都会按照下面的格式：
   * 由相应的Component对接Cyber（如订阅或发布某些话题）
   * 由该Component的成员变量实现具体的算法细节
   * 这种设计思想可以很方便的移植算法实例
   * 因为算法实例本身并没有和Cyber强绑定
   * **/
  ACHECK(InitAlgorithmPlugin()) << "Failed to init algorithm plugin.";

  /**
   * 详情参见Transform模块注释
   * **/
  radar2world_trans_.Init(tf_child_frame_id_);
  radar2novatel_trans_.Init(tf_child_frame_id_);

  /**
   * 成员变量localization_subscriber_类型 MsgBuffer<LocalizationEstimate>
   * MsgBuffer会负责订阅某个topic的数据，并缓存
   * 提供Lookup系列函数，用于查找
   * MsgBuffer实现比较简单
   * **/
  localization_subscriber_.Init(
      odometry_channel_name_,
      odometry_channel_name_ + '_' + comp_config.radar_name());
  return true;
}

/**
 * RadarDetectionComponent对应的dag文件
 * modules/perception/production/dag/dag_streaming_perception.dag
 * 内容：
 * 该模块对应channel: "/apollo/sensor/radar/front"
 * 或者 channel: "/apollo/sensor/radar/rear"
 * 前后Radar子模块只是订阅的channel不同，其他完全一样
 * 本注释假设channel= "/apollo/sensor/radar/front"
 * 数据类型ContiRadar在下面文件中定义
 * modules/drivers/proto/conti_radar.proto
 * 大陆ArsRadar坐标系
 *
 *                  x axis  ^
 *                          | longitude_dist
 *                          |
 *                          |
 *                          |
 *            lateral_dist  |
 *            y axis        |
 *          <----------------
 *
 * **/
bool RadarDetectionComponent::Proc(const std::shared_ptr<ContiRadar>& message) {
  AINFO << "Enter radar preprocess, message timestamp: "
        << message->header().timestamp_sec() << " current timestamp "
        << Clock::NowInSeconds();

  /**
   * 
   * class SensorFrameMessage {
   * public:
   *   apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;
   *   std::string sensor_id_;
   *   double timestamp_ = 0.0;
   *   uint64_t lidar_timestamp_ = 0;
   *   uint32_t seq_num_ = 0;
   *   std::string type_name_;
   *   base::HdmapStructConstPtr hdmap_;
   *   base::FramePtr frame_;
   *   ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
   * };
   * 
   * base::FramePtr frame_;保存这有价值的数据
   *  **/
  auto out_message = std::make_shared<SensorFrameMessage>();

  if (!InternalProc(message, out_message)) {
    return false;
  }

  /**
   * Radar子模块的输出Topic
   * output_channel_name: "/perception/inner/PrefusedObjects"
   * out_message是系列track到的点
   * **/
  writer_->Write(out_message);
  AINFO << "Send radar processing output message.";
  return true;
}

bool RadarDetectionComponent::InitAlgorithmPlugin() {
  AINFO << "onboard radar_preprocessor: " << preprocessor_method_;

  /**
   * DEFINE_bool(obs_enable_hdmap_input, true, "enable hdmap input for roi
   * filter"); 由下面文件定义：
   * modules/perception/onboard/common_flags/common_flags.cc
   * **/
  // TODO 分析以下代码
  if (FLAGS_obs_enable_hdmap_input) {
    hdmap_input_ = map::HDMapInput::Instance();
    ACHECK(hdmap_input_->Init()) << "Failed to init hdmap input.";
  }

  /**
   * preprocessor_method_: "ContiArsPreprocessor"
   * 得到一个ContiArsPreprocessor类指针
   * ContiArsPreprocessor继承自BasePreprocessor
   * **/
  radar::BasePreprocessor* preprocessor =
      radar::BasePreprocessorRegisterer::GetInstanceByName(
          preprocessor_method_);

  CHECK_NOTNULL(preprocessor);
  /**
   * 成员变量radar_preprocessor_
   * std::shared_ptr<radar::BasePreprocessor>
   * **/
  radar_preprocessor_.reset(preprocessor);  // BasePreprocessor

  /**
   * 初始化预处理实例,Init()内容比较简单
   * **/
  ACHECK(radar_preprocessor_->Init()) << "Failed to init radar preprocessor.";

  /**
   * perception_method_="RadarObstaclePerception"
   * 得到一个RadarObstaclePerception
   * adarObstaclePerception继承自BaseRadarObstaclePerception
   * **/
  radar::BaseRadarObstaclePerception* radar_perception =
      radar::BaseRadarObstaclePerceptionRegisterer::GetInstanceByName(
          perception_method_);

  ACHECK(radar_perception != nullptr)
      << "No radar obstacle perception named: " << perception_method_;

  /**
   * 成员变量radar_perception_
   * 类型std::shared_ptr<radar::BaseRadarObstaclePerception>
   * ***/
  radar_perception_.reset(radar_perception);  // BaseRadarObstaclePerception

  // pipeline_name_="FrontRadarPipeline"
  /**
   * 初始化感知实例：
   * 1.根据配置文件，获取相关配置项
   * 2.创建实例ContiArsDetector
   * 3.创建实例HdmapRadarRoiFilter
   * 4.创建实例ContiArsTracker
   * 5.初始化相关实例
   * **/
  ACHECK(radar_perception_->Init(pipeline_name_))
      << "Failed to init radar perception.";
  AINFO << "Init algorithm plugin successfully.";
  return true;
}

/**
 * 步骤1：预处理筛选掉不符合要求的点
 * 代码：radar_preprocessor_->Preprocess(raw_obstacles, preprocessor_options,
 *                      &corrected_obstacles);
 * 步骤2：得到radar2world的转换矩阵
 * 代码：radar2world_trans_.GetSensor2worldTrans(timestamp, &radar_trans)
 * 步骤3：得到radar2自车的转换矩阵
 * 代码：adar2novatel_trans_.GetTrans(timestamp, &radar2novatel_trans, "novatel",
 *                                    tf_child_frame_id_)
 * 步骤4：得到自车线速度和角速度：car_linear_speed，car_angular_speed
 * 代码：GetCarLocalizationSpeed(timestamp,
 *                              &(options.detector_options.car_linear_speed),
 *                              &(options.detector_options.car_angular_speed))
 * 步骤5：得到radar到world的T矩阵偏移量，实际上是Radar的是世界坐标系下的x,y,z
 * 步骤6：
 * 代码：hdmap_input_->GetRoiHDMapStruct(position, radar_forward_distance_,
 *                                   options.roi_filter_options.roi);
 * 步骤7：核心工作，对预处理修正后的点云运行感知算法
 * 代码：adar_perception_->Perceive(corrected_obstacles, options,
 *                                  &radar_objects)
 * 步骤8：整理输出结果
 * **/
bool RadarDetectionComponent::InternalProc(
    const std::shared_ptr<ContiRadar>& in_message,
    std::shared_ptr<SensorFrameMessage> out_message) {
  PERF_FUNCTION_WITH_INDICATOR(radar_info_.name);
  ContiRadar raw_obstacles = *in_message;
  {
    std::unique_lock<std::mutex> lock(_mutex);
    ++seq_num_;
  }
  double timestamp = in_message->header().timestamp_sec();
  const double cur_time = Clock::NowInSeconds();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Radar:Start:msg_time[" << timestamp
        << "]:cur_time[" << cur_time << "]:cur_latency[" << start_latency
        << "]";
  PERF_BLOCK_START();
  // Init preprocessor_options
  // 初始化预处理参数
  radar::PreprocessorOptions preprocessor_options;
  // 预处理结果
  ContiRadar corrected_obstacles;
  /**
   * 进行预处理做三件事：
   * 1.筛选掉时间上不符合的点
   * 2.重新分配各个点的id
   * 3.矫正该帧时间
   * **/
  // 步骤1
  radar_preprocessor_->Preprocess(raw_obstacles, preprocessor_options,
                                  &corrected_obstacles);

  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "radar_preprocessor");

  timestamp = corrected_obstacles.header().timestamp_sec();

  out_message->timestamp_ = timestamp;
  out_message->seq_num_ = seq_num_;
  out_message->process_stage_ = ProcessStage::LONG_RANGE_RADAR_DETECTION;
  out_message->sensor_id_ = radar_info_.name;

  /**
   * 得到radar2world转换矩阵：radar_trans
   * radar2自车转换矩阵：radar2novatel_trans
   * 自车线速度和角速度：car_linear_speed，car_angular_speed
   * **/
  // Init radar perception options
  radar::RadarPerceptionOptions options;
  options.sensor_name = radar_info_.name;
  // Init detector_options
  // 步骤2
  Eigen::Affine3d radar_trans;
  if (!radar2world_trans_.GetSensor2worldTrans(timestamp, &radar_trans)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get pose at time: " << timestamp;
    return true;
  }
  
  // 步骤3
  Eigen::Affine3d radar2novatel_trans;
  if (!radar2novatel_trans_.GetTrans(timestamp, &radar2novatel_trans, "novatel",
                                     tf_child_frame_id_)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get radar2novatel trans at time: " << timestamp;
    return true;
  }

  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "GetSensor2worldTrans");
  Eigen::Matrix4d radar2world_pose = radar_trans.matrix();
  options.detector_options.radar2world_pose = &radar2world_pose;
  Eigen::Matrix4d radar2novatel_trans_m = radar2novatel_trans.matrix();
  options.detector_options.radar2novatel_trans = &radar2novatel_trans_m;

  // 步骤4
  if (!GetCarLocalizationSpeed(timestamp,
                               &(options.detector_options.car_linear_speed),
                               &(options.detector_options.car_angular_speed))) {
    AERROR << "Failed to call get_car_speed. [timestamp: " << timestamp;
    // return false;
  }

  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "GetCarSpeed");
  // Init roi_filter_options

  /**
   * radar到world的T矩阵偏移量
   * **/
  // 步骤5
  base::PointD position;
  position.x = radar_trans(0, 3);
  position.y = radar_trans(1, 3);
  position.z = radar_trans(2, 3);

  options.roi_filter_options.roi.reset(new base::HdmapStruct());
  /**
   * FLAGS_obs_enable_hdmap_input在下面文件中定义
   * modules/perception/onboard/common_flags/common_flags.cc
   * DEFINE_bool(obs_enable_hdmap_input, true, 
   *        "enable hdmap input for roi filter");
   * **/
  // TODO 注解
  // 步骤6
  if (FLAGS_obs_enable_hdmap_input) {
    hdmap_input_->GetRoiHDMapStruct(position, radar_forward_distance_,
                                    options.roi_filter_options.roi);
  }
  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "GetRoiHDMapStruct");
  // Init object_filter_options
  // Init track_options
  // Init object_builder_options
  /**
   * 进行感知算法,
   * 成员变量radar_perception_
   * 类型std::shared_ptr<radar::BaseRadarObstaclePerception>
   * 指向实例RadarObstaclePerception
   * 
   * **/
  // 步骤7
  std::vector<base::ObjectPtr> radar_objects;
  if (!radar_perception_->Perceive(corrected_obstacles, options,
                                   &radar_objects)) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "RadarDetector Proc failed.";
    return true;
  }

  // 步骤8 
  out_message->frame_.reset(new base::Frame());
  out_message->frame_->sensor_info = radar_info_;
  out_message->frame_->timestamp = timestamp;
  out_message->frame_->sensor2world_pose = radar_trans;
  out_message->frame_->objects = radar_objects;

  const double end_timestamp = Clock::NowInSeconds();
  const double end_latency =
      (end_timestamp - in_message->header().timestamp_sec()) * 1e3;
  AINFO << "FRAME_STATISTICS:Radar:End:msg_time["
        << in_message->header().timestamp_sec() << "]:cur_time["
        << end_timestamp << "]:cur_latency[" << end_latency << "]";
  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "radar_perception");

  return true;
}

bool RadarDetectionComponent::GetCarLocalizationSpeed(
    double timestamp, Eigen::Vector3f* car_linear_speed,
    Eigen::Vector3f* car_angular_speed) {
  if (car_linear_speed == nullptr) {
    AERROR << "car_linear_speed is not available";
    return false;
  }
  (*car_linear_speed) = Eigen::Vector3f::Zero();
  if (car_linear_speed == nullptr) {
    AERROR << "car_linear_speed is not available";
    return false;
  }
  (*car_angular_speed) = Eigen::Vector3f::Zero();
  std::shared_ptr<LocalizationEstimate const> loct_ptr;
  if (!localization_subscriber_.LookupNearest(timestamp, &loct_ptr)) {
    AERROR << "Cannot get car speed.";
    return false;
  }
  (*car_linear_speed)[0] =
      static_cast<float>(loct_ptr->pose().linear_velocity().x());
  (*car_linear_speed)[1] =
      static_cast<float>(loct_ptr->pose().linear_velocity().y());
  (*car_linear_speed)[2] =
      static_cast<float>(loct_ptr->pose().linear_velocity().z());
  (*car_angular_speed)[0] =
      static_cast<float>(loct_ptr->pose().angular_velocity().x());
  (*car_angular_speed)[1] =
      static_cast<float>(loct_ptr->pose().angular_velocity().y());
  (*car_angular_speed)[2] =
      static_cast<float>(loct_ptr->pose().angular_velocity().z());

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
