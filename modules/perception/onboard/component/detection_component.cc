/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/onboard/component/detection_component.h"

#include "cyber/time/clock.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/onboard/common_flags/common_flags.h"

using ::apollo::cyber::Clock;

namespace apollo {
namespace perception {
namespace onboard {

std::atomic<uint32_t> DetectionComponent::seq_num_{0};

bool DetectionComponent::Init() {
  /**
   * comp_config对应的配置文件
   * apollo/modules/perception/production/conf/perception/
   *    lidar/velodyne128_detection_conf.pb.txt
   * 内容：
   * sensor_name: "velodyne128"
   * enable_hdmap: true
   * lidar_query_tf_offset: 0
   * lidar2novatel_tf2_child_frame_id: "velodyne128"
   * output_channel_name: "/perception/inner/DetectionObjects"
   */

  LidarDetectionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  ADEBUG << "Lidar Component Configs: " << comp_config.DebugString();

  /**
   * output_channel_name_= "/perception/inner/DetectionObjects"
   * **/
  output_channel_name_ = comp_config.output_channel_name();
  sensor_name_ = comp_config.sensor_name();
  lidar2novatel_tf2_child_frame_id_ =
      comp_config.lidar2novatel_tf2_child_frame_id();
  lidar_query_tf_offset_ =
      static_cast<float>(comp_config.lidar_query_tf_offset());
  //  enable_hdmap_ = comp_config.enable_hdmap();
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

  // 初始化成员算法类
  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init detection component algorithm plugin.";
    return false;
  }
  return true;
}

bool DetectionComponent::Proc(
    const std::shared_ptr<drivers::PointCloud>& message) {
  AINFO << std::setprecision(16)
        << "Enter detection component, message timestamp: "
        << message->measurement_time()
        << " current timestamp: " << Clock::NowInSeconds();

  /**
   * LidarFrameMessage 在下面文件中定义
   * modules/perception/onboard/component/lidar_inner_component_messages.h
   * 
   * 其中一个比较重要的成员变量
   * std::shared_ptr<lidar::LidarFrame> lidar_frame_;
   * 
   * lidar::LidarFrame在下面文件中定义
   * modules/perception/lidar/common/lidar_frame.h
   * **/
  auto out_message = std::make_shared<LidarFrameMessage>();

  bool status = InternalProc(message, out_message);

  /**
   * std::shared_ptr<drivers::PointCloud>& message 是输入原始点云
   *
   * 经过一系列是筛选和detect后
   * 结果保存在out_message中
   * 
   * out_message->timestamp_ = timestamp;
   * out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
   * out_message->seq_num_ = seq_num;
   * out_message->process_stage_ = ProcessStage::LIDAR_DETECTION;
   * out_message->error_code_ = apollo::common::ErrorCode::OK;
   * auto& frame = out_message->lidar_frame_;
   * frame = lidar::LidarFramePool::Instance().Get();
   * frame->timestamp = timestamp;
   * frame->sensor_info = sensor_info_;
   * frame->cloud保存lidar坐标系下筛选后的点云
   * frame->world_cloud保存世界坐标系下筛选后的点云
   * frame->segmented_objects保存检测结果
   * 
   * **/

  /**
   * 将输出结果写到output_channel_name_
   * 其中output_channel_name_由配置文件赋值
   * 
   * 对于sensor_name: "velodyne128"
   * output_channel_name: "/perception/inner/DetectionObjects"
   * **/
  if (status) {
    writer_->Write(out_message);
    AINFO << "Send lidar detect output message.";
  }
  return status;
}

bool DetectionComponent::InitAlgorithmPlugin() {
  ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_,
                                                          &sensor_info_));

  /**
   * lidar::LidarObstacleDetection在下面为文件中定义
   * modules/perception/lidar/app/lidar_obstacle_detection.cc
   * **/
  detector_.reset(new lidar::LidarObstacleDetection);
  if (detector_ == nullptr) {
    AERROR << "sensor_name_ "
           << "Failed to get detection instance";
    return false;
  }
  lidar::LidarObstacleDetectionInitOptions init_options;
  init_options.sensor_name = sensor_name_;
  //  init_options.enable_hdmap_input =
  //      FLAGS_obs_enable_hdmap_input && enable_hdmap_;
  /**
   * lidar::LidarObstacleDetection初始化
   * **/
  if (!detector_->Init(init_options)) {
    AINFO << "sensor_name_ "
          << "Failed to init detection.";
    return false;
  }

  /**
   * 
   * TransformWrapper lidar2world_trans_;
   * TransformWrapper 在感知模块中会频繁用到
   * modules/perception/onboard/transform_wrapper/transform_wrapper.cc
   * 
   * 成员变量 std::string lidar2novatel_tf2_child_frame_id_;
   * 在该模块初始化时被赋值
   * lidar2novatel_tf2_child_frame_id_ =
   *      comp_config.lidar2novatel_tf2_child_frame_id();
   * 
   * lidar2novatel_tf2_child_frame_id_= "velodyne128"
   * 
   * **/
  lidar2world_trans_.Init(lidar2novatel_tf2_child_frame_id_);
  return true;
}


/**
 * 激光点云数据，在下面文件中定义
 * modules/drivers/proto/pointcloud.proto
 * 内容：
 * message PointXYZIT {
 *    optional float x = 1 [default = nan];
 *    optional float y = 2 [default = nan];
 *    optional float z = 3 [default = nan];
 *    optional uint32 intensity = 4 [default = 0];
 *    optional uint64 timestamp = 5 [default = 0];
 * }
 * 
 * message PointCloud {
 *    optional apollo.common.Header header = 1;
 *    optional string frame_id = 2;
 *    optional bool is_dense = 3;
 *    repeated PointXYZIT point = 4;
 *    optional double measurement_time = 5;
 *    optional uint32 width = 6;
 *    optional uint32 height = 7;
 * }
 * 
 * 步骤1：为out_message赋初值
 * 
 * 步骤2：得到lidar2world的坐标变换关系pose
 * 
 * 步骤3：得到传感器到自车的外参转换矩阵
 * 
 * 步骤4：开始检测
 * 代码：lidar::LidarProcessResult ret =
 *    detector_->Process(detect_opts, in_message, frame.get());
 * **/
bool DetectionComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud>& in_message,
    const std::shared_ptr<LidarFrameMessage>& out_message) {
  uint32_t seq_num = seq_num_.fetch_add(1);
  const double timestamp = in_message->measurement_time();
  const double cur_time = Clock::NowInSeconds();

  /**
   * 计算一下延时
   * **/
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << std::setprecision(16) << "FRAME_STATISTICS:Lidar:Start:msg_time["
        << timestamp << "]:sensor[" << sensor_name_ << "]:cur_time[" << cur_time
        << "]:cur_latency[" << start_latency << "]";

  /**
   * LidarFrameMessage在下面文件中定义
   * modules/perception/onboard/component/lidar_inner_component_messages.h
   * 
   * class LidarFrameMessage {
   * public:
   *    double timestamp_ = 0.0;
   *    uint64_t lidar_timestamp_ = 0；
   *    uint32_t seq_num_ = 0;
   *    std::string type_name_;
   *    ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
   *    apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;
   *    std::shared_ptr<lidar::LidarFrame> lidar_frame_
   * };
   * 
   * **/
  // 步骤1
  out_message->timestamp_ = timestamp;
  out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
  out_message->seq_num_ = seq_num;
  out_message->process_stage_ = ProcessStage::LIDAR_DETECTION;
  out_message->error_code_ = apollo::common::ErrorCode::OK;


  /**
   * lidar::LidarFrame 在下面文件中定义
   * modules/perception/lidar/common/lidar_frame.h
   * struct LidarFrame {
   *   std::shared_ptr<base::AttributePointCloud<base::PointF>> cloud;
   *   std::shared_ptr<base::AttributePointCloud<base::PointD>> world_cloud;
   *   double timestamp = 0.0;
   *   Eigen::Affine3d lidar2world_pose = Eigen::Affine3d::Identity();
   *   Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
   *   std::shared_ptr<base::HdmapStruct> hdmap_struct = nullptr;
   *   std::vector<std::shared_ptr<base::Object>> segmented_objects;
   *   std::vector<std::shared_ptr<base::Object>> tracked_objects;
   *   base::PointIndices roi_indices;
   *   base::PointIndices non_ground_indices;
   *   base::PointIndices secondary_indices;
   *   base::SensorInfo sensor_info;
   * .......................
   * } 
   * 
   * **/
  auto& frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->cloud = base::PointFCloudPool::Instance().Get();
  frame->timestamp = timestamp;
  frame->sensor_info = sensor_info_;

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();

  /**
   * 成员变量
   * float lidar_query_tf_offset_ = 20.0f;
   * **/
  const double lidar_query_tf_timestamp =
      timestamp - lidar_query_tf_offset_ * 0.001;

  /**
   * 得到lidar2world的坐标变换关系pose
   * Eigen::Affine3d pose实际上是一个4*4齐次矩阵变换
   * Eigen::Affine3d仿射变换矩阵
   * Eigen::Affine3d affine3f = translation*quater.toRotationMatrix();
   * **/ 
  // 步骤2  
  if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp,
                                               &pose)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get pose at time: "
           << FORMAT_TIMESTAMP(lidar_query_tf_timestamp);
    return false;
  }

  // 赋值，世界坐标系的转换矩阵
  frame->lidar2world_pose = pose;

  /**
   * struct LidarObstacleDetectionOptions {
   *    std::string sensor_name;
   *    Eigen::Affine3d sensor2novatel_extrinsics;
   * ..............
   * }
   * 
   * **/
  lidar::LidarObstacleDetectionOptions detect_opts;
  detect_opts.sensor_name = sensor_name_;
  
  /**
   * 传感器转自车的外参转换矩阵
   * 注意novatel代表的是自车坐标系
   * **/
  // 步骤3 
  lidar2world_trans_.GetExtrinsics(&detect_opts.sensor2novatel_extrinsics);

  /**
   * 进行检测
   * detector_在InitAlgorithmPlugin()函数中被初始化成 LidarObstacleDetection
   * 具体的检测流程在 LidarObstacleDetection实现
   *
   * frame->cloud保存lidar坐标系下筛选后的点云
   * frame->world_cloud保存世界坐标系下筛选后的点云
   * frame->segmented_objects保存检测结果
   * 
   * **/
  // 步骤4
  lidar::LidarProcessResult ret =
      detector_->Process(detect_opts, in_message, frame.get());
  if (ret.error_code != lidar::LidarErrorCode::Succeed) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Lidar detection process error, " << ret.log;
    return false;
  }

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
