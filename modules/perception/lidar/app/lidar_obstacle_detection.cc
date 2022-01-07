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

/*****************************************************************************
 * AnnotationAuthor  : HaiYang
 * Email   : hanhy20@mails.jlu.edu.cn
 * Desc    : annotation for apollo
 *****************************************************************************/

#include "modules/perception/lidar/app/lidar_obstacle_detection.h"

#include "modules/perception/lidar/app/proto/lidar_obstacle_detection_config.pb.h"

#include "cyber/common/file.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

bool LidarObstacleDetection::Init(
    const LidarObstacleDetectionInitOptions& options) {
  auto& sensor_name = options.sensor_name;
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;

  /**
   * model_config对应的配置文件
   * modules/perception/production/conf/perception/lidar/
   *    modules/lidar_obstacle_pipeline.config
   * 内容
   * model_configs {
   *  name: "LidarObstacleDetection"
   *  version: "1.0.0"
   *  string_params {
   *    name: "root_path"
   *    value: "./data/perception/lidar/models/lidar_obstacle_pipeline"
   *  }
   * }
   * **/
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));

  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  // root_path="./data/perception/lidar/models/lidar_obstacle_pipeline"
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = cyber::common::GetAbsolutePath(work_root, root_path);
  /**
   * LidarObstacleDetection由DetectionComponent模块调用
   * sensor_name由DetectionComponent的配置文件确定
   * 假设sensor_name="velodyne128"
   * **/
  config_file = cyber::common::GetAbsolutePath(config_file, sensor_name);

  /**
   * 最终config_file指向文件:
   * modules/perception/production/data/perception/lidar/models/
   *    lidar_obstacle_pipeline/velodyne128/lidar_obstacle_detection.conf
   * 内容:
   * detector: "PointPillarsDetection"
   * use_map_manager: true
   * use_object_filter_bank: true
   * **/
  config_file = cyber::common::GetAbsolutePath(config_file,
                                               "lidar_obstacle_detection.conf");

  /**
   * 使用配置文件config_file填充config
   * **/
  LidarObstacleDetectionConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  /**
   * detector_name_="PointPillarsDetection"
   * 由此可见Apollo6.0激光雷达的感知用的是PointPillars
   * **/
  detector_name_ = config.detector();

  /**
   * 预处理选项
   * struct PointCloudPreprocessorInitOptions {
   *    std::string sensor_name = "velodyne64";
   * };
   *
   *
   * DetectionComponent调用该LidarObstacleDetection
   * 并对其进行初始化
   * 根据DetectionComponent模块的配置文件可以确定sensor_name的真实值
   * preprocessor_init_options.sensor_name = sensor_name;
   *
   * 如果DetectionComponent模块使用配置文件
   * "apollo/modules/perception/production/conf/perception/
   *    lidar/velodyne128_detection_conf.pb.txt"
   * 则preprocessor_init_options.sensor_name="velodyne128"
   *
   * **/
  PointCloudPreprocessorInitOptions preprocessor_init_options;
  preprocessor_init_options.sensor_name = sensor_name;

  /**
   * 成员变量 点云预处理类
   * PointCloudPreprocessor cloud_preprocessor_;
   * 在下面文件中定义
   * modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.cc
   * 
   * preprocessor_init_options里面只包含了传感器的名字，
   * cloud_preprocessor_在初始化是会根据这个传感器名字查相应位置的配置文件
   * **/
  ACHECK(cloud_preprocessor_.Init(preprocessor_init_options));

  /**
   * detector_原先的版本是通过配置文件中的segmentor_name_获得实例
   * 注释调的代码是原先的版本
   * 原先版本的代码：
   * modules/perception/lidar/app/lidar_obstacle_segmentation.cc
   *
   * 现在使用了PointPillarsDetection，并没有用到配置文件中的detector_name
   * PointPillarsDetection是目前工业界比较流行的激光雷达检测算法
   * **/
  detector_.reset(new PointPillarsDetection);
  // detector_.reset(
  //    BaseSegmentationRegisterer::GetInstanceByName(segmentor_name_));
  CHECK_NOTNULL(detector_.get());
  DetectionInitOptions detection_init_options;
  // segmentation_init_options.sensor_name = sensor_name;
  ACHECK(detector_->Init(detection_init_options));

  return true;
}

LidarProcessResult LidarObstacleDetection::Process(
    const LidarObstacleDetectionOptions& options, LidarFrame* frame) {
  PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics =
      options.sensor2novatel_extrinsics;
   
  if (cloud_preprocessor_.Preprocess(preprocessor_options, frame)) {
    return ProcessCommon(options, frame);
  }
  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}

/**
 * 在modules/perception/lidar/app/lidar_obstacle_detection.cc的
 * 函数DetectionComponent::InternalProc中
 * lidar::LidarProcessResult ret =
 *     detector_->Process(detect_opts, in_message, frame.get());
 *
 * 所以该实例以及该函数是DetectionComponent模块的具体算法实现
 *
 * LidarObstacleDetection::Process是DetectionComponent算法的入口
 * 
 * 
 * 步骤1： 预处理
 * 代码：cloud_preprocessor_.Preprocess(preprocessor_options, message, frame)
 * 
 * 步骤2 ：进行检测
 * 代码： ProcessCommon(options, frame);
 *    
 * **/
LidarProcessResult LidarObstacleDetection::Process(
    const LidarObstacleDetectionOptions& options,
    const std::shared_ptr<apollo::drivers::PointCloud const>& message,
    LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  PERF_FUNCTION_WITH_INDICATOR(options.sensor_name);

  PERF_BLOCK_START();

  /**
   * PointCloudPreprocessorOptions在下面文件中定义
   * modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h
   * struct PointCloudPreprocessorOptions {
   *    Eigen::Affine3d sensor2novatel_extrinsics;
   * }
   *
   * sensor2novatel_extrinsics描述的是传感器到车自身的位置变换关系
   * **/
  PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics =
      options.sensor2novatel_extrinsics;
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "preprocess");

  /**
   * 对数据进行预处理
   * preprocessor_options只包含了
   * sensor2novatel_extrinsics传感器到车自身的位置变换关系
   * 
   * cloud_preprocessor_.Preprocess筛选点,并变换到世界坐标系下
   * **/
  if (cloud_preprocessor_.Preprocess(preprocessor_options, message, frame)) {
    /**
     * 进行检测
     * **/
    return ProcessCommon(options, frame);
  }

  /**
   *
   * frame->cloud保存lidar坐标系下的点云
   * frame->world_cloud保存世界坐标系下的点云
   * frame->segmented_objects保存检测结果
   * **/
  
  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}



LidarProcessResult LidarObstacleDetection::ProcessCommon(
    const LidarObstacleDetectionOptions& options, LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  PERF_BLOCK_START();

  /**
   * DetectionOptions在下面文件中定义
   * modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars_detection.h
   * struct DetectionOptions {};
   * **/
  DetectionOptions detection_options;

  /**
   * 使用PointPillarsDetection进行detect
   * 具体的实现过程由PointPillarsDetection提供
   * **/
  if (!detector_->Detect(detection_options, frame)) {
    return LidarProcessResult(LidarErrorCode::DetectionError,
                              "Failed to detect.");
  }
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "detection");

  return LidarProcessResult(LidarErrorCode::Succeed);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
