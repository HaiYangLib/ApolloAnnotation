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

#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"

#include <limits>

#include "modules/perception/lidar/lib/pointcloud_preprocessor/proto/pointcloud_preprocessor_config.pb.h"

#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;

const float PointCloudPreprocessor::kPointInfThreshold = 1e3;


/**
 * 只做一件事，从配置文件中得到参数并给成员变量赋值
 * 
 * **/
bool PointCloudPreprocessor::Init(
    const PointCloudPreprocessorInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  /**
   * 得到配置参数
   * Name()="PointCloudPreprocessor"
   *
   * PointCloudPreprocessor的配置文件内容为
   * model_configs {
   *    name: "PointCloudPreprocessor"
   *    version: "1.0.0"
   *    string_params {
   *        name: "root_path"
   *        value: "./data/perception/lidar/models/pointcloud_preprocessor"
   *    }
   * }
   * **/
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));

  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;

  // root_path="./data/perception/lidar/models/pointcloud_preprocessor"
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, options.sensor_name);

  /**
   * LidarObstacleDetection调用该PointCloudPreprocessor
   * 并对PointCloudPreprocessor初始化
   * options.sensor_name是传入参数由LidarObstacleDetection创建
   * LidarObstacleDetection被DetectionComponent模块调用
   * options.sensor_name是LidarObstacleDetection根据DetectionComponent模块的创建的
   * 所以options.sensor_name由DetectionComponent模块根据根据其配置文件的确定
   * 假设为sensor_name: "velodyne128"
   *
   * config_file=
   *    "modules/perception/production/data/perception/lidar/models/
   *              pointcloud_preprocessor/velodyne128/pointcloud_preprocessor.conf"
   * 内容:
   * filter_naninf_points: false
   * filter_nearby_box_points: true
   * box_forward_x: 2.0
   * box_backward_x: -2.0
   * box_forward_y: 3.0
   * box_backward_y: -5.0
   * filter_high_z_points: false
   * z_threshold: 5.0
   *
   * **/
  config_file = GetAbsolutePath(config_file, "pointcloud_preprocessor.conf");
  PointCloudPreprocessorConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config));

  // filter_naninf_points_=false
  filter_naninf_points_ = config.filter_naninf_points();
  // filter_nearby_box_points_=true
  filter_nearby_box_points_ = config.filter_nearby_box_points();
  box_forward_x_ = config.box_forward_x();
  box_backward_x_ = config.box_backward_x();
  box_forward_y_ = config.box_forward_y();
  box_backward_y_ = config.box_backward_y();

  /*const auto &vehicle_param =
    common::VehicleConfigHelper::GetConfig().vehicle_param();
  box_forward_x_ = static_cast<float>(vehicle_param.right_edge_to_center());
  box_backward_x_ = static_cast<float>(-vehicle_param.left_edge_to_center());
  box_forward_y_ = static_cast<float>(vehicle_param.front_edge_to_center());
  box_backward_y_ = static_cast<float>(-vehicle_param.back_edge_to_center());*/

  filter_high_z_points_ = config.filter_high_z_points();
  z_threshold_ = config.z_threshold();

  return true;
}

/**
 * 步骤1： 经过坐标系变换得到每个点在车自身坐标系下的位置
 * 步骤2： 对经过坐标变换的点进行筛选
 * 步骤3： 将筛选后的点变换到世界坐标系下
 * 
 * frame->cloud保存lidar坐标系下的点云
 * frame->world_cloud保存世界坐标系下的点云
 * **/
bool PointCloudPreprocessor::Preprocess(
    const PointCloudPreprocessorOptions& options,
    const std::shared_ptr<apollo::drivers::PointCloud const>& message,
    LidarFrame* frame) const {
  if (frame == nullptr) {
    return false;
  }
  if (frame->cloud == nullptr) {
    frame->cloud = base::PointFCloudPool::Instance().Get();
  }
  if (frame->world_cloud == nullptr) {
    frame->world_cloud = base::PointDCloudPool::Instance().Get();
  }
  frame->cloud->set_timestamp(message->measurement_time());
  if (message->point_size() > 0) {
    frame->cloud->reserve(message->point_size());
    base::PointF point;

    for (int i = 0; i < message->point_size(); ++i) {
      const apollo::drivers::PointXYZIT& pt = message->point(i);
      // filter_naninf_points_=false
      if (filter_naninf_points_) {
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
          continue;
        }
        if (fabs(pt.x()) > kPointInfThreshold ||
            fabs(pt.y()) > kPointInfThreshold ||
            fabs(pt.z()) > kPointInfThreshold) {
          continue;
        }
      }

      // 步骤1
      /**
       * 将激光雷达自身自身坐标系下的一个点vec3d_lidar
       * 转换到车自身坐标系vec3d_novatel
       * **/
      Eigen::Vector3d vec3d_lidar(pt.x(), pt.y(), pt.z());
      Eigen::Vector3d vec3d_novatel =
          options.sensor2novatel_extrinsics * vec3d_lidar;

      // 步骤2
      /**
       * filter_nearby_box_points: true
       * box_forward_x: 2.0
       * box_backward_x: -2.0
       * box_forward_y: 3.0
       * box_backward_y: -5.0
       *
       * Novatel坐标系定义为X轴向右、Y轴向前、Z轴向上。
       * box_forward_x
       * box_backward_x
       * box_forward_y
       * box_backward_y
       * 分别是车的左右,前后
       * box_forward_y: 3.0 绝对值小于box_backward_y: -5.0
       * 难道是 Novatel在前轴？
       * **/
      /**
       * 如果filter_nearby_box_points设置为true
       * 车体包围框以内(box以内)的点会被筛选掉
       * **/
      if (filter_nearby_box_points_ && vec3d_novatel[0] < box_forward_x_ &&
          vec3d_novatel[0] > box_backward_x_ &&
          vec3d_novatel[1] < box_forward_y_ &&
          vec3d_novatel[1] > box_backward_y_) {
        continue;
      }

      /**
       * filter_high_z_points: false
       * z_threshold: 5.0
       *
       * 如果filter_high_z_points设置为true
       * 超过5.0米的点将会被筛选掉
       * **/
      if (filter_high_z_points_ && pt.z() > z_threshold_) {
        continue;
      }

      point.x = pt.x();
      point.y = pt.y();
      point.z = pt.z();

      point.intensity = static_cast<float>(pt.intensity());

      /**
       * inline void push_back(const PointT& point, double timestamp,
       *                float height = std::numeric_limits<float>::max()，
       *                 int32_t beam_id = -1, uint8_t label = 0)
       * **/
      frame->cloud->push_back(point, static_cast<double>(pt.timestamp()) * 1e-9,
                              std::numeric_limits<float>::max(), i, 0);
    }

    /**
     * TransformCloud(
     *    const base::PointFCloudPtr& local_cloud, const Eigen::Affine3d& pose,
     *    base::PointDCloudPtr world_cloud)
     * 对每个点进行坐标系变换得到每个点在世界坐标系下的位置
     * 将结果存放在frame->world_cloud
     * **/
    // 步骤3 
    TransformCloud(frame->cloud, frame->lidar2world_pose, frame->world_cloud);
  }
  return true;
}

bool PointCloudPreprocessor::Preprocess(
    const PointCloudPreprocessorOptions& options, LidarFrame* frame) const {
  if (frame == nullptr || frame->cloud == nullptr) {
    return false;
  }
  if (frame->world_cloud == nullptr) {
    frame->world_cloud = base::PointDCloudPool::Instance().Get();
  }
  if (frame->cloud->size() > 0) {
    size_t size = frame->cloud->size();
    size_t i = 0;
    while (i < size) {
      auto& pt = frame->cloud->at(i);
      // filter_naninf_points_=false
      if (filter_naninf_points_) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
          frame->cloud->SwapPoint(i, size--);
          continue;
        }
        if (fabs(pt.x) > kPointInfThreshold ||
            fabs(pt.y) > kPointInfThreshold ||
            fabs(pt.z) > kPointInfThreshold) {
          frame->cloud->SwapPoint(i, size--);
          continue;
        }
      }

      /**
       * 将激光雷达自身自身坐标系下的一个点vec3d_lidar
       * 转换到车自身坐标系vec3d_novatel
       * **/
      Eigen::Vector3d vec3d_lidar(pt.x, pt.y, pt.z);
      Eigen::Vector3d vec3d_novatel =
          options.sensor2novatel_extrinsics * vec3d_lidar;

      if (filter_nearby_box_points_ && vec3d_novatel[0] < box_forward_x_ &&
          vec3d_novatel[0] > box_backward_x_ &&
          vec3d_novatel[1] < box_forward_y_ &&
          vec3d_novatel[1] > box_backward_y_) {
        frame->cloud->SwapPoint(i, size--);
        continue;
      }
      if (filter_high_z_points_ && pt.z > z_threshold_) {
        frame->cloud->SwapPoint(i, size--);
        continue;
      }
      ++i;
    }
    frame->cloud->resize(i);
    TransformCloud(frame->cloud, frame->lidar2world_pose, frame->world_cloud);
    AINFO << "Preprocessor filter points: " << size << " to " << i;
  }
  return true;
}

bool PointCloudPreprocessor::TransformCloud(
    const base::PointFCloudPtr& local_cloud, const Eigen::Affine3d& pose,
    base::PointDCloudPtr world_cloud) const {
  if (local_cloud == nullptr) {
    return false;
  }
  world_cloud->clear();
  world_cloud->reserve(local_cloud->size());
  for (size_t i = 0; i < local_cloud->size(); ++i) {
    auto& pt = local_cloud->at(i);
    Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
    trans_point = pose * trans_point;
    base::PointD world_point;
    world_point.x = trans_point(0);
    world_point.y = trans_point(1);
    world_point.z = trans_point(2);
    world_point.intensity = pt.intensity;
    world_cloud->push_back(world_point, local_cloud->points_timestamp(i),
                           std::numeric_limits<float>::max(),
                           local_cloud->points_beam_id()[i], 0);
  }
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
