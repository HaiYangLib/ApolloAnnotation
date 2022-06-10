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


#include "modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars_detection.h"

#include <algorithm>
#include <numeric>
#include <random>

#include <cuda_runtime_api.h>

#include "cyber/common/log.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/params.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::Object;
using base::PointD;
using base::PointF;

PointPillarsDetection::PointPillarsDetection()
    : x_min_(Params::kMinXRange),
      x_max_(Params::kMaxXRange),
      y_min_(Params::kMinYRange),
      y_max_(Params::kMaxYRange),
      z_min_(Params::kMinZRange),
      z_max_(Params::kMaxZRange) {
  if (FLAGS_enable_ground_removal) {
    z_min_ = std::max(z_min_, static_cast<float>(FLAGS_ground_removal_height));
  }
}

// TODO(chenjiahao):
//  specify score threshold and nms over lap threshold for each class.
/**
 * 步骤1： 根据Flags，创建一个PointPillars实例
 * 代码：
 * point_pillars_ptr_.reset(new PointPillars(
 *    FLAGS_reproduce_result_mode, FLAGS_score_threshold,
 *    FLAGS_nms_overlap_threshold, FLAGS_pfe_torch_file, FLAGS_rpn_onnx_file));
 * **/
bool PointPillarsDetection::Init(const DetectionInitOptions& options) {
  /**
   * 创建一个PointPillars实例
   *
   * 参数在文件modules/perception/common/perception_gflags.cc定义
   *
   * DEFINE_bool(reproduce_result_mode, false, "True if preprocess in CPU
   * mode.");
   *
   * DEFINE_double(score_threshold, 0.5, "Classification score threshold.");
   *
   * DEFINE_double(nms_overlap_threshold, 0.5, "Nms overlap threshold.");
   *
   *
   * pytorch训练得到的模型
   * DEFINE_string(pfe_torch_file,
   *            "/apollo/modules/perception/production/data/perception/lidar/"
   *           "models/detection/point_pillars/pfe.pt",
   *           "The path of pillars feature extractor torch file.");
   *
   *
   * DEFINE_string(rpn_onnx_file,
   *          "/apollo/modules/perception/production/data/perception/lidar/"
   *           "models/detection/point_pillars/rpn.onnx",
   *           "The path of RPN onnx file.");
   * **/

  /**
   * @brief Constructor
   * @param[in] reproduce_result_mode 如果为真，则相同输入的输出是可再现的 false
   * @param[in] score_threshold 过滤输出的分数阈值 0.5
   * @param[in] nms_overlap_threshold NMS的IOU阈值 0.5
   * @param[in] pfe_torch_file pytorch训练得到的模型
   * @param[in] rpn_onnx_file ONNX文件路径
   * @details Variables could be changed through point_pillars_detection
   * **/
  point_pillars_ptr_.reset(new PointPillars(
      FLAGS_reproduce_result_mode, FLAGS_score_threshold,
      FLAGS_nms_overlap_threshold, FLAGS_pfe_torch_file, FLAGS_rpn_onnx_file));
  return true;
}

/**
 * PointPillarsDetection的算法入口
 * 由LidarObstacleDetection调用
 *
 * 步骤1： 判断是否启用带系数的点云光束下采样。否
 * 步骤2： 判断启用将点云向下采样到体素网格的质心。否
 * 步骤3： 判断是否启用将前一帧的点云融合到当前帧中。否
 * 步骤4： 判断是否在预处理之前启用洗牌点。否认否
 * 步骤5： 根据x_min_，x_max_等阈值筛选点云，同时将强度信息归一化
 * 代码：CloudToArray(cur_cloud_ptr_, points_array, FLAGS_normalizing_factor);
 * 步骤6： 得到目标box的坐标以及分类
 * 代码： GetObjects(&frame->segmented_objects, frame->lidar2world_pose,
 *              &out_detections, &out_labels);
 * 
 * **/
bool PointPillarsDetection::Detect(const DetectionOptions& options,
                                   LidarFrame* frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }

  // record input cloud and lidar frame

  /**
   * original_cloud_是在激光雷达坐标系下的点云
   * original_world_cloud_是在车自身坐标系下的点云
   * **/
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  lidar_frame_ref_ = frame;

  // check output
  frame->segmented_objects.clear();
  /**
   * modules/perception/common/perception_gflags.cc
   * DEFINE_int32(gpu_id, 0, "The id of gpu used for inference.");
   * **/
  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  Timer timer;

  int num_points;
  /**
   * 成员变量
   * base::PointFCloudPtr cur_cloud_ptr_;
   * **/
  cur_cloud_ptr_ = std::shared_ptr<base::PointFCloud>(
      new base::PointFCloud(*original_cloud_));

  // down sample the point cloud through filtering beams
  /**
   * modules/perception/common/perception_gflags.cc
   * DEFINE_bool(enable_downsample_beams, false,
   *         "Enable down sampling point cloud beams with a factor.");
   *
   * 是否启用带系数的点云光束下采样。
   * **/
  // 步骤1
  if (FLAGS_enable_downsample_beams) {
    base::PointFCloudPtr downsample_beams_cloud_ptr(new base::PointFCloud());
    if (DownSamplePointCloudBeams(original_cloud_, downsample_beams_cloud_ptr,
                                  FLAGS_downsample_beams_factor)) {
      cur_cloud_ptr_ = downsample_beams_cloud_ptr;
    } else {
      AWARN << "Down-sample beams factor must be >= 1. Cancel down-sampling."
               " Current factor: "
            << FLAGS_downsample_beams_factor;
    }
  }

  // down sample the point cloud through filtering voxel grid

  /**
   *
   * modules/perception/common/perception_gflags.cc
   * DEFINE_bool(enable_downsample_pointcloud, false,
   *        "Enable down sampling point cloud into centroids of voxel grid.");
   * 启用将点云向下采样到体素网格的质心。
   * **/
  // 步骤2
  if (FLAGS_enable_downsample_pointcloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    TransformToPCLXYZI(*cur_cloud_ptr_, pcl_cloud_ptr);
    DownSampleCloudByVoxelGrid(
        pcl_cloud_ptr, filtered_cloud_ptr, FLAGS_downsample_voxel_size_x,
        FLAGS_downsample_voxel_size_y, FLAGS_downsample_voxel_size_z);

    // transform pcl point cloud to apollo point cloud
    base::PointFCloudPtr downsample_voxel_cloud_ptr(new base::PointFCloud());
    TransformFromPCLXYZI(filtered_cloud_ptr, downsample_voxel_cloud_ptr);
    cur_cloud_ptr_ = downsample_voxel_cloud_ptr;
  }

  downsample_time_ = timer.toc(true);

  num_points = cur_cloud_ptr_->size();
  AINFO << "num points before fusing: " << num_points;

  // fuse clouds of preceding frames with current cloud
  cur_cloud_ptr_->mutable_points_timestamp()->assign(cur_cloud_ptr_->size(),
                                                     0.0);

  /**
   *
   * DEFINE_bool(enable_fuse_frames, false,
   *       "Enable fusing preceding frames' point cloud into current frame.");
   * 是否启用将前一帧的点云融合到当前帧中
   *
   * DEFINE_int32(num_fuse_frames, 5,
   *       "Number of frames to fuse, including current frame.");
   * 要融合的帧数，包括当前帧。
   * **/
  // 步骤3
  if (FLAGS_enable_fuse_frames && FLAGS_num_fuse_frames > 1) {
    // before fusing
    while (!prev_world_clouds_.empty() &&
           frame->timestamp - prev_world_clouds_.front()->get_timestamp() >
               FLAGS_fuse_time_interval) {
      prev_world_clouds_.pop_front();
    }
    // transform current cloud to world coordinate and save to a new ptr
    base::PointDCloudPtr cur_world_cloud_ptr =
        std::make_shared<base::PointDCloud>();
    for (size_t i = 0; i < cur_cloud_ptr_->size(); ++i) {
      auto& pt = cur_cloud_ptr_->at(i);
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
      trans_point = lidar_frame_ref_->lidar2world_pose * trans_point;
      PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      world_point.intensity = pt.intensity;
      cur_world_cloud_ptr->push_back(world_point);
    }
    cur_world_cloud_ptr->set_timestamp(frame->timestamp);

    // fusing clouds
    for (auto& prev_world_cloud_ptr : prev_world_clouds_) {
      num_points += prev_world_cloud_ptr->size();
    }
    FuseCloud(cur_cloud_ptr_, prev_world_clouds_);

    // after fusing
    while (static_cast<int>(prev_world_clouds_.size()) >=
           FLAGS_num_fuse_frames - 1) {
      prev_world_clouds_.pop_front();
    }
    prev_world_clouds_.emplace_back(cur_world_cloud_ptr);
  }
  AINFO << "num points after fusing: " << num_points;

  fuse_time_ = timer.toc(true);

  // shuffle points and cut off
  /**
   * DEFINE_bool(enable_shuffle_points, false,
   *         "Enable shuffling points before preprocessing.");
   * 是否在预处理之前启用洗牌点。否认否
   * **/
  // 步骤4
  if (FLAGS_enable_shuffle_points) {
    num_points = std::min(num_points, FLAGS_max_num_points);
    std::vector<int> point_indices = GenerateIndices(0, num_points, true);
    base::PointFCloudPtr shuffle_cloud_ptr(
        new base::PointFCloud(*cur_cloud_ptr_, point_indices));
    cur_cloud_ptr_ = shuffle_cloud_ptr;
  }

  shuffle_time_ = timer.toc(true);

  // point cloud to array
  /**
   * DEFINE_int32(num_point_feature, 5,
   *         "Length of raw point feature. Features include x, y, z,"、
   *          "intensity and delta of time.");
   * 原始点特征的长度。特征包括x、y、z、强度和时间差。
   *
   * 申请一个定长数组points_array
   * num_points = cur_cloud_ptr_->size();
   * **/
  float* points_array = new float[num_points * FLAGS_num_point_feature]();

  /**
   * ur_cloud_ptr_, points_array是激光雷达坐标系下的点云
   *
   * DEFINE_double(normalizing_factor, 255,
   *           "Normalize intensity range to [0, 1] by this factor.");
   * CloudToArray(const base::PointFCloudPtr& pc_ptr,
   *                                    float* out_points_array,
   *                                    const float normalizing_factor)
   * CloudToArray将输出结果保存在points_array中
   * **/
  // 步骤5
  CloudToArray(cur_cloud_ptr_, points_array, FLAGS_normalizing_factor);
  cloud_to_array_time_ = timer.toc(true);

  // inference
  std::vector<float> out_detections;
  std::vector<int> out_labels;

  // 步骤6
  point_pillars_ptr_->DoInference(points_array, num_points, &out_detections,
                                  &out_labels);
  inference_time_ = timer.toc(true);

  // transfer output bounding boxes to objects

  /**
   * 得到目标box的坐标以及分类
   * 将结果写在frame->segmented_objects中
   * 
   * **/
  GetObjects(&frame->segmented_objects, frame->lidar2world_pose,
             &out_detections, &out_labels);
  collect_time_ = timer.toc(true);

  AINFO << "PointPillars: "
        << "\n"
        << "down sample: " << downsample_time_ << "\t"
        << "fuse: " << fuse_time_ << "\t"
        << "shuffle: " << shuffle_time_ << "\t"
        << "cloud_to_array: " << cloud_to_array_time_ << "\t"
        << "inference: " << inference_time_ << "\t"
        << "collect: " << collect_time_;

  delete[] points_array;
  return true;
}

/**
 * 
 * static constexpr float kMinXRange = -100.0f;
 * static constexpr float kMinYRange = -70.0f;
 * static constexpr float kMinZRange = -5.0f;
 * static constexpr float kMaxXRange = 100.0f;
 * static constexpr float kMaxYRange = 70.0f;
 * static constexpr float kMaxZRange = 3.0f;
 * x_min_(Params::kMinXRange)
 * x_max_(Params::kMaxXRange)
 * y_min_(Params::kMinYRange)
 * y_max_(Params::kMaxYRange)
 * z_min_(Params::kMinZRange)
 * z_max_(Params::kMaxZRange)
 *
 * 只做一件事：根据系列阈值筛选点云
 * 并将结果保存在out_points_array中，强度信息经过归一化
 * **/
void PointPillarsDetection::CloudToArray(const base::PointFCloudPtr& pc_ptr,
                                         float* out_points_array,
                                         const float normalizing_factor) {
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto& point = pc_ptr->at(i);
    float x = point.x;
    float y = point.y;
    float z = point.z;
    float intensity = point.intensity;
    if (z < z_min_ || z > z_max_ || y < y_min_ || y > y_max_ || x < x_min_ ||
        x > x_max_) {
      continue;
    }
    out_points_array[i * FLAGS_num_point_feature + 0] = x;
    out_points_array[i * FLAGS_num_point_feature + 1] = y;
    out_points_array[i * FLAGS_num_point_feature + 2] = z;
    out_points_array[i * FLAGS_num_point_feature + 3] =
        intensity / normalizing_factor;
    // delta of timestamp between prev and cur frames
    out_points_array[i * FLAGS_num_point_feature + 4] =
        static_cast<float>(pc_ptr->points_timestamp(i));
  }
}

void PointPillarsDetection::FuseCloud(
    const base::PointFCloudPtr& out_cloud_ptr,
    const std::deque<base::PointDCloudPtr>& fuse_clouds) {
  for (auto iter = fuse_clouds.rbegin(); iter != fuse_clouds.rend(); ++iter) {
    double delta_t = lidar_frame_ref_->timestamp - (*iter)->get_timestamp();
    // transform prev world point cloud to current sensor's coordinates
    for (size_t i = 0; i < (*iter)->size(); ++i) {
      auto& point = (*iter)->at(i);
      Eigen::Vector3d trans_point(point.x, point.y, point.z);
      trans_point = lidar_frame_ref_->lidar2world_pose.inverse() * trans_point;
      base::PointF pt;
      pt.x = static_cast<float>(trans_point(0));
      pt.y = static_cast<float>(trans_point(1));
      pt.z = static_cast<float>(trans_point(2));
      pt.intensity = static_cast<float>(point.intensity);
      // delta of time between current and prev frame
      out_cloud_ptr->push_back(pt, delta_t);
    }
  }
}

std::vector<int> PointPillarsDetection::GenerateIndices(int start_index,
                                                        int size,
                                                        bool shuffle) {
  // create a range number array
  std::vector<int> indices(size);
  std::iota(indices.begin(), indices.end(), start_index);

  // shuffle the index array
  if (shuffle) {
    unsigned seed = 0;
    std::shuffle(indices.begin(), indices.end(),
                 std::default_random_engine(seed));
  }
  return indices;
}


/**
 * 步骤1：得到box在lidar坐标系下的坐标
 * 步骤2：得到box在world坐标系下的坐标
 * 步骤3：box对应目标的类型
 * **/
void PointPillarsDetection::GetObjects(
    std::vector<std::shared_ptr<Object>>* objects, const Eigen::Affine3d& pose,
    std::vector<float>* detections, std::vector<int>* labels) {
  /**
   *
   * DEFINE_int32(num_output_box_feature, 7, "Length of output box feature.");
   * **/
  int num_objects = detections->size() / FLAGS_num_output_box_feature;

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  /**
   * 3d box由(x,y,z,w,l,h,theta)确定.　
   * 其中x为图像横向，y为图像纵向
   * 类似于2d box由(x,y,w,h)确定,
   * 3d　box多了一个z方向的数据,以及一个角度,用以预计3d box的朝向(绕z轴的角度).
   *
   * **/
  for (int i = 0; i < num_objects; ++i) {
    auto& object = objects->at(i);
    object->id = i;

    // read params of bounding box
    // x, y, z
    float x = detections->at(i * FLAGS_num_output_box_feature + 0);
    float y = detections->at(i * FLAGS_num_output_box_feature + 1);
    float z = detections->at(i * FLAGS_num_output_box_feature + 2);

    // w,l,h
    float dx = detections->at(i * FLAGS_num_output_box_feature + 4);
    float dy = detections->at(i * FLAGS_num_output_box_feature + 3);
    float dz = detections->at(i * FLAGS_num_output_box_feature + 5);

    // theta
    float yaw = detections->at(i * FLAGS_num_output_box_feature + 6);

    yaw += M_PI / 2;
    yaw = std::atan2(sinf(yaw), cosf(yaw));
    yaw = -yaw;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;

    // compute vertexes of bounding box and transform to world coordinate
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    float roll = 0, pitch = 0;
    Eigen::Quaternionf quater =
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(x, y, z);
    Eigen::Affine3f affine3f = translation * quater.toRotationMatrix();

    /**
     * 一个三维box包含8个点
     * **/
    for (float vx : std::vector<float>{dx / 2, -dx / 2}) {
      for (float vy : std::vector<float>{dy / 2, -dy / 2}) {
        for (float vz : std::vector<float>{0, dz}) {
          /**
           * 
           * roll，pitch，yaw指的是box相对于box中心的旋转
           * dx / 2，dy / 2等是box相对于box中心的坐标
           * 通过affine3f可以将box(六边形，8个点)转换到lidar坐标系下
           * 
           * **/
          // 步骤1
          Eigen::Vector3f v3f(vx, vy, vz);
          v3f = affine3f * v3f;
          PointF point;
          point.x = v3f.x();
          point.y = v3f.y();
          point.z = v3f.z();
          object->lidar_supplement.cloud.push_back(point);

          /**
           * 通过pose：lidar2world_pose
           * 将box(六边形，8个点)转换到世界坐标系下
           * **/
          // 步骤2
          Eigen::Vector3d trans_point(point.x, point.y, point.z);
          trans_point = pose * trans_point;
          PointD world_point;
          world_point.x = trans_point(0);
          world_point.y = trans_point(1);
          world_point.z = trans_point(2);
          object->lidar_supplement.cloud_world.push_back(world_point);
        }
      }
    }

    // classification
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());

    /**
     * 确定类型
     * 一共9种类型，通过枚举定义
     * **/
    // 步骤3
    object->sub_type = GetObjectSubType(labels->at(i));
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
  }
}

// TODO(all): update the base ObjectSubType with more fine-grained types
// TODO(chenjiahao): move types into an array in the same order as offline
base::ObjectSubType PointPillarsDetection::GetObjectSubType(const int label) {
  switch (label) {
    case 0:
      return base::ObjectSubType::BUS;
    case 1:
      return base::ObjectSubType::CAR;
    case 2:  // construction vehicle
      return base::ObjectSubType::UNKNOWN_MOVABLE;
    case 3:
      return base::ObjectSubType::TRUCK;
    case 4:  // barrier
      return base::ObjectSubType::UNKNOWN_UNMOVABLE;
    case 5:
      return base::ObjectSubType::CYCLIST;
    case 6:
      return base::ObjectSubType::MOTORCYCLIST;
    case 7:
      return base::ObjectSubType::PEDESTRIAN;
    case 8:
      return base::ObjectSubType::TRAFFICCONE;
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
