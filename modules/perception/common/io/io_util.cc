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
#include "modules/perception/common/io/io_util.h"

#include <boost/filesystem.hpp>

#include "absl/strings/match.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/camera.h"

namespace apollo {
namespace perception {
namespace common {

using cyber::common::PathExists;

bool ReadPoseFile(const std::string &filename, Eigen::Affine3d *pose,
                  int *frame_id, double *time_stamp) {
  if (pose == nullptr || frame_id == nullptr || time_stamp == nullptr) {
    AERROR << "Nullptr error.";
    return false;
  }

  std::ifstream fin(filename.c_str());
  if (!fin.is_open()) {
    AERROR << "Failed to open pose file: " << filename;
    return false;
  }

  Eigen::Vector3d translation;
  Eigen::Quaterniond quat;
  fin >> *frame_id >> *time_stamp >> translation(0) >> translation(1) >>
      translation(2) >> quat.x() >> quat.y() >> quat.z() >> quat.w();

  *pose = Eigen::Affine3d::Identity();
  pose->prerotate(quat);
  pose->pretranslate(translation);

  fin.close();
  return true;
}

/**
 * 步骤1：加载相机内参文件
 * 步骤2：给畸变模型设置参数
 * **/
bool LoadBrownCameraIntrinsic(const std::string &yaml_file,
                              base::BrownCameraDistortionModel *model) {
  if (!PathExists(yaml_file) || model == nullptr) {
    return false;
  }

  /**
   * yaml_file：
   * modules/perception/data/params/front_6mm_intrinsics.yaml
   * modules/perception/data/params/front_12mm_intrinsics.yaml
   *
   * 以front_6mm_intrinsics.yaml为例：
   * 
   *   frame_id: front_6mm
   * height: 1080
   * width: 1920
   * distortion_model: plumb_bob
   * 
   * 畸变参数 径向和切向
   * D: [-0.404512764, 0.287894705, -1.92487504e-03, 2.69998475e-04,
   *                                                 -3.25939801e-01]
   * 
   * 图像坐标系转换为像素坐标系
   * K: [1983.97376, 0.0, 998.341216, 0.0, 1981.62916, 621.618227, 0.0,
   *                                                  0.0, 1.0]
   * R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
   * 
   * P: [1983.97376,0.0, 998.341216, 0.0, 0.0, 1981.62916,
   *                               621.618227, 0.0, 0.0, 0.0, 1.0, 0.0]
   * binning_x: 0
   * binning_y: 0
   * roi:
   *   x_offset: 0
   *   y_offset: 0
   *   height: 0
   *   width: 0
   *   do_rectify: False
   *
   *
   * **/

  // 步骤1
  YAML::Node node = YAML::LoadFile(yaml_file);
  if (node.IsNull()) {
    AINFO << "Load " << yaml_file << " failed! please check!";
    return false;
  }

  float camera_width = 0.0f;
  float camera_height = 0.0f;
  Eigen::VectorXf params(9 + 5);
  try {
    camera_width = node["width"].as<float>();
    camera_height = node["height"].as<float>();
    for (size_t i = 0; i < 9; ++i) {
      params(i) = node["K"][i].as<float>();
    }
    for (size_t i = 0; i < 5; ++i) {
      params(9 + i) = node["D"][i].as<float>();
    }

    // 步骤2
    model->set_params(static_cast<size_t>(camera_width),
                      static_cast<size_t>(camera_height), params);
  } catch (YAML::Exception &e) {
    AERROR << "load camera intrisic file " << yaml_file
           << " with error, YAML exception: " << e.what();
    return false;
  }

  return true;
}

bool LoadOmnidirectionalCameraIntrinsics(
    const std::string &yaml_file,
    base::OmnidirectionalCameraDistortionModel *model) {
  if (!PathExists(yaml_file) || model == nullptr) {
    return false;
  }

  YAML::Node node = YAML::LoadFile(yaml_file);
  if (node.IsNull()) {
    AINFO << "Load " << yaml_file << " failed! please check!";
    return false;
  }

  if (!node["width"].IsDefined() || !node["height"].IsDefined() ||
      !node["center"].IsDefined() || !node["affine"].IsDefined() ||
      !node["cam2world"].IsDefined() || !node["world2cam"].IsDefined() ||
      !node["focallength"].IsDefined() || !node["principalpoint"].IsDefined()) {
    AINFO << "Invalid intrinsics file for an omnidirectional camera.";
    return false;
  }

  try {
    int camera_width = 0;
    int camera_height = 0;

    std::vector<float> params;  // center|affine|f|p|i,cam2world|j,world2cam

    camera_width = node["width"].as<int>();
    camera_height = node["height"].as<int>();

    params.push_back(node["center"]["x"].as<float>());
    params.push_back(node["center"]["y"].as<float>());

    params.push_back(node["affine"]["c"].as<float>());
    params.push_back(node["affine"]["d"].as<float>());
    params.push_back(node["affine"]["e"].as<float>());

    params.push_back(node["focallength"].as<float>());
    params.push_back(node["principalpoint"]["x"].as<float>());
    params.push_back(node["principalpoint"]["y"].as<float>());

    params.push_back(static_cast<float>(node["cam2world"].size()));

    for (size_t i = 0; i < node["cam2world"].size(); ++i) {
      params.push_back(node["cam2world"][i].as<float>());
    }

    params.push_back(static_cast<float>(node["world2cam"].size()));

    for (size_t i = 0; i < node["world2cam"].size(); ++i) {
      params.push_back(node["world2cam"][i].as<float>());
    }

    Eigen::VectorXf eigen_params(params.size());
    for (size_t i = 0; i < params.size(); ++i) {
      eigen_params(i) = params[i];
    }

    model->set_params(camera_width, camera_height, eigen_params);
  } catch (YAML::Exception &e) {
    AERROR << "load camera intrisic file " << yaml_file
           << " with error, YAML exception: " << e.what();
    return false;
  }

  return true;
}

bool GetFileList(const std::string &path, const std::string &suffix,
                 std::vector<std::string> *files) {
  if (!PathExists(path)) {
    AINFO << path << " not exist.";
    return false;
  }

  boost::filesystem::recursive_directory_iterator itr(path);
  while (itr != boost::filesystem::recursive_directory_iterator()) {
    try {
      if (absl::EndsWith(itr->path().string(), suffix)) {
        files->push_back(itr->path().string());
      }
      ++itr;
    } catch (const std::exception &ex) {
      AWARN << "Caught execption: " << ex.what();
      continue;
    }
  }
  return true;
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
