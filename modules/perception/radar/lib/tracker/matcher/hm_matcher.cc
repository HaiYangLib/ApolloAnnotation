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

#include "modules/perception/radar/lib/tracker/matcher/hm_matcher.h"

#include <string>
#include <utility>

#include "modules/perception/proto/tracker_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace radar {

using cyber::common::GetAbsolutePath;

HMMatcher::HMMatcher() { name_ = "HMMatcher"; }

HMMatcher::~HMMatcher() {}

/**
 * 匈牙利匹配算法初始化
 * **/
bool HMMatcher::Init() {
  auto config_manager = lib::ConfigManager::Instance();

  // name_ = "HMMatcher"
  std::string model_name = name_;

  const lib::ModelConfig *model_config = nullptr;
  AINFO << "matcher name: " << name_;

  /**
   * 变量model_config对应的配置文件
   * modules/perception/production/conf/perception/radar/
   *              modules/hm_matcher.config
   * model_configs {
   * name: "HMMatcher"
   *   version: "1.0.0"
   *   string_params {
   *     name: "root_path"
   *     value: "./data/perception/radar/models/tracker"
   *   }
   * }
   *
   * **/
  if (!config_manager->GetModelConfig(model_name, &model_config)) {
    AERROR << "not found model: " << model_name;
    return false;
  }

  const std::string &work_root = config_manager->work_root();
  std::string root_path;

  // root_path="./data/perception/radar/models/tracker"
  ACHECK(model_config->get_value("root_path", &root_path))
      << "Failed to get value of root_path.";

  std::string config_file;

  config_file = GetAbsolutePath(work_root, root_path);
  /**
   * config_file=modules/perception/production/data/perception/radar/
   *                      models/tracker/hm_matcher.conf
   * 内容：
   * max_match_distance : 2.5
   * bound_match_distance : 10.0
   * **/
  config_file = GetAbsolutePath(config_file, "hm_matcher.conf");
  // get config params
  MatcherConfig config_params;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config_params))
      << "Failed to parse MatcherConfig config file.";
  // max_match_distance= 2.5
  double max_match_distance = config_params.max_match_distance();
  // bound_match_distance=10.0
  double bound_match_distance = config_params.bound_match_distance();

  // s_max_match_distance_ = max_match_distance=2.5
  BaseMatcher::SetMaxMatchDistance(max_match_distance);
  // s_max_match_distance_=bound_match_distance=10.0
  BaseMatcher::SetBoundMatchDistance(bound_match_distance);
  return true;
}

/**
 * @brief match radar objects to tracks
 * @params[IN] radar_tracks: global tracks
 * @params[IN] radar_frame: current radar frame
 * @params[IN] options: matcher options for future use
 * @params[OUT] assignments: matched pair of tracks and measurements
 * @params[OUT] unassigned_tracks: unmatched tracks
 * @params[OUT] unassigned_objects: unmatched objects
 * @return nothing
 *
 * 步骤1：ID匹配，相同id且距离小于2.5直接匹配上
 * 代码：IDMatch(radar_tracks, radar_frame, assignments, unassigned_tracks,
 *                unassigned_objects);
 * 步骤2：计算关联值
 * 代码：TrackObjectPropertyMatch(radar_tracks, radar_frame, assignments,
 *                          unassigned_tracks, unassigned_objects);
 * **/
bool HMMatcher::Match(const std::vector<RadarTrackPtr> &radar_tracks,
                      const base::Frame &radar_frame,
                      const TrackObjectMatcherOptions &options,
                      std::vector<TrackObjectPair> *assignments,
                      std::vector<size_t> *unassigned_tracks,
                      std::vector<size_t> *unassigned_objects) {
  /**
   * IDMatch实现还是比较简单的
   *
   * 1.遍历radar_tracks和radar_frame.objects中的所有点
   *    找到他们id相同，且距离小于2.5的点，将其对应关系(索引关系)保存在assignments中
   * 2.记录radar_tracks中哪些点没有被assign，其下标保存在unassigned_tracks中
   * 3.记录radar_frame.objects中哪些点没有被assign，其下标保存在unassigned_objects中
   * **/
  // 步骤1
  IDMatch(radar_tracks, radar_frame, assignments, unassigned_tracks,
          unassigned_objects);

  /**
   *
   *
   * **/
  // 步骤2
  TrackObjectPropertyMatch(radar_tracks, radar_frame, assignments,
                           unassigned_tracks, unassigned_objects);
  return true;
}

/**
 * 参数：
 * track_object是之前track到的一个点
 * track_timestamp是该点的时间戳
 * radar_object是当前帧中的一个点
 * radar_timestamp是该帧的时间戳
 *
 * **/
bool HMMatcher::RefinedTrack(const base::ObjectPtr &track_object,
                             double track_timestamp,
                             const base::ObjectPtr &radar_object,
                             double radar_timestamp) {
  double dist = 0.5 * DistanceBetweenObs(track_object, track_timestamp,
                                         radar_object, radar_timestamp) +
                0.5 * DistanceBetweenObs(radar_object, radar_timestamp,
                                         track_object, track_timestamp);

  /**
   * 默认值BaseMatcher::GetMaxMatchDistance()=2.5
   * 2.5会不会有些大？
   * **/
  return dist < BaseMatcher::GetMaxMatchDistance();
}

/**
 * 入参介绍：
 * 该函数会被HMMatcher::Match函数调用
 * 在HMMatcher::Match中，先进行ID匹配，将匹配到的点保存在assignments中
 * unassigned_tracks和unassigned_objects是IDMatch没有匹配到的点
 * IDMatch没有匹配到的点并不一定真的不能匹配，所以有了TrackObjectPropertyMatch函数
 * 
 * 分析HMMatcher::TrackObjectPropertyMatch函数步骤
 * 步骤1：计算unassigned_tracks和unassigned_objects对应点的距离，这种距离关系保存在
 * 关联矩阵association_mat
 * 代码：ComputeAssociationMat(radar_tracks, radar_frame, *unassigned_tracks,
 *                        *unassigned_objects, &association_mat);
 * 
 * 步骤2：用表示距离关系表示代价矩阵global_costs
 * 代码： (*global_costs)(i, j) = association_mat[i][j];
 * 
 * 步骤3：进行匹配，在内部使用了Munkres分配算法
 * 
 * 步骤4：追加assignments
 * 
 * 步骤5：更更新unassigned_tracks，unassigned_objects
 * **/
void HMMatcher::TrackObjectPropertyMatch(
    const std::vector<RadarTrackPtr> &radar_tracks,
    const base::Frame &radar_frame, std::vector<TrackObjectPair> *assignments,
    std::vector<size_t> *unassigned_tracks,
    std::vector<size_t> *unassigned_objects) {
  // 是否全匹配上    
  if (unassigned_tracks->empty() || unassigned_objects->empty()) {
    return;
  }

  /**
   * 看到association_mat数组的尺寸就应该知道，关联矩阵association_mat会保存
   * unassigned_tracks到unassigned_objects对应点的距离
   * 代码：association_mat->at(i).at(j) =
   *       0.5 * distance_forward + 0.5 * distance_backward;
   * **/
  std::vector<std::vector<double>> association_mat(unassigned_tracks->size());
  for (size_t i = 0; i < association_mat.size(); ++i) {
    association_mat[i].resize(unassigned_objects->size(), 0);
  }

  // 得到距离关系
  // 步骤1
  ComputeAssociationMat(radar_tracks, radar_frame, *unassigned_tracks,
                        *unassigned_objects, &association_mat);

  // from perception-common
  /**
   * 成员变量common::GatedHungarianMatcher<double> hungarian_matcher_;
   *  
   * 类GatedHungarianMatcher成员变量SecureMat<T> global_costs_;
   * 
   * SecureMat在下面文件中定义
   * modules/perception/common/graph/secure_matrix.h
   * template <typename T>
   * class SecureMat {
   *   Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_;
   *   size_t max_height_ = 1000;
   *   size_t max_width_ = 1000;
   *   size_t height_ = 0;
   *   size_t width_ = 0;
   * };
   * 
   * **/
  // 步骤2
  common::SecureMat<double> *global_costs =
      hungarian_matcher_.mutable_global_costs();

  global_costs->Resize(unassigned_tracks->size(), unassigned_objects->size());

  // 用距离表示代价
  for (size_t i = 0; i < unassigned_tracks->size(); ++i) {
    for (size_t j = 0; j < unassigned_objects->size(); ++j) {
      (*global_costs)(i, j) = association_mat[i][j];
    }
  }

  std::vector<TrackObjectPair> property_assignments;
  std::vector<size_t> property_unassigned_tracks;
  std::vector<size_t> property_unassigned_objects;

  /**
   * BaseMatcher::GetMaxMatchDistance()=2.5
   * BaseMatcher::GetBoundMatchDistance()=10.0
   * 
   * hungarian_matcher_中的global_costs_保存着用距离表示的代价
   * (*global_costs)(i, j) = association_mat[i][j];
   * **/
  // 步骤3
  hungarian_matcher_.Match(
      BaseMatcher::GetMaxMatchDistance(), BaseMatcher::GetBoundMatchDistance(),
      common::GatedHungarianMatcher<double>::OptimizeFlag::OPTMIN,
      &property_assignments, &property_unassigned_tracks,
      &property_unassigned_objects);

  // 步骤4
  for (size_t i = 0; i < property_assignments.size(); ++i) {
    size_t gt_idx = unassigned_tracks->at(property_assignments[i].first);
    size_t go_idx = unassigned_objects->at(property_assignments[i].second);
    assignments->push_back(std::pair<size_t, size_t>(gt_idx, go_idx));
  }

  std::vector<size_t> temp_unassigned_tracks;
  std::vector<size_t> temp_unassigned_objects;
  
  // 步骤5
  for (size_t i = 0; i < property_unassigned_tracks.size(); ++i) {
    size_t gt_idx = unassigned_tracks->at(property_unassigned_tracks[i]);
    temp_unassigned_tracks.push_back(gt_idx);
  }

  for (size_t i = 0; i < property_unassigned_objects.size(); ++i) {
    size_t go_idx = unassigned_objects->at(property_unassigned_objects[i]);
    temp_unassigned_objects.push_back(go_idx);
  }

  // 更更新unassigned_tracks，unassigned_objects
  *unassigned_tracks = temp_unassigned_tracks;
  *unassigned_objects = temp_unassigned_objects;
}

/**
 * 
 * association_mat[i][j]表示i与j的距离
 * **/
void HMMatcher::ComputeAssociationMat(
    const std::vector<RadarTrackPtr> &radar_tracks,
    const base::Frame &radar_frame,
    const std::vector<size_t> &unassigned_tracks,
    const std::vector<size_t> &unassigned_objects,
    std::vector<std::vector<double>> *association_mat) {
  double frame_timestamp = radar_frame.timestamp;

  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    for (size_t j = 0; j < unassigned_objects.size(); ++j) {

      const base::ObjectPtr &track_object =
          radar_tracks[unassigned_tracks[i]]->GetObs();

      const base::ObjectPtr &frame_object =
          radar_frame.objects[unassigned_objects[j]];

      double track_timestamp =
          radar_tracks[unassigned_tracks[i]]->GetTimestamp();

      double distance_forward = DistanceBetweenObs(
          track_object, track_timestamp, frame_object, frame_timestamp);

      double distance_backward = DistanceBetweenObs(
          frame_object, frame_timestamp, track_object, track_timestamp);

      association_mat->at(i).at(j) =
          0.5 * distance_forward + 0.5 * distance_backward;
    }
  }
}

double HMMatcher::DistanceBetweenObs(const base::ObjectPtr &obs1,
                                     double timestamp1,
                                     const base::ObjectPtr &obs2,
                                     double timestamp2) {
  double time_diff = timestamp2 - timestamp1;
  return (obs2->center - obs1->center -
          obs1->velocity.cast<double>() * time_diff)
      .head(2)
      .norm();
}

PERCEPTION_REGISTER_MATCHER(HMMatcher);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
