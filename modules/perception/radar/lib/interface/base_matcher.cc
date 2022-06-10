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


#include "modules/perception/radar/lib/interface/base_matcher.h"
#include <numeric>

namespace apollo {
namespace perception {
namespace radar {

double BaseMatcher::s_max_match_distance_ = 2.5;
double BaseMatcher::s_bound_match_distance_ = 10.0;

void BaseMatcher::SetMaxMatchDistance(double dist) {
  s_max_match_distance_ = dist;
}

double BaseMatcher::GetMaxMatchDistance() { return s_max_match_distance_; }

void BaseMatcher::SetBoundMatchDistance(double dist) {
  s_bound_match_distance_ = dist;
}

double BaseMatcher::GetBoundMatchDistance() { return s_bound_match_distance_; }


/**
 * IDMatch实现还是比较简单的
 * 
 * 步骤1：遍历radar_tracks和radar_frame.objects中的所有点
 *    找到他们id相同，且距离小于2.5的点，将其对应关系(索引关系)保存在assignments中
 * 步骤2：记录radar_tracks中哪些点没有被assign
 * 步骤3：记录radar_frame.objects中哪些点没有被assign
 * **/
void BaseMatcher::IDMatch(const std::vector<RadarTrackPtr> &radar_tracks,
                          const base::Frame &radar_frame,
                          std::vector<TrackObjectPair> *assignments,
                          std::vector<size_t> *unassigned_tracks,
                          std::vector<size_t> *unassigned_objects) {
  // track个数
  size_t num_track = radar_tracks.size();

  // 当前检测到的点
  const auto &objects = radar_frame.objects;
  double object_timestamp = radar_frame.timestamp;

  // 检测点个数
  size_t num_obj = objects.size();

  if (num_track == 0 || num_obj == 0) {
    unassigned_tracks->resize(num_track);
    unassigned_objects->resize(num_obj);
    // 填充0,1,2...
    std::iota(unassigned_tracks->begin(), unassigned_tracks->end(), 0);
    std::iota(unassigned_objects->begin(), unassigned_objects->end(), 0);
    return;
  }

  std::vector<bool> track_used(num_track, false);
  std::vector<bool> object_used(num_obj, false);

  // 步骤1
  for (size_t i = 0; i < num_track; ++i) {
    // 得到一个之前track到的点
    const auto &track_object = radar_tracks[i]->GetObsRadar();
    // 得到该点的时间戳
    double track_timestamp = radar_tracks[i]->GetTimestamp();
    if (track_object.get() == nullptr) {
      AERROR << "track_object is not available";
      continue;
    }

    int track_object_track_id = track_object->track_id;

    for (size_t j = 0; j < num_obj; ++j) {
      // 遍历当前帧点的id
      int object_track_id = objects[j]->track_id;

      /**
       * 在HMMatcher的RefinedTrack中
       * 计算两个点之间的距离，然后返回该距离是否小于一个默认值2.5
       * 如果小于，则返回真
       * **/
      if (track_object_track_id == object_track_id &&
          RefinedTrack(track_object, track_timestamp, objects[j],
                       object_timestamp)) {
        assignments->push_back(std::pair<size_t, size_t>(i, j));
        track_used[i] = true;
        object_used[j] = true;
      }
    }
  }
  
  // 步骤2
  for (size_t i = 0; i < track_used.size(); ++i) {
    if (!track_used[i]) {
      unassigned_tracks->push_back(i);
    }
  }

  // 步骤3
  for (size_t i = 0; i < object_used.size(); ++i) {
    if (!object_used[i]) {
      unassigned_objects->push_back(i);
    }
  }
}


/**
 * 由子类重载实现
 * **/
bool BaseMatcher::RefinedTrack(const base::ObjectPtr &track_object,
                               double track_timestamp,
                               const base::ObjectPtr &radar_object,
                               double radar_timestamp) {
  // This function is supposed to return true in the base class.
  // Specific actions can be overrided in derived classes.
  return true;
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
