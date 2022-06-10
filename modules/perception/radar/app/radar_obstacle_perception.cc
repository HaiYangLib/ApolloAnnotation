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


#include "modules/perception/radar/app/radar_obstacle_perception.h"

#include "modules/common/util/perf_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/registerer/registerer.h"

using apollo::perception::lib::ConfigManager;
using apollo::perception::lib::ModelConfig;

namespace apollo {
namespace perception {
namespace radar {

/**
 * 步骤1：根据配置文件，获取相关配置项
 * 步骤2：创建实例ContiArsDetector
 * 代码：BaseDetector* detector =
 *      BaseDetectorRegisterer::GetInstanceByName(detector_name);
 * 步骤3：创建实例HdmapRadarRoiFilter
 * 代码： BaseRoiFilter* roi_filter =
 *      BaseRoiFilterRegisterer::GetInstanceByName(roi_filter_name);
 * 步骤4：创建实例ContiArsTracker
 * 代码：BaseTracker* tracker = 
 *        BaseTrackerRegisterer::GetInstanceByName(tracker_name);
 * 步骤5：初始化相关实例
 * 
 * **/
bool RadarObstaclePerception::Init(const std::string& pipeline_name) {
  std::string model_name = pipeline_name;
  const ModelConfig* model_config = nullptr;
  /**
   * pipeline_name由RadarDetectionComponent传入
   * 
   * 假设pipeline_name="FrontRadarPipeline"
   * 
   * 变量model_config对应的配置文件
   * modules/perception/production/conf/perception/radar/modules/
   *                      front_radar_pipeline.config
   * model_configs {
   * name: "FrontRadarPipeline"
   *   version: "1.0.0"
   *   string_params {
   *     name: "Detector"
   *     value: "ContiArsDetector"
   *   }
   *   string_params {
   *     name: "RoiFilter"
   *     value: "HdmapRadarRoiFilter"
   *   }
   *   string_params {
   *     name: "Tracker"
   *     value: "ContiArsTracker"
   *   }
   * }
   * 
   * **/  
  // 步骤1
  ACHECK(ConfigManager::Instance()->GetModelConfig(model_name, &model_config))
      << "not found model: " << model_name;

  /**
   * 得到detector的名字 
   * detector_name="ContiArsDetector"
   * **/
  std::string detector_name;
  ACHECK(model_config->get_value("Detector", &detector_name))
      << "Detector not found";

  /**
   * 得到roi_filter的名字
   * roi_filter="HdmapRadarRoiFilter"
   * **/
  std::string roi_filter_name;
  ACHECK(model_config->get_value("RoiFilter", &roi_filter_name))
      << "RoiFilter not found";

  /**
   * 得到tracker的名字
   * tracker_name="ContiArsTracker"
   * **/
  std::string tracker_name;
  ACHECK(model_config->get_value("Tracker", &tracker_name))
      << "Tracker not found";

  
  /**
   * 根据detector_name得到一个detector实例ContiArsDetector
   * modules/perception/radar/lib/detector/conti_ars_detector/conti_ars_detector.cc
   * 继承自BaseDetector 
   * **/
  // 步骤2
  BaseDetector* detector =
      BaseDetectorRegisterer::GetInstanceByName(detector_name);
      
  CHECK_NOTNULL(detector);
  detector_.reset(detector);

  /**
   * 得到一个roi_filter实例HdmapRadarRoiFilter
   * modules/perception/radar/lib/roi_filter/hdmap_radar_roi_filter/
   *                hdmap_radar_roi_filter.cc
   * 继承自BaseRoiFilter
   * **/
  // 步骤3
  BaseRoiFilter* roi_filter =
      BaseRoiFilterRegisterer::GetInstanceByName(roi_filter_name);
  CHECK_NOTNULL(roi_filter);
  roi_filter_.reset(roi_filter);

  /**
   * 得到一个tracker实例ContiArsTracker
   * modules/perception/radar/lib/tracker/conti_ars_tracker/
   *                  conti_ars_tracker.cc
   * 继承自 BaseTracker 
   * **/
  // 步骤4
  BaseTracker* tracker = BaseTrackerRegisterer::GetInstanceByName(tracker_name);
  CHECK_NOTNULL(tracker);
  tracker_.reset(tracker);
 
  /**
   * 对detector，roi_filter，tracker初始化
   * detector，roi_filter初始ua比较简单，几乎没做什么事情
   * tracker_初始化比较复杂：
   * 1.根据配置文件为成员变量赋值
   * 2.创建匈牙利匹配算法实例，并初始化
   * 3.创建实例RadarTrackManager
   * **/
  // 步骤5
  ACHECK(detector_->Init()) << "radar detector init error";
  ACHECK(roi_filter_->Init()) << "radar roi filter init error";
  ACHECK(tracker_->Init()) << "radar tracker init error";

  return true;
}

/**
 * 步骤1：进行检测，对于Radar所谓的检测指的是将点云从Radar坐标系向世界坐标系转换
 * 代码：detector_->Detect(corrected_obstacles, options.detector_options,
 *                        detect_frame_ptr)
 * 
 * 步骤2：感兴趣区域过滤
 * 代码：roi_filter_->RoiFilter(options.roi_filter_options, detect_frame_ptr)
 * 
 * 步骤3：追踪，tracker_frame_ptr中保存这track的点（不只是当前帧的点）
 * 代码：tracker_->Track(*detect_frame_ptr, options.track_options,
 *            tracker_frame_ptr)
 * 
 * 步骤4：整理感知结果
 * 代码：*objects = tracker_frame_ptr->objects;
 * **/
bool RadarObstaclePerception::Perceive(
    const drivers::ContiRadar& corrected_obstacles,
    const RadarPerceptionOptions& options,
    std::vector<base::ObjectPtr>* objects) {
  PERF_FUNCTION();
  const std::string& sensor_name = options.sensor_name;
  PERF_BLOCK_START();
  base::FramePtr detect_frame_ptr(new base::Frame());

  /**
   * detector_指向ContiArsDetector
   * 对于Radar所谓的检测指的是将点云从Radar坐标系向世界坐标系转换
   * **/
  // 步骤1
  if (!detector_->Detect(corrected_obstacles, options.detector_options,
                         detect_frame_ptr)) {
    AERROR << "radar detect error";
    return false;
  }
  ADEBUG << "Detected frame objects number: "
         << detect_frame_ptr->objects.size();
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "detector");

  /**
   * roi_filter_指向HdmapRadarRoiFilter
   * 感兴趣区域过滤
   * **/
  // 步骤2
  // TODO 注解
  if (!roi_filter_->RoiFilter(options.roi_filter_options, detect_frame_ptr)) {
    ADEBUG << "All radar objects were filtered out";
  }
  ADEBUG << "RoiFiltered frame objects number: "
         << detect_frame_ptr->objects.size();
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "roi_filter");

  /**
   * tracker_指向ContiArsTracker
   * 追踪，tracker_frame_ptr中保存这track的点（不只是当前帧的点）
   * **/
  // 步骤3 
  base::FramePtr tracker_frame_ptr(new base::Frame);
  if (!tracker_->Track(*detect_frame_ptr, options.track_options,
                       tracker_frame_ptr)) {
    AERROR << "radar track error";
    return false;
  }
  ADEBUG << "tracked frame objects number: "
         << tracker_frame_ptr->objects.size();
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "tracker");

  /**
   * 整理感知结果
   * **/
  // 步骤4
  *objects = tracker_frame_ptr->objects;

  return true;
}

std::string RadarObstaclePerception::Name() const {
  return "RadarObstaclePerception";
}

PERCEPTION_REGISTER_RADAR_OBSTACLE_PERCEPTION(RadarObstaclePerception);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
