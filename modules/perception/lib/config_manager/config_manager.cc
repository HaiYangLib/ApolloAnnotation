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


#include "modules/perception/lib/config_manager/config_manager.h"

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lib {

using cyber::common::GetAbsolutePath;
using cyber::common::GetProtoFromASCIIFile;

ConfigManager::ConfigManager() {

  /**
   * FLAGS_work_root下面文件中定义
   * modules/perception/common/perception_gflags.cc
   * 
   * DEFINE_string(work_root, "", "Project work root direcotry.");
   * **/
  work_root_ = FLAGS_work_root;

  // For start at arbitrary path
  if (work_root_.empty()) {
    work_root_ = cyber::common::GetEnv("MODULE_PATH");
    if (work_root_.empty()) {
      work_root_ = cyber::common::GetEnv("CYBER_PATH");
    }
  }
}

bool ConfigManager::Init() {
  MutexLock lock(&mutex_);
  return InitInternal();
}

bool ConfigManager::InitInternal() {
  if (inited_) {
    return true;
  }
  for (auto iter = model_config_map_.begin(); iter != model_config_map_.end();
       ++iter) {
    delete iter->second;
  }
  model_config_map_.clear();

  /**
   * FLAGS_config_manager_path下面文件中定义
   * modules/perception/common/perception_gflags.cc
   * 
   * DEFINE_string(config_manager_path, "./conf", "The ModelConfig config paths.");
   * 
   * config_module_path=modules/perception/production/conf
   * **/
  std::string config_module_path =
      GetAbsolutePath(work_root_, FLAGS_config_manager_path);
  AINFO << "WORK_ROOT: " << work_root_
        << " config_root_path: " << config_module_path;

  /**
   * common::GetFileList会递归的遍历modules/perception/production/conf
   * 得到所有名字为"config_manager.config"文件路径
   * 如:
   * modules/perception/production/conf/perception/lidar/config_manager.config
   * modules/perception/production/conf/perception/radar/config_manager.config
   * 
   * 并将他们保存在model_config_files中
   * **/
  std::vector<std::string> model_config_files;
  if (!common::GetFileList(config_module_path, "config_manager.config",
                           &model_config_files)) {
    AERROR << "config_root_path : " << config_module_path
           << " get file list error.";
    return false;
  }


  /**
   * ModelConfigFileListProto对应的proto在下面文件中定义
   * modules/perception/proto/perception_config_schema.proto
   * message ModelConfigFileListProto {
   *    repeated string model_config_path = 1;
   * }
   * 
   * 以lidar为例其配置信息下面文件中定义
   * modules/perception/production/conf/perception/lidar/config_manager.config
   * 内容指明了model_config的实际路径 核心思想是把配置文件的路径写到两一个配置文件中
   * model_config_path: "./conf/perception/lidar/modules/map_manager.config"
   * #model_config_path: "./conf/perception/lidar/modules/scene_manager.config"
   * model_config_path: "./conf/perception/lidar/modules/fused_classifier.config"
   * model_config_path: "./conf/perception/lidar/modules/multi_lidar_fusion.config"
   * 
   * 
   * 
   * message MultiModelConfigProto {
   *    repeated ModelConfigProto model_configs = 1;
   * }
   * 
   * message ModelConfigProto {
   *    optional string name = 1;
   *    optional string version = 2;
   *    repeated KeyValueInt integer_params = 3;
   *    repeated KeyValueString string_params = 4;
   *    repeated KeyValueDouble double_params = 5;
   *    repeated KeyValueFloat float_params = 6;
   *    repeated KeyValueBool bool_params = 7;
   *    repeated KeyValueArrayInt array_integer_params = 8;
   *    repeated KeyValueArrayString array_string_params = 9;
   *    repeated KeyValueArrayDouble array_double_params = 10;
   *    repeated KeyValueArrayFloat array_float_params = 11;  
   *    repeated KeyValueArrayBool array_bool_params = 12;
   * }
   * **/

  /**
   * 步骤1.遍历model_config_files数组
   * 代码： for (const auto &model_config_file : model_config_files) 
   * 填充ModelConfigFileListProto
   * 代码：  
   *    ModelConfigFileListProto file_list_proto;
   *    GetProtoFromASCIIFile(model_config_file, &file_list_proto)  
   * 
   * 步骤2. 然后遍历file_list_proto.model_config_path()
   * 代码： for (const std::string &model_config_path : 
   *                                    file_list_proto.model_config_path())
   * 
   * 填充MultiModelConfigProto
   * 代码： MultiModelConfigProto multi_model_config_proto;
   *       GetProtoFromASCIIFile(abs_path, &multi_model_config_proto)
   * 
   * 步骤3. 遍历 multi_model_config_proto.model_configs()
   * 代码 ： for (const ModelConfigProto &model_config_proto :
   *                                multi_model_config_proto.model_configs()) 
   * 得到最终的ModelConfigProto
   * 并用其model_config_proto填充自定义数据类型ModelConfig(调用ModelConfig成员函数Reset实现)
   * ModelConfig成员函数Reset会解析model_config_proto并填充model_config_proto
   * 代码： model_config->Reset(model_config_proto)
   * 
   * 步骤4. 最终得到添加一个map 的键-值
   * auto result =
   *         model_config_map_.emplace(model_config->name(), model_config);
   * 
   * 用户可以通过模块的名字(model_config->name())得到其对应的配置(model_config)
   * 在通过model_config中的重载的各种get_value方法得到某一项配置的值
   * 
   * 由此可见ConfigManager管理的所有配置文件都在
   * config_module_path=modules/perception/production/conf的各级子文件
   * "config_manager.config"中
   * **/

  // 步骤1
  for (const auto &model_config_file : model_config_files) {
    ModelConfigFileListProto file_list_proto;
    if (!GetProtoFromASCIIFile(model_config_file, &file_list_proto)) {
      AERROR << "Invalid ModelConfigFileListProto file: " << model_config_file;
      return false;
    }
    //步骤2
    for (const std::string &model_config_path :
         file_list_proto.model_config_path()) {
      const std::string abs_path =
          GetAbsolutePath(work_root_, model_config_path);
      MultiModelConfigProto multi_model_config_proto;
      if (!GetProtoFromASCIIFile(abs_path, &multi_model_config_proto)) {
        AERROR << "Invalid MultiModelConfigProto file: " << abs_path;
        return false;
      }
      //步骤3
      for (const ModelConfigProto &model_config_proto :
           multi_model_config_proto.model_configs()) {
        ModelConfig *model_config = new ModelConfig();
        if (!model_config->Reset(model_config_proto)) {
          return false;
        }

        AINFO << "load ModelConfig succ. name: " << model_config->name();
        // 步骤4
        auto result =
            model_config_map_.emplace(model_config->name(), model_config);
        if (!result.second) {
          AWARN << "duplicate ModelConfig, name: " << model_config->name();
          return false;
        }
      }
    }
  }

  AINFO << "finish to load ModelConfigs. NumModels: "
        << model_config_map_.size();

  inited_ = true;

  return true;
}

bool ConfigManager::Reset() {
  MutexLock lock(&mutex_);
  inited_ = false;
  return InitInternal();
}

bool ConfigManager::GetModelConfig(const std::string &model_name,
                                   const ModelConfig **model_config) {
  if (!inited_ && !Init()) {
    return false;
  }

  auto citer = model_config_map_.find(model_name);
  if (citer == model_config_map_.end()) {
    return false;
  }
  *model_config = citer->second;
  return true;
}

ConfigManager::~ConfigManager() {
  for (auto iter = model_config_map_.begin(); iter != model_config_map_.end();
       ++iter) {
    delete iter->second;
  }
}

bool ModelConfig::Reset(const ModelConfigProto &proto) {
  name_ = proto.name();
  version_ = proto.version();

  integer_param_map_.clear();
  string_param_map_.clear();
  double_param_map_.clear();
  float_param_map_.clear();
  bool_param_map_.clear();
  array_integer_param_map_.clear();
  array_string_param_map_.clear();
  array_double_param_map_.clear();
  array_float_param_map_.clear();
  array_bool_param_map_.clear();

  for (const KeyValueInt &pair : proto.integer_params()) {
    integer_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueString &pair : proto.string_params()) {
    string_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueDouble &pair : proto.double_params()) {
    double_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueFloat &pair : proto.float_params()) {
    float_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueBool &pair : proto.bool_params()) {
    bool_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueArrayInt &pair : proto.array_integer_params()) {
    std::vector<int> values;
    RepeatedToVector(pair.values(), &values);
    array_integer_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayString &pair : proto.array_string_params()) {
    std::vector<std::string> values;
    values.reserve(pair.values_size());
    for (const std::string &value : pair.values()) {
      values.push_back(value);
    }
    array_string_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayDouble &pair : proto.array_double_params()) {
    std::vector<double> values;
    RepeatedToVector(pair.values(), &values);
    array_double_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayFloat &pair : proto.array_float_params()) {
    std::vector<float> values;
    RepeatedToVector(pair.values(), &values);
    array_float_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayBool &pair : proto.array_bool_params()) {
    std::vector<bool> values;
    RepeatedToVector(pair.values(), &values);
    array_bool_param_map_.emplace(pair.name(), values);
  }

  AINFO << "reset ModelConfig. model_name: " << name_
        << " integer_param_map's size: " << integer_param_map_.size()
        << " string_param_map's size: " << string_param_map_.size()
        << " double_param_map's size: " << double_param_map_.size()
        << " float_param_map's size: " << float_param_map_.size()
        << " bool_param_map's size: " << bool_param_map_.size()
        << " array_integer_param_map's size: "
        << array_integer_param_map_.size()
        << " array_string_param_map's size: " << array_string_param_map_.size()
        << " array_double_param_map's size: " << array_double_param_map_.size()
        << " array_float_param_map's size: " << array_float_param_map_.size()
        << " array_bool_param_map's size: " << array_bool_param_map_.size();

  return true;
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
