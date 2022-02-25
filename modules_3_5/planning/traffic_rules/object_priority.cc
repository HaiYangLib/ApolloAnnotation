/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/traffic_rules/object_priority.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

ObjectPriority::ObjectPriority(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

Status ObjectPriority::ApplyRule(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
