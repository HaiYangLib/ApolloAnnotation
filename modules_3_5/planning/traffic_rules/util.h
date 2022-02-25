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

/**
 * @file util.h
 **/

#pragma once

#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {
namespace util {

double GetADCStopDeceleration(ReferenceLineInfo* const reference_line_info,
                              const double stop_line_s,
                              const double min_pass_s_distance);

}  // util
}  // namespace planning
}  // namespace apollo
