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
 * @file discretized_path.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

/**
 * message PathPoint {
  // coordinates
  optional double x = 1;
  optional double y = 2;
  optional double z = 3;

  // direction on the x-y plane
  optional double theta = 4;
  // curvature on the x-y planning
  optional double kappa = 5;
  // accumulated distance from beginning of the path
  optional double s = 6;

  // derivative of kappa w.r.t s.
  optional double dkappa = 7;
  // derivative of derivative of kappa w.r.t s.
  optional double ddkappa = 8;
  // The lane ID where the path point is on
  optional string lane_id = 9;

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
  optional double x_derivative = 10;
  optional double y_derivative = 11;
  }
 * 
 * **/
class DiscretizedPath : public std::vector<common::PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<common::PathPoint> path_points);

  double Length() const;

  common::PathPoint Evaluate(const double path_s) const;

  common::PathPoint EvaluateReverse(const double path_s) const;

 protected:
  std::vector<common::PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;
  std::vector<common::PathPoint>::const_iterator QueryUpperBound(
      const double path_s) const;
};

}  // namespace planning
}  // namespace apollo
