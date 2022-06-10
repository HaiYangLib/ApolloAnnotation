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

/******************************************************************************
* AnnotationAuthor  : HaiYang
* Email   : hanhy20@mails.jlu.edu.cn
* Desc    : annotation for apollo
******************************************************************************/


/**
 * @file
 **/

#include "modules/planning/lattice/trajectory_generation/trajectory1d_generator.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory1d/constant_deceleration_trajectory1d.h"
#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/common/trajectory1d/standing_still_trajectory1d.h"
#include "modules/planning/lattice/trajectory_generation/lateral_osqp_optimizer.h"
#include "modules/planning/lattice/trajectory_generation/lateral_qp_optimizer.h"

namespace apollo {
namespace planning {

// A common function for trajectory bundles generation with
// a given initial state and  end conditions.
typedef std::array<double, 3> State;
typedef std::pair<State, double> Condition;
typedef std::vector<std::shared_ptr<Curve1d>> Trajectory1DBundle;

Trajectory1dGenerator::Trajectory1dGenerator(
    const State& lon_init_state, const State& lat_init_state,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier)
    : init_lon_state_(lon_init_state),
      init_lat_state_(lat_init_state),
      end_condition_sampler_(lon_init_state, lat_init_state,
                             ptr_path_time_graph, ptr_prediction_querier),
      ptr_path_time_graph_(ptr_path_time_graph) {}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const PlanningTarget& planning_target,
    Trajectory1DBundle* ptr_lon_trajectory_bundle,
    Trajectory1DBundle* ptr_lat_trajectory_bundle) {
  // 纵向轨迹生成    
  GenerateLongitudinalTrajectoryBundle(planning_target,
                                       ptr_lon_trajectory_bundle);
  // 横向轨迹生成
  GenerateLateralTrajectoryBundle(ptr_lat_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    const double target_speed,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  ADEBUG << "cruise speed is  " << target_speed;
  /**
   * 对不同时刻的速度进行采样
   * **/
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForCruising(target_speed);

  if (end_conditions.empty()) {
    return;
  }

  // For the cruising case, We use the "QuarticPolynomialCurve1d" class (not the
  // "QuinticPolynomialCurve1d" class) to generate curves. Therefore, we can't
  // invoke the common function to generate trajectory bundles.
  /**
   * 由于采样状态的纵向位移s是变化的、不受约束的，因此少了一个约束条件，、
   * 因此为了饱和约束，采用了4次多项式拟合。
   * **/
  // 根据边界条件和初始状态生成四次多项式，初始条件:{s,ds,dds},结束状态:{ds,dds}
  GenerateTrajectory1DBundle<4>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForStopping(
    const double stop_point,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  ADEBUG << "stop point is " << stop_point;
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForStopping(stop_point);
  if (end_conditions.empty()) {
    return;
  }

  // Use the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForPathTimeObstacles(
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForPathTimePoints();
  if (end_conditions.empty()) {
    return;
  }

  // Use the common function to generate trajectory bundles.
  // 根据边界条件和初始状态生成五式，初始条件:{s,ds,dds},结束状态:{s,ds,dds}
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateLongitudinalTrajectoryBundle(
    const PlanningTarget& planning_target,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  // cruising trajectories are planned regardlessly.
  // 巡航模式下的纵向拟合函数
  GenerateSpeedProfilesForCruising(planning_target.cruise_speed(),
                                   ptr_lon_trajectory_bundle);
  // 根据障碍物进行纵向拟合函数的求解
  GenerateSpeedProfilesForPathTimeObstacles(ptr_lon_trajectory_bundle);

  if (planning_target.has_stop_point()) {
    GenerateSpeedProfilesForStopping(planning_target.stop_point().s(),
                                     ptr_lon_trajectory_bundle);
  }
}

void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    Trajectory1DBundle* ptr_lat_trajectory_bundle) const {

  /**
   * DEFINE_bool(lateral_optimization, true,
   *         "whether using optimization for lateral trajectory generation");
   * **/    
  if (!FLAGS_lateral_optimization) {
    auto end_conditions = end_condition_sampler_.SampleLatEndConditions();

    // Use the common function to generate trajectory bundles.
    GenerateTrajectory1DBundle<5>(init_lat_state_, end_conditions,
                                  ptr_lat_trajectory_bundle);
  } else {
    double s_min = init_lon_state_[0];
    /**
     * DEFINE_double(max_s_lateral_optimization, 60.0,
              "The maximal s for lateral optimization.");
     * **/
    double s_max = s_min + FLAGS_max_s_lateral_optimization;
    /**
     * DEFINE_double(default_delta_s_lateral_optimization, 1.0,
              "The default delta s for lateral optimization.");
     * **/
    double delta_s = FLAGS_default_delta_s_lateral_optimization;

    auto lateral_bounds =
        ptr_path_time_graph_->GetLateralBounds(s_min, s_max, delta_s);

    // LateralTrajectoryOptimizer lateral_optimizer;
    std::unique_ptr<LateralQPOptimizer> lateral_optimizer(
        new LateralOSQPOptimizer);

    lateral_optimizer->optimize(init_lat_state_, delta_s, lateral_bounds);

    auto lateral_trajectory = lateral_optimizer->GetOptimalTrajectory();

    ptr_lat_trajectory_bundle->push_back(
        std::make_shared<PiecewiseJerkTrajectory1d>(lateral_trajectory));
  }
}

}  // namespace planning
}  // namespace apollo
