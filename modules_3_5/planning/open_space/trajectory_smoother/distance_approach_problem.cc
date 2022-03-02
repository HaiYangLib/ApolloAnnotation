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

/*
 * @file
 */

#include "modules/planning/open_space/trajectory_smoother/distance_approach_problem.h"

namespace apollo {
namespace planning {

DistanceApproachProblem::DistanceApproachProblem(
    const PlannerOpenSpaceConfig& planner_open_space_config) {
  planner_open_space_config_.CopyFrom(planner_open_space_config);
}

bool DistanceApproachProblem::Solve(
    const Eigen::MatrixXd& x0, const Eigen::MatrixXd& xF,
    const Eigen::MatrixXd& last_time_u, const size_t& horizon,
    const double& ts, const Eigen::MatrixXd& ego, const Eigen::MatrixXd& xWS,
    const Eigen::MatrixXd& uWS, const Eigen::MatrixXd& l_warm_up,
    const Eigen::MatrixXd& n_warm_up, const std::vector<double>& XYbounds,
    const size_t& obstacles_num,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
    Eigen::MatrixXd* time_result, Eigen::MatrixXd* dual_l_result,
    Eigen::MatrixXd* dual_n_result) {
  // TODO(QiL) : evaluate whether need to new it everytime
  auto t_start = cyber::Time::Now().ToSecond();
  DistanceApproachIPOPTInterface* ptop = new DistanceApproachIPOPTInterface(
      horizon, ts, ego, xWS, uWS, l_warm_up, n_warm_up, x0, xF, last_time_u,
      XYbounds, obstacles_edges_num, obstacles_num, obstacles_A, obstacles_b,
      planner_open_space_config_);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_print_level());
  app->Options()->SetIntegerValue("mumps_mem_percent",
      planner_open_space_config_.distance_approach_config().
        ipopt_config().mumps_mem_percent());
  app->Options()->SetNumericValue("mumps_pivtol",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().mumps_pivtol());
  app->Options()->SetIntegerValue("max_iter",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_max_iter());
  app->Options()->SetNumericValue("tol",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_tol());
  app->Options()->SetNumericValue("acceptable_constr_viol_tol",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_acceptable_constr_viol_tol());
  app->Options()->SetNumericValue("min_hessian_perturbation",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_min_hessian_perturbation());
  app->Options()->SetNumericValue("jacobian_regularization_value",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_jacobian_regularization_value());
  app->Options()->SetStringValue("print_timing_statistics",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_print_timing_statistics());
  app->Options()->SetStringValue("alpha_for_y",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_alpha_for_y());
  app->Options()->SetStringValue("recalc_y",
      planner_open_space_config_.distance_approach_config().\
        ipopt_config().ipopt_recalc_y());

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AERROR << "*** Distiance Approach problem error during initialization!";
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    AINFO << "*** The problem solved in " << iter_count << " iterations!";

    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    AINFO << "*** The final value of the objective function is " << final_obj
          << '.';

    auto t_end = cyber::Time::Now().ToSecond();

    AINFO << "DistanceApproachProblem solving time in second : "
          << t_end - t_start;
  } else {
    AINFO << "Solve not succeeding, return status: " << int(status);
  }

  ptop->get_optimization_results(state_result, control_result, time_result,
                                 dual_l_result, dual_n_result);

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

}  // namespace planning
}  // namespace apollo
