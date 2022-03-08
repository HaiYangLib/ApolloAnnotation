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

/*
 * @file
 */

#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf) {
  planner_open_space_config_.CopyFrom(open_space_conf);
  reed_shepp_generator_.reset(
      new ReedShepp(vehicle_param_, planner_open_space_config_));
  next_node_num_ =
      planner_open_space_config_.warm_start_config().next_node_num();
  max_steer_angle_ =
      vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio();
  // step_size : 0.5
  step_size_ = planner_open_space_config_.warm_start_config().step_size();
  // xy_grid_resolution : 0.3
  xy_grid_resolution_ =
      planner_open_space_config_.warm_start_config().xy_grid_resolution();
  // delta_t : 0.5    
  delta_t_ = planner_open_space_config_.delta_t();
  traj_forward_penalty_ =
      planner_open_space_config_.warm_start_config().traj_forward_penalty();
  traj_back_penalty_ =
      planner_open_space_config_.warm_start_config().traj_back_penalty();
  traj_gear_switch_penalty_ =
      planner_open_space_config_.warm_start_config().traj_gear_switch_penalty();
  traj_steer_penalty_ =
      planner_open_space_config_.warm_start_config().traj_steer_penalty();
  traj_steer_change_penalty_ = planner_open_space_config_.warm_start_config()
                                   .traj_steer_change_penalty();
  heu_rs_forward_penalty_ =
      planner_open_space_config_.warm_start_config().heu_rs_forward_penalty();
  heu_rs_back_penalty_ =
      planner_open_space_config_.warm_start_config().heu_rs_back_penalty();
  heu_rs_gear_switch_penalty_ = planner_open_space_config_.warm_start_config()
                                    .heu_rs_gear_switch_penalty();
  heu_rs_steer_penalty_ =
      planner_open_space_config_.warm_start_config().heu_rs_steer_penalty();
  heu_rs_steer_change_penalty_ = planner_open_space_config_.warm_start_config()
                                     .heu_rs_steer_change_penalty();
}

bool HybridAStar::AnalyticExpansion(
    std::shared_ptr<Node3d> current_node,
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      ReedSheppPath_cache_[current_node->GetIndex()];
  // 是否碰撞及超出边界    
  if (!RSPCheck(reeds_shepp_to_check, obstacles_vertices_vec)) {

    return false;
  }

  AINFO << "Reach the end configuration with Reed Sharp";
  // load the whole RSP as nodes and add to the close set
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::ReedSheppHeuristic(
    std::shared_ptr<Node3d> current_node,
    std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_end)) {
    AERROR << "ShortestRSP failed";
    return false;
  }
  // 保存着该节点到终止节点的ReedsShepp
  ReedSheppPath_cache_.insert(
      std::make_pair(current_node->GetIndex(), reeds_shepp_to_end));
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec) {
  for (size_t i = 0; i < reeds_shepp_to_end->x.size(); i++) {
    if (reeds_shepp_to_end->x[i] > XYbounds_[1] ||
        reeds_shepp_to_end->x[i] < XYbounds_[0] ||
        reeds_shepp_to_end->y[i] > XYbounds_[3] ||
        reeds_shepp_to_end->y[i] < XYbounds_[2]) {
      return false;
    }

    std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
        reeds_shepp_to_end->x[i], reeds_shepp_to_end->y[i],
        reeds_shepp_to_end->phi[i], XYbounds_, planner_open_space_config_));

    // 是否碰撞    
    if (!ValidityCheck(node, obstacles_vertices_vec)) {
      return false;
    }
  }

  return true;
}

bool HybridAStar::ValidityCheck(
    std::shared_ptr<Node3d> node,
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec) {
  if (obstacles_vertices_vec.size() == 0) {
    return true;
  }

  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    for (size_t i = 0; i < vertices_num - 1; i++) {
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]);
      if (node->GetBoundingBox(vehicle_param_).HasOverlap(line_segment)) {
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  end_node->SetPre(current_node);
  end_node->SetTrajCost(CalculateRSPCost(reeds_shepp_to_end));
  close_set_.insert(std::make_pair(end_node->GetIndex(), end_node));
  return end_node;
}

/**
 * 扩展节点的重点在于把车辆运动学模型的约束考虑进去，根据限定的steering angle
 * 去搜索相邻的grid。
 * **/
std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;
  size_t index = 0;
  double traveled_distance = 0.0;

  // next_node_num_:20
  // step_size : 0.5
  // steering angle为什么这么算？
  // 首先，根据next_node_index与next_node_num_的对比是可以区分运动方向的
  // 这里的if-else就是区分运动正反方向讨论的（前进和倒车）
  // 其次，车辆在当前的姿态下，既可以向左转、又可以向右转，那么steering angle的
  // 取值范围其实是[-max_steer_angle_, max_steer_angle_]，在正向或反向下，
  // 能取next_node_num_/2个有效值。
  // 即，把[-max_steer_angle_, max_steer_angle_]分为（next_node_num_/2-1）份
  // 所以，steering = 初始偏移量 + 单位间隔 × index
  // steering angle的正负取决于车的转向，而非前进的正反
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -step_size_;
  }

  // take above motion primitive to generate a curve driving the car to a
  // different grid
  // xy_grid_resolution : 0.3
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.emplace_back(last_x);
  intermediate_y.emplace_back(last_y);
  intermediate_phi.emplace_back(last_phi);
  // arc / step_size_:1
  for (size_t i = 0; i < arc / step_size_; i++) {
    double next_x = last_x + traveled_distance * std::cos(last_phi);
    double next_y = last_y + traveled_distance * std::sin(last_phi);
    double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base() * std::tan(steering));
    intermediate_x.emplace_back(next_x);
    intermediate_y.emplace_back(next_y);
    intermediate_phi.emplace_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }

  // check if the vehicle runs outside of XY boundary
  if (intermediate_x.back() > XYbounds_[1] ||
      intermediate_x.back() < XYbounds_[0] ||
      intermediate_y.back() > XYbounds_[3] ||
      intermediate_y.back() < XYbounds_[2]) {
    return nullptr;
  }

  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_));

  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0);
  next_node->SetSteer(steering);

  return next_node;
}

void HybridAStar::CalculateNodeCost(
    std::shared_ptr<Node3d> current_node, std::shared_ptr<Node3d> next_node,
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  // evaluate heuristic cost
  next_node->SetHeuCost(NonHoloNoObstacleHeuristic(reeds_shepp_to_end));
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::NonHoloNoObstacleHeuristic(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  return CalculateRSPCost(reeds_shepp_to_end);
}

double HybridAStar::CalculateRSPCost(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  double RSP_cost = 0.0;
  for (size_t i = 0; i < reeds_shepp_to_end->segs_lengths.size(); i++) {
    if (reeds_shepp_to_end->segs_lengths[i] > 0.0) {
      RSP_cost += reeds_shepp_to_end->segs_lengths[i] * heu_rs_forward_penalty_;
    } else {
      RSP_cost += -reeds_shepp_to_end->segs_lengths[i] * heu_rs_back_penalty_;
    }
  }

  for (size_t i = 0; i < reeds_shepp_to_end->segs_lengths.size() - 1; i++) {
    if (reeds_shepp_to_end->segs_lengths[i] *
            reeds_shepp_to_end->segs_lengths[i + 1] <
        0.0) {
      RSP_cost += heu_rs_gear_switch_penalty_;
    }
  }
  // steering cost
  bool first_nonS_flag = false;
  char last_turning;
  for (size_t i = 0; i < reeds_shepp_to_end->segs_types.size(); i++) {
    if (reeds_shepp_to_end->segs_types[i] != 'S') {
      RSP_cost += heu_rs_steer_penalty_ * max_steer_angle_;
      if (!first_nonS_flag) {
        last_turning = reeds_shepp_to_end->segs_types[i];
        first_nonS_flag = true;
        continue;
      }
      if (reeds_shepp_to_end->segs_types[i] != last_turning) {
        RSP_cost += 2 * heu_rs_steer_change_penalty_ * max_steer_angle_;
      }
    }
  }
  return RSP_cost;
}
bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.size() == 0 || y.size() == 0 || phi.size() == 0) {
      AERROR << "result size check failed";
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }

  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;
  if (!GenerateSpeedAcceleration(result)) {
    AERROR << "GenerateSpeedAcceleration fail";
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "state sizes not equal";
    return false;
  }

  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    AERROR << "control sizes not equal or not right";
    AERROR << result->a.size();
    AERROR << result->steer.size();
    AERROR << result->x.size();
    return false;
  }
  return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR << "result size check when generating speed and acceleration fail";
    return false;
  }
  size_t x_size = result->x.size();
  // load velocity from position
  for (size_t i = 0; i < x_size - 1; i++) {
    double discrete_v = ((result->x[i + 1] - result->x[i]) / delta_t_) *
                            std::cos(result->phi[i]) +
                        ((result->y[i + 1] - result->y[i]) / delta_t_) *
                            std::sin(result->phi[i]);
    result->v.emplace_back(discrete_v);
  }
  result->v.emplace_back(0.0);
  // load acceleration from velocity
  for (size_t i = 0; i < x_size - 1; i++) {
    double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.emplace_back(discrete_a);
  }
  // load steering from phi
  for (size_t i = 0; i < x_size - 1; i++) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            vehicle_param_.wheel_base() / step_size_;
    if (result->v[i] > 0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.emplace_back(discrete_steer);
  }
  
  return true;
}

bool HybridAStar::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    HybridAStartResult* result) {
  // clear containers
  open_set_.clear();
  close_set_.clear();
  while (!open_pq_.empty()) open_pq_.pop();
  final_node_ = nullptr;

  // load XYbounds
  XYbounds_ = XYbounds;
  // load nodes and obstacles
  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));

  if (!ValidityCheck(start_node_, obstacles_vertices_vec)) {
    AERROR << "start_node in collision with obstacles";
    return false;
  }

  if (!ValidityCheck(end_node_, obstacles_vertices_vec)) {
    AERROR << "end_node in collision with obstacles";
    return false;
  }

  // load open set, priority queue and ReedSheepPath_cache
  open_set_.insert(std::make_pair(start_node_->GetIndex(), start_node_));
  open_pq_.push(
      std::make_pair(start_node_->GetIndex(), start_node_->GetCost()));
  std::shared_ptr<ReedSheppPath> reeds_shepp_first_node =
      std::shared_ptr<ReedSheppPath>(new ReedSheppPath());
  if (!reed_shepp_generator_->ShortestRSP(start_node_, end_node_,
                                          reeds_shepp_first_node)) {
    AERROR << "ShortestRSP failed";
    return false;
  }

  ReedSheppPath_cache_.insert(
      std::make_pair(start_node_->GetIndex(), reeds_shepp_first_node));

  // Hybrid A* begins
  size_t explored_node_num = 0;
  double reeds_shepp_time = 0.0;
  double start_timestamp = 0.0;
  double end_timestamp = 0.0;
  while (!open_pq_.empty()) {
    // take out the lowest cost neighoring node
    size_t current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // check if a analystic curve could be connected from current configuration
    // to the end configuration without collision. if so, search ends.
    start_timestamp = Clock::NowInSeconds();
    if (AnalyticExpansion(current_node, obstacles_vertices_vec)) {
      break;
    }
    
    close_set_.insert(std::make_pair(current_node->GetIndex(), current_node));
    end_timestamp = Clock::NowInSeconds();
    reeds_shepp_time += (end_timestamp - start_timestamp);
    // next_node_num_:20
    for (size_t i = 0; i < next_node_num_; i++) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // collision check
      // 进行碰撞检查
      if (!ValidityCheck(next_node, obstacles_vertices_vec)) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }

      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        start_timestamp = Clock::NowInSeconds();
        std::shared_ptr<ReedSheppPath> reeds_shepp_heuristic =
            std::shared_ptr<ReedSheppPath>(new ReedSheppPath());

        if (!ReedSheppHeuristic(next_node, reeds_shepp_heuristic)) {
          AERROR << "Heuristic fail";
          continue;
        }

        CalculateNodeCost(current_node, next_node, reeds_shepp_heuristic);

        end_timestamp = Clock::NowInSeconds();
        reeds_shepp_time += (end_timestamp - start_timestamp);

        open_set_.insert(std::make_pair(next_node->GetIndex(), next_node));
        open_pq_.push(
            std::make_pair(next_node->GetIndex(), next_node->GetCost()));
      }
    }
  }

  if (final_node_ == nullptr) {
    AERROR << "Hybrid A searching return null ptr(open_set ran out)";
    return false;
  }

  if (!GetResult(result)) {
    AERROR << "GetResult failed";
    return false;
  }

  AINFO << "explored node num is " << explored_node_num;
  AINFO << "reeds_shepp_time is " << reeds_shepp_time;
  return true;
}
}  // namespace planning
}  // namespace apollo
