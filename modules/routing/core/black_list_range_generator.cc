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


#include "modules/routing/core/black_list_range_generator.h"

namespace apollo {
namespace routing {

constexpr double S_GAP_FOR_BLACK = 0.01;

namespace {

double MoveSForward(double s, double upper_bound) {
  if (s > upper_bound) {
    AERROR << "Illegal s: " << s << ", upper bound: " << upper_bound;
    return s;
  }

  // S_GAP_FOR_BLACK = 0.01;
  if (s + S_GAP_FOR_BLACK < upper_bound) {
    return (s + S_GAP_FOR_BLACK);
  } else {
    return ((s + upper_bound) / 2.0);
  }
}

double MoveSBackward(double s, double lower_bound) {
  if (s < lower_bound) {
    AERROR << "Illegal s: " << s << ", lower bound: " << lower_bound;
    return s;
  }

  // S_GAP_FOR_BLACK = 0.01;
  if (s - S_GAP_FOR_BLACK > lower_bound) {
    return (s - S_GAP_FOR_BLACK);
  } else {
    return ((s + lower_bound) / 2.0);
  }
}

void GetOutParallelLane(const TopoNode* node,
                        std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->OutToLeftOrRightEdge()) {
    const auto* to_node = edge->ToNode();
    if (node_set->count(to_node) == 0) {
      node_set->emplace(to_node);
      GetOutParallelLane(to_node, node_set);
    }
  }
}

void GetInParallelLane(const TopoNode* node,
                       std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->InFromLeftOrRightEdge()) {
    const auto* from_node = edge->FromNode();
    if (node_set->count(from_node) == 0) {
      node_set->emplace(from_node);
      GetInParallelLane(from_node, node_set);
    }
  }
}

// for new navigator
void AddBlackMapFromRoad(const RoutingRequest& request, const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  for (const auto& road_id : request.blacklisted_road()) {
    std::unordered_set<const TopoNode*> road_nodes_set;
    graph->GetNodesByRoadId(road_id, &road_nodes_set);
    for (const auto& node : road_nodes_set) {
      range_manager->Add(node, 0.0, node->Length());
    }
  }
}

// for new navigator
void AddBlackMapFromLane(const RoutingRequest& request, const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  for (const auto& lane : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(lane.id());
    if (node) {
      range_manager->Add(node, lane.start_s(), lane.end_s());
    }
  }
}

// cut_ratio=start_cut_s / start_length
void AddBlackMapFromOutParallel(const TopoNode* node, double cut_ratio,
                                TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetOutParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

// cut_ratio=end_cut_s / end_length
void AddBlackMapFromInParallel(const TopoNode* node, double cut_ratio,
                               TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetInParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

}  // namespace

// 通过request请求传入黑名单lane和road，每次直接屏蔽一整条road或者lane
void BlackListRangeGenerator::GenerateBlackMapFromRequest(
    const RoutingRequest& request, const TopoGraph* graph,
    TopoRangeManager* const range_manager) const {
  AddBlackMapFromLane(request, graph, range_manager);
  AddBlackMapFromRoad(request, graph, range_manager);
  range_manager->SortAndMerge();
}

/**
 * 虽然range_manager支持传入range，但是这种场景只是针对routing_request传入的点对lane做切割，
 * 方便计算，每次切割的区间的起点和终点重合，是一个特殊场景，后续应该有用到比如在一条lane里，
 * 有某一段不能行驶的功能
 * **/
void BlackListRangeGenerator::AddBlackMapFromTerminal(
    const TopoNode* src_node, const TopoNode* dest_node, double start_s,
    double end_s, TopoRangeManager* const range_manager) const {
  double start_length = src_node->Length();
  double end_length = dest_node->Length();

  static constexpr double kEpsilon = 1e-2;
  const double start_s_adjusted =
      (start_s > start_length && start_s - start_length <= kEpsilon)
          ? start_length
          : start_s;
  const double end_s_adjusted =
      (end_s > end_length && end_s - end_length <= kEpsilon) ? end_length
                                                             : end_s;

  if (start_s_adjusted < 0.0 || start_s_adjusted > start_length) {
    AERROR << "Illegal start_s: " << start_s << ", length: " << start_length;
    return;
  }

  if (end_s_adjusted < 0.0 || end_s_adjusted > end_length) {
    AERROR << "Illegal end_s: " << end_s << ", length: " << end_length;
    return;
  }

  double start_cut_s = MoveSBackward(start_s_adjusted, 0.0);

  // 注意这里range的起点和终点是同一个点，为routing的起点
  range_manager->Add(src_node, start_cut_s, start_cut_s);
  // 把平行的节点也按照比例做相同的切分
  AddBlackMapFromOutParallel(src_node, start_cut_s / start_length,
                             range_manager);

  double end_cut_s = MoveSForward(end_s_adjusted, end_length);
  // 注意这里range的起点和终点是同一个点，为routing的终点
  range_manager->Add(dest_node, end_cut_s, end_cut_s);

  AddBlackMapFromInParallel(dest_node, end_cut_s / end_length, range_manager);
  // 排序并且合并
  range_manager->SortAndMerge();
}

}  // namespace routing
}  // namespace apollo
