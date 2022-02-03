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

#include "modules/routing/strategy/a_star_strategy.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/graph/topo_graph.h"

namespace apollo {
namespace routing {
namespace {

struct SearchNode {
  const TopoNode* topo_node = nullptr;
  double f = std::numeric_limits<double>::max();

  SearchNode() = default;
  explicit SearchNode(const TopoNode* node)
      : topo_node(node), f(std::numeric_limits<double>::max()) {}
  SearchNode(const SearchNode& search_node) = default;

  bool operator<(const SearchNode& node) const {
    // in order to let the top of priority queue is the smallest one!
    return f > node.f;
  }

  bool operator==(const SearchNode& node) const {
    return topo_node == node.topo_node;
  }
};

double GetCostToNeighbor(const TopoEdge* edge) {
  return (edge->Cost() + edge->ToNode()->Cost());
}

const TopoNode* GetLargestNode(const std::vector<const TopoNode*>& nodes) {
  double max_range = 0.0;
  const TopoNode* largest = nullptr;
  for (const auto* node : nodes) {
    const double temp_range = node->EndS() - node->StartS();
    if (temp_range > max_range) {
      max_range = temp_range;
      largest = node;
    }
  }
  return largest;
}

bool AdjustLaneChangeBackward(
    std::vector<const TopoNode*>* const result_node_vec) {
  for (int i = static_cast<int>(result_node_vec->size()) - 2; i > 0; --i) {
    const auto* from_node = result_node_vec->at(i);
    const auto* to_node = result_node_vec->at(i + 1);
    const auto* base_node = result_node_vec->at(i - 1);
    const auto* from_to_edge = from_node->GetOutEdgeTo(to_node);
    if (from_to_edge == nullptr) {
      // may need to recalculate edge,
      // because only edge from origin node to subnode is saved
      from_to_edge = to_node->GetInEdgeFrom(from_node);
    }
    if (from_to_edge == nullptr) {
      AERROR << "Get null ptr to edge:" << from_node->LaneId() << " ("
             << from_node->StartS() << ", " << from_node->EndS() << ")"
             << " --> " << to_node->LaneId() << " (" << to_node->StartS()
             << ", " << to_node->EndS() << ")";
      return false;
    }
    if (from_to_edge->Type() != TopoEdgeType::TET_FORWARD) {
      if (base_node->EndS() - base_node->StartS() <
          from_node->EndS() - from_node->StartS()) {
        continue;
      }
      std::vector<const TopoNode*> candidate_set;
      candidate_set.push_back(from_node);
      const auto& out_edges = base_node->OutToLeftOrRightEdge();
      for (const auto* edge : out_edges) {
        const auto* candidate_node = edge->ToNode();
        if (candidate_node == from_node) {
          continue;
        }
        if (candidate_node->GetOutEdgeTo(to_node) != nullptr) {
          candidate_set.push_back(candidate_node);
        }
      }
      const auto* largest_node = GetLargestNode(candidate_set);
      if (largest_node == nullptr) {
        return false;
      }
      if (largest_node != from_node) {
        result_node_vec->at(i) = largest_node;
      }
    }
  }
  return true;
}

bool AdjustLaneChangeForward(
    std::vector<const TopoNode*>* const result_node_vec) {
  for (size_t i = 1; i < result_node_vec->size() - 1; ++i) {
    const auto* from_node = result_node_vec->at(i - 1);
    const auto* to_node = result_node_vec->at(i);
    const auto* base_node = result_node_vec->at(i + 1);
    const auto* from_to_edge = from_node->GetOutEdgeTo(to_node);
    if (from_to_edge == nullptr) {
      // may need to recalculate edge,
      // because only edge from origin node to subnode is saved
      from_to_edge = to_node->GetInEdgeFrom(from_node);
    }
    if (from_to_edge == nullptr) {
      AERROR << "Get null ptr to edge:" << from_node->LaneId() << " ("
             << from_node->StartS() << ", " << from_node->EndS() << ")"
             << " --> " << to_node->LaneId() << " (" << to_node->StartS()
             << ", " << to_node->EndS() << ")";
      return false;
    }
    if (from_to_edge->Type() != TopoEdgeType::TET_FORWARD) {
      if (base_node->EndS() - base_node->StartS() <
          to_node->EndS() - to_node->StartS()) {
        continue;
      }
      std::vector<const TopoNode*> candidate_set;
      candidate_set.push_back(to_node);
      const auto& in_edges = base_node->InFromLeftOrRightEdge();
      for (const auto* edge : in_edges) {
        const auto* candidate_node = edge->FromNode();
        if (candidate_node == to_node) {
          continue;
        }
        if (candidate_node->GetInEdgeFrom(from_node) != nullptr) {
          candidate_set.push_back(candidate_node);
        }
      }
      const auto* largest_node = GetLargestNode(candidate_set);
      if (largest_node == nullptr) {
        return false;
      }
      if (largest_node != to_node) {
        result_node_vec->at(i) = largest_node;
      }
    }
  }
  return true;
}

bool AdjustLaneChange(std::vector<const TopoNode*>* const result_node_vec) {
  if (result_node_vec->size() < 3) {
    return true;
  }
  if (!AdjustLaneChangeBackward(result_node_vec)) {
    AERROR << "Failed to adjust lane change backward";
    return false;
  }
  if (!AdjustLaneChangeForward(result_node_vec)) {
    AERROR << "Failed to adjust lane change backward";
    return false;
  }
  return true;
}

bool Reconstruct(
    const std::unordered_map<const TopoNode*, const TopoNode*>& came_from,
    const TopoNode* dest_node, std::vector<NodeWithRange>* result_nodes) {
  std::vector<const TopoNode*> result_node_vec;
  result_node_vec.push_back(dest_node);

  auto iter = came_from.find(dest_node);
  while (iter != came_from.end()) {
    result_node_vec.push_back(iter->second);
    iter = came_from.find(iter->second);
  }
  std::reverse(result_node_vec.begin(), result_node_vec.end());
  if (!AdjustLaneChange(&result_node_vec)) {
    AERROR << "Failed to adjust lane change";
    return false;
  }
  result_nodes->clear();
  for (const auto* node : result_node_vec) {
    result_nodes->emplace_back(node->OriginNode(), node->StartS(),
                               node->EndS());
  }
  return true;
}

}  // namespace

AStarStrategy::AStarStrategy(bool enable_change)
    : change_lane_enabled_(enable_change) {}

void AStarStrategy::Clear() {
  closed_set_.clear();
  open_set_.clear();
  came_from_.clear();
  enter_s_.clear();
  g_score_.clear();
}

double AStarStrategy::HeuristicCost(const TopoNode* src_node,
                                    const TopoNode* dest_node) {
  const auto& src_point = src_node->AnchorPoint();
  const auto& dest_point = dest_node->AnchorPoint();
  double distance = std::fabs(src_point.x() - dest_point.x()) +
                    std::fabs(src_point.y() - dest_point.y());
  return distance;
}

/**
 * routing模块的路径搜索功是通过AStart算法完成的，
 * 输入修正后的起点、终点、读取的拓扑地图以及根据起点终点生成的子拓扑图，得到起点到达终点的点集
 * **/
bool AStarStrategy::Search(const TopoGraph* graph,
                           const SubTopoGraph* sub_graph,
                           const TopoNode* src_node, const TopoNode* dest_node,
                           std::vector<NodeWithRange>* const result_nodes) {
  Clear();
  AINFO << "Start A* search algorithm.";

  /**
   * 优先级队列openlist,用priority_queue实现.std::priority_queue是一种容器适配器，
   * 它提供常数时间的最大元素查找功能，亦即其栈顶元素top永远输出队列中的最大元素。
   * 但SearchNode内部重载了<运算符，对小于操作作了相反的定义
   * 因此std::priority_queue<SearchNode>的栈顶元素永远输出队列中的最小元素。
   * **/
  std::priority_queue<SearchNode> open_set_detail;
  // 将源结点设置为检查结点
  SearchNode src_search_node(src_node);
  // 计算检查结点的启发式代价值f
  src_search_node.f = HeuristicCost(src_node, dest_node);
  // 将检查结点压入OPEN集优先级队列
  open_set_detail.push(src_search_node);
  // 将源结点加入OPEN集
  open_set_.insert(src_node);
  // 源结点到自身的移动代价值g为0
  g_score_[src_node] = 0.0;
  // 设置源结点的进入s值
  enter_s_[src_node] = src_node->StartS();

  SearchNode current_node;
  std::unordered_set<const TopoEdge*> next_edge_set;
  std::unordered_set<const TopoEdge*> sub_edge_set;
  // 只要OPEN集优先级队列不为空，就不断循环检查
  while (!open_set_detail.empty()) {
    // 取出栈顶元素（f值最小）
    current_node = open_set_detail.top();
    // 设置起始结点
    const auto* from_node = current_node.topo_node;
    // 若起始结点已抵达最终的目标结点，则反向回溯输出完整的路由，返回。
    if (current_node.topo_node == dest_node) {
      if (!Reconstruct(came_from_, from_node, result_nodes)) {
        AERROR << "Failed to reconstruct route.";
        return false;
      }
      return true;
    }

    // 从OPEN集中删除起始结点
    open_set_.erase(from_node);
    // 从OPEN集队列中删除起始结点
    open_set_detail.pop();

    // 若起始结点from_node在CLOSED集中的计数不为0，表明之前已被检查过，直接跳过
    if (closed_set_.count(from_node) != 0) {
      // if showed before, just skip...
      continue;
    }

    // 将起始结点加入关闭集
    closed_set_.emplace(from_node);

    // if residual_s is less than FLAGS_min_length_for_lane_change, only move
    // forward

    /**
     * DEFINE_double(min_length_for_lane_change, 1.0,
     *         "meters, which is 100 feet.  Minimum distance needs to travel on
     * " "a lane before making a lane change. Recommended by "
     *         "https://www.oregonlaws.org/ors/811.375");
     *
     * **/

    // 获取起始结点from_node的所有相邻边
    // 若起始结点from_node到终点的剩余距离s比FLAGS_min_length_for_lane_change要短，
    // 则不考虑变换车道，即只考虑前方结点而不考虑左右结点。
    // 反之，若s比FLAGS_min_length_for_lane_change要长，则考虑前方及左右结点。

    //先通过判断用不用变道,再获取当前节点的所有相邻边
    // GetResidualS 为当前节点到终点的剩余距离s
    //若s<min_length则不变道,若s > min_length 则变道
    const auto& neighbor_edges =
        (GetResidualS(from_node) > FLAGS_min_length_for_lane_change &&
         change_lane_enabled_)
            ? from_node->OutToAllEdge()
            : from_node->OutToSucEdge();

    // 当前测试的移动代价值
    double tentative_g_score = 0.0;
    next_edge_set.clear();
    // 从相邻边neighbor_edges中获取其内部包含的边，将所有相邻边全部加入集合：next_edge_set
    for (const auto* edge : neighbor_edges) {
      sub_edge_set.clear();
      sub_graph->GetSubInEdgesIntoSubGraph(edge, &sub_edge_set);
      next_edge_set.insert(sub_edge_set.begin(), sub_edge_set.end());
    }

    // 所有相邻边的目标结点就是我们需要逐一测试的相邻结点，对相结点点逐一测试，寻找
    // 总代价f = g + h最小的结点，该结点就是起始结点所需的相邻目标结点。
    for (const auto* edge : next_edge_set) {
      const auto* to_node = edge->ToNode();
      if (closed_set_.count(to_node) == 1) {
        continue;
      }

      // 若当前边到相邻结点to_node的距离小于FLAGS_min_length_for_lane_change，表明不能
      // 通过变换车道的方式从当前边切换到相邻结点to_node，直接忽略。
      if (GetResidualS(edge, to_node) < FLAGS_min_length_for_lane_change) {
        continue;
      }

      tentative_g_score =
          g_score_[current_node.topo_node] + GetCostToNeighbor(edge);

      // 如果边类型不是前向，而是左向或右向，表示变换车道的情形，则更改移动代价值g
      // 的计算方式
      if (edge->Type() != TopoEdgeType::TET_FORWARD) {
        tentative_g_score -=
            (edge->FromNode()->Cost() + edge->ToNode()->Cost()) / 2;
      }

      // 总代价 f = g + h
      double f = tentative_g_score + HeuristicCost(to_node, dest_node);

      // 若相邻结点to_node在OPEN集且当前总代价f大于源结点到相邻结点to_node的移动代价g，表明现有情形下
      // 从当前结点到相邻结点to_node的路径不是最优，直接忽略。
      // 因为相邻结点to_node在OPEN集中，后续还会对该结点进行考察。
      // open_set_.count(to_node) != 0修改为open_set_.count(to_node) > 0似乎更好
      if (open_set_.count(to_node) != 0 && f >= g_score_[to_node]) {
        continue;
      }

      // if to_node is reached by forward, reset enter_s to start_s
      // 如果是以向前（而非向左或向右）的方式抵达相邻结点to_node，则将to_node的进入距离更新为
      // to_node的起始距离。
      if (edge->Type() == TopoEdgeType::TET_FORWARD) {
        enter_s_[to_node] = to_node->StartS();
      } else {
        // else, add enter_s with FLAGS_min_length_for_lane_change
        // 若是以向左或向右方式抵达相邻结点to_node，则将to_node的进入距离更新为
        // 当前结点from_node的进入距离加上最小换道长度，并乘以相邻结点to_node长度
        // 与当前结点from_node长度的比值（这么做的目的是为了归一化，以便最终的代价量纲一致）。
        double to_node_enter_s =
            (enter_s_[from_node] + FLAGS_min_length_for_lane_change) /
            from_node->Length() * to_node->Length();
        // enter s could be larger than end_s but should be less than length
        to_node_enter_s = std::min(to_node_enter_s, to_node->Length());
        // if enter_s is larger than end_s and to_node is dest_node
        if (to_node_enter_s > to_node->EndS() && to_node == dest_node) {
          continue;
        }
        enter_s_[to_node] = to_node_enter_s;
      }
      // 更新从源点移动到结点to_node的移动代价（因为找到了一条代价更小的路径，必须更新它）
      g_score_[to_node] = f;
      // 将相邻结点to_node设置为下一个待考察结点
      SearchNode next_node(to_node);
      next_node.f = f;
      // 当下一个待考察结点next_node加入到OPEN优先级队列
      open_set_detail.push(next_node);
      // 将to_node的父结点设置为from_node
      came_from_[to_node] = from_node;
      // 若相邻结点不在OPEN集中，则将其加入OPEN集，以便后续考察
      if (open_set_.count(to_node) == 0) {
        open_set_.insert(to_node);
      }
    }
  }
  AERROR << "Failed to find goal lane with id: " << dest_node->LaneId();
  return false;
}

double AStarStrategy::GetResidualS(const TopoNode* node) {
  double start_s = node->StartS();
  const auto iter = enter_s_.find(node);
  if (iter != enter_s_.end()) {
    if (iter->second > node->EndS()) {
      return 0.0;
    }
    start_s = iter->second;
  } else {
    AWARN << "lane " << node->LaneId() << "(" << node->StartS() << ", "
          << node->EndS() << "not found in enter_s map";
  }
  double end_s = node->EndS();
  const TopoNode* succ_node = nullptr;
  for (const auto* edge : node->OutToAllEdge()) {
    if (edge->ToNode()->LaneId() == node->LaneId()) {
      succ_node = edge->ToNode();
      break;
    }
  }
  if (succ_node != nullptr) {
    end_s = succ_node->EndS();
  }
  return (end_s - start_s);
}

double AStarStrategy::GetResidualS(const TopoEdge* edge,
                                   const TopoNode* to_node) {
  if (edge->Type() == TopoEdgeType::TET_FORWARD) {
    return std::numeric_limits<double>::max();
  }
  double start_s = to_node->StartS();
  const auto* from_node = edge->FromNode();
  const auto iter = enter_s_.find(from_node);
  if (iter != enter_s_.end()) {
    double temp_s = iter->second / from_node->Length() * to_node->Length();
    start_s = std::max(start_s, temp_s);
  } else {
    AWARN << "lane " << from_node->LaneId() << "(" << from_node->StartS()
          << ", " << from_node->EndS() << "not found in enter_s map";
  }
  double end_s = to_node->EndS();
  const TopoNode* succ_node = nullptr;
  for (const auto* edge : to_node->OutToAllEdge()) {
    if (edge->ToNode()->LaneId() == to_node->LaneId()) {
      succ_node = edge->ToNode();
      break;
    }
  }
  if (succ_node != nullptr) {
    end_s = succ_node->EndS();
  }
  return (end_s - start_s);
}

}  // namespace routing
}  // namespace apollo
