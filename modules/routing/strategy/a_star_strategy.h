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

#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/routing/strategy/strategy.h"

namespace apollo {
namespace routing {

class AStarStrategy : public Strategy {
 public:
  explicit AStarStrategy(bool enable_change);
  ~AStarStrategy() = default;

  virtual bool Search(const TopoGraph* graph, const SubTopoGraph* sub_graph,
                      const TopoNode* src_node, const TopoNode* dest_node,
                      std::vector<NodeWithRange>* const result_nodes);

 private:
  void Clear();
  double HeuristicCost(const TopoNode* src_node, const TopoNode* dest_node);
  double GetResidualS(const TopoNode* node);
  double GetResidualS(const TopoEdge* edge, const TopoNode* to_node);

 private:
  // 允许变换车道
  bool change_lane_enabled_;
  // OPEN集
  std::unordered_set<const TopoNode*> open_set_;
  // CLOSED集
  std::unordered_set<const TopoNode*> closed_set_;
  // 子父结点键值对
  // key: 子结点
  // value: 父结点
  std::unordered_map<const TopoNode*, const TopoNode*> came_from_;
  // 移动代价键值对
  // key: Node
  // value: 从源结点移动到Node的代价
  std::unordered_map<const TopoNode*, double> g_score_;
  // 结点的进入距离键值对
  // key: Node
  // value: Node的进入距离
  std::unordered_map<const TopoNode*, double> enter_s_;
};

}  // namespace routing
}  // namespace apollo
