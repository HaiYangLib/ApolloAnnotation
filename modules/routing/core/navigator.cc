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

#include "modules/routing/core/navigator.h"

#include "cyber/common/file.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {

namespace {

using apollo::common::ErrorCode;

bool ShowRequestInfo(const RoutingRequest& request, const TopoGraph* graph) {
  for (const auto& wp : request.waypoint()) {
    const auto* node = graph->GetNode(wp.id());
    if (node == nullptr) {
      AERROR << "Way node is not found in topo graph! ID: " << wp.id();
      return false;
    }
    AINFO << "Way point:\tlane id: " << wp.id() << " s: " << wp.s()
          << " x: " << wp.pose().x() << " y: " << wp.pose().y()
          << " length: " << node->Length();
  }

  for (const auto& bl : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(bl.id());
    if (node == nullptr) {
      AERROR << "Black list node is not found in topo graph! ID: " << bl.id();
      return false;
    }
    AINFO << "Black point:\tlane id: " << bl.id()
          << " start_s: " << bl.start_s() << " end_s: " << bl.end_s()
          << " length: " << node->Length();
  }

  return true;
}

bool GetWayNodes(const RoutingRequest& request, const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 std::vector<double>* const way_s) {
  for (const auto& point : request.waypoint()) {
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
      AERROR << "Cannot find way point in graph! Id: " << point.id();
      return false;
    }
    way_nodes->push_back(cur_node);
    way_s->push_back(point.s());
  }
  return true;
}

void SetErrorCode(const common::ErrorCode& error_code_id,
                  const std::string& error_string,
                  common::StatusPb* const error_code) {
  error_code->set_error_code(error_code_id);
  error_code->set_msg(error_string);
  if (error_code_id == common::ErrorCode::OK) {
    ADEBUG << error_string.c_str();
  } else {
    AERROR << error_string.c_str();
  }
}

void PrintDebugData(const std::vector<NodeWithRange>& nodes) {
  AINFO << "Route lane id\tis virtual\tstart s\tend s";
  for (const auto& node : nodes) {
    AINFO << node.GetTopoNode()->LaneId() << "\t"
          << node.GetTopoNode()->IsVirtual() << "\t" << node.StartS() << "\t"
          << node.EndS();
  }
}

}  // namespace

Navigator::Navigator(const std::string& topo_file_path) {
  /**
   * Graph在下面文件中定义：
   * modules/routing/proto/topo_graph.proto
   *
   * message Graph {
   * optional string hdmap_version = 1;
   * optional string hdmap_district = 2;
   * repeated Node node = 3;
   * repeated Edge edge = 4;
   * }
   *
   *
   * topo_file_path的一个候选是：
   * modules/map/data/demo/routing_map.txt
   *
   *
   * routing_map由GraphCreator生成
   * **/

  Graph graph;
  if (!cyber::common::GetProtoFromFile(topo_file_path, &graph)) {
    AERROR << "Failed to read topology graph from " << topo_file_path;
    return;
  }

  /**
   * 成员变量：
   * 拓扑地图
   * std::unique_ptr<TopoGraph> graph_;
   *
   * TopoGraph在下面文件中定义：
   * modules/routing/graph/topo_graph.cc
   * **/
  graph_.reset(new TopoGraph());
  if (!graph_->LoadGraph(graph)) {
    AINFO << "Failed to init navigator graph failed! File path: "
          << topo_file_path;
    return;
  }

  black_list_generator_.reset(new BlackListRangeGenerator);
  result_generator_.reset(new ResultGenerator);
  is_ready_ = true;

  AINFO << "The navigator is ready.";
}

Navigator::~Navigator() {}

bool Navigator::IsReady() const { return is_ready_; }

void Navigator::Clear() { topo_range_manager_.Clear(); }

bool Navigator::Init(const RoutingRequest& request, const TopoGraph* graph,
                     std::vector<const TopoNode*>* const way_nodes,
                     std::vector<double>* const way_s) {
  Clear();
  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {
    AERROR << "Failed to find search terminal point in graph!";
    return false;
  }
  black_list_generator_->GenerateBlackMapFromRequest(request, graph_.get(),
                                                     &topo_range_manager_);
  return true;
}

bool Navigator::MergeRoute(
    const std::vector<NodeWithRange>& node_vec,
    std::vector<NodeWithRange>* const result_node_vec) const {
  for (const auto& node : node_vec) {
    if (result_node_vec->empty() ||
        result_node_vec->back().GetTopoNode() != node.GetTopoNode()) {
      result_node_vec->push_back(node);
    } else {
      if (result_node_vec->back().EndS() < node.StartS()) {
        AERROR << "Result route is not continuous.";
        return false;
      }
      result_node_vec->back().SetEndS(node.EndS());
    }
  }
  return true;
}


/**
 * 在routing请求中可以指定黑名单路和车道，这样routing请求将不会计算这些车道。
 * 应用场景是需要避开拥堵路段，这需要能够根据情况实时请求，
 * 在routing_request中可以设置黑名单也刚好可以满足上面的需求
 * **/
bool Navigator::SearchRouteByStrategy(
    const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,
    const std::vector<double>& way_s,
    std::vector<NodeWithRange>* const result_nodes) const {
  std::unique_ptr<Strategy> strategy_ptr;

  /**
   * DEFINE_bool(enable_change_lane_in_result, true,
   *     "contain change lane operator in result");
   * **/
  // 使用A*算法
  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));

  result_nodes->clear();
  std::vector<NodeWithRange> node_vec;
  // 遍历routing_request节点
  for (size_t i = 1; i < way_nodes.size(); ++i) {
    const auto* way_start = way_nodes[i - 1];
    const auto* way_end = way_nodes[i];
    double way_start_s = way_s[i - 1];
    double way_end_s = way_s[i];

    TopoRangeManager full_range_manager = topo_range_manager_;
    
    /**
     * 添加黑名单，这里主要是把车道根据起点和终点做分割。
     * 
     * "AddBlackMapFromTerminal"中会把节点(这里的节点就是lane)切分，
     * 切分之后的数据保存在"TopoRangeManager"中，
     * 而"SubTopoGraph"会根据"TopoRangeManager"中的数据初始化子图
     * 节点就是一条lane，而子节点是对lane做了切割，把一条lane根据黑名单区域，生成几个子节点。
     * **/
    black_list_generator_->AddBlackMapFromTerminal(
        way_start, way_end, way_start_s, way_end_s, &full_range_manager);

    /**
     * 因为对车道做了分割，这里会创建子图，
     * 比如一个车道分成2个子节点，2个子节点会创建一张子图。
     * **/
    SubTopoGraph sub_graph(full_range_manager.RangeMap());

    // 获取起点
    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
    if (start == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s;
      return false;
    }

    // 获取终点
    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
    if (end == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s;
      return false;
    }

    std::vector<NodeWithRange> cur_result_nodes;
    // 通过Astar查找最优路径
    if (!strategy_ptr->Search(graph, &sub_graph, start, end,
                              &cur_result_nodes)) {
      AERROR << "Failed to search route with waypoint from " << start->LaneId()
             << " to " << end->LaneId();
      return false;
    }

    // 保存结果到node_vec
    node_vec.insert(node_vec.end(), cur_result_nodes.begin(),
                    cur_result_nodes.end());
  }

  // 合并Route
  if (!MergeRoute(node_vec, result_nodes)) {
    AERROR << "Failed to merge route.";
    return false;
  }
  return true;
}

/**
 * 步骤1： 对请求参数进行检查；
 * 步骤2:  判断自身是否处于就绪状态；
 * 步骤3： 初始化请求需要的参数；
 * 步骤4： 执行搜索算法；
 * 步骤5： 组装搜索结果；
 * **/
bool Navigator::SearchRoute(const RoutingRequest& request,
                            RoutingResponse* const response) {
  // 步骤1
  if (!ShowRequestInfo(request, graph_.get())) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST,
                 "Error encountered when reading request point!",
                 response->mutable_status());
    return false;
  }

  // 步骤2
  if (!IsReady()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!",
                 response->mutable_status());
    return false;
  }

  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  // 步骤3 根据request中的waypoint填充way_nodes, way_s
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY,
                 "Failed to initialize navigator!", response->mutable_status());
    return false;
  }

  std::vector<NodeWithRange> result_nodes;
  // 步骤4
  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to find route with request!",
                 response->mutable_status());
    return false;
  }

  if (result_nodes.empty()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!",
                 response->mutable_status());
    return false;
  }

  result_nodes.front().SetStartS(request.waypoint().begin()->s());
  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());
  // 步骤5 组装搜索结果
  if (!result_generator_->GeneratePassageRegion(
          graph_->MapVersion(), request, result_nodes, topo_range_manager_,
          response)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to generate passage regions based on result lanes",
                 response->mutable_status());
    return false;
  }

  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());

  PrintDebugData(result_nodes);
  return true;
}

}  // namespace routing
}  // namespace apollo
