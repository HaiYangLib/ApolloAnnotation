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

#include "modules/routing/topo_creator/graph_creator.h"

#include <vector>

#include "absl/strings/match.h"
#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/topo_creator/edge_creator.h"
#include "modules/routing/topo_creator/node_creator.h"

namespace apollo {
namespace routing {

using apollo::common::PointENU;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::kMathEpsilon;
using apollo::common::math::Vec2d;
using apollo::hdmap::Id;
using apollo::hdmap::LaneBoundary;
using apollo::hdmap::LaneBoundaryType;
using ::google::protobuf::RepeatedPtrField;

namespace {

/**
 * 判断是否允许向指定车道边界变道
 * **/
bool IsAllowedToCross(const LaneBoundary& boundary) {
  for (const auto& boundary_type : boundary.boundary_type()) {
    if (boundary_type.types(0) != LaneBoundaryType::DOTTED_YELLOW &&
        boundary_type.types(0) != LaneBoundaryType::DOTTED_WHITE) {
      return false;
    }
  }
  return true;
}

}  // namespace

/**
 * 对于定义的GraphCreator类对象的构造函数初始化，定义输入、输出地图路径及
 * routing_conf配置
 * **/
GraphCreator::GraphCreator(const std::string& base_map_file_path,
                           const std::string& dump_topo_file_path,
                           const RoutingConfig& routing_conf)
    : base_map_file_path_(base_map_file_path),
      dump_topo_file_path_(dump_topo_file_path),
      routing_conf_(routing_conf) {}

/**
 * 核心函数：建立拓扑地图函数Create()
 * **/
bool GraphCreator::Create() {
  /**
   * 这里注意，有2种格式，一种是Opendrive格式，通过OpendriveAdapter来读取
   * 另外一种是apollo自己定义的格式(protobuf格式)。
   *
   * 成员变量pbmap_在下面文件中定义：
   *
   *
   * modules/map/proto/map.proto
   * message Map {
   * optional Header header = 1;
   *
   * repeated Crosswalk crosswalk = 2;
   * repeated Junction junction = 3;
   * repeated Lane lane = 4;
   * repeated StopSign stop_sign = 5;
   * repeated Signal signal = 6;
   * repeated YieldSign yield = 7;
   * repeated Overlap overlap = 8;
   * repeated ClearArea clear_area = 9;
   * repeated SpeedBump speed_bump = 10;
   * repeated Road road = 11;
   * repeated ParkingSpace parking_space = 12;
   * repeated PNCJunction pnc_junction = 13;
   * repeated RSU rsu = 14;
   * }
   * **/
  if (absl::EndsWith(base_map_file_path_, ".xml")) {
    if (!hdmap::adapter::OpendriveAdapter::LoadData(base_map_file_path_,
                                                    &pbmap_)) {
      AERROR << "Failed to load base map file from " << base_map_file_path_;
      return false;
    }
  } else {
    /**
     * 从输入路径加载protobuf格式地图到pbmap_
     * **/
    if (!cyber::common::GetProtoFromFile(base_map_file_path_, &pbmap_)) {
      AERROR << "Failed to load base map file from " << base_map_file_path_;
      return false;
    }
  }

  AINFO << "Number of lanes: " << pbmap_.lane_size();

  /**
   * 成员成员变量在下面文件中定义:
   * modules/routing/proto/topo_graph.proto
   *
   * graph_为最后得到的图，消息格式在topo_graph.proto中申明
   *
   * message Graph {
   * optional string hdmap_version = 1;
   * optional string hdmap_district = 2;
   * repeated Node node = 3;
   * repeated Edge edge = 4;
   * }
   *
   * **/
  graph_.set_hdmap_version(pbmap_.header().version());
  graph_.set_hdmap_district(pbmap_.header().district());

  /**
   * 用来储存道路，车道，节点之间的对应关系
   * **/
  node_index_map_.clear();
  road_id_map_.clear();
  showed_edge_id_set_.clear();

  /**
   * 遍历base_map中的lane,将lane与road id对应起来
   * **/
  for (const auto& road : pbmap_.road()) {
    for (const auto& section : road.section()) {
      for (const auto& lane_id : section.lane_id()) {
        road_id_map_[lane_id.id()] = road.id().id();
      }
    }
  }

  /**
   * 初始化禁止车道，将类型不为city_driving的车道设置放入set容器里
   * **/
  InitForbiddenLanes();

  /**
   * 从配置文件中读取最小掉头半径
   * 加载最小转弯半径\apollo\modules\common\configs\proto\vehicle_config.proto
   *
   * **/
  const double min_turn_radius =
      VehicleConfigHelper::GetConfig().vehicle_param().min_turn_radius();

  /**
   * 遍历pbmap_中的车道,创建节点。
   * **/
  for (const auto& lane : pbmap_.lane()) {
    const auto& lane_id = lane.id().id();
    /**
     * 跳过不是城市道路(CITY_DRIVING)的车道
     * **/
    if (forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end()) {
      ADEBUG << "Ignored lane id: " << lane_id
             << " because its type is NOT CITY_DRIVING.";
      continue;
    }

    /**
     * 转弯半径过小则跳过
     * **/
    if (lane.turn() == hdmap::Lane::U_TURN &&
        !IsValidUTurn(lane, min_turn_radius)) {
      ADEBUG << "The u-turn lane radius is too small for the vehicle to turn";
      continue;
    }

    AINFO << "Current lane id: " << lane_id;
    // 存储node index和lane id的关系
    node_index_map_[lane_id] = graph_.node_size();

    /**
     * 从road_id_map里找lane id，在创建节点时指定道路id,没找到则道路id为空，
     * 创建节点引用
     * **/
    const auto iter = road_id_map_.find(lane_id);
    if (iter != road_id_map_.end()) {
      node_creator::GetPbNode(lane, iter->second, routing_conf_,
                              graph_.add_node());
    } else {
      AWARN << "Failed to find road id of lane " << lane_id;
      node_creator::GetPbNode(lane, "", routing_conf_, graph_.add_node());
    }
  }

  /**
   * 再次遍历pbmap_中的车道,创建边
   * **/
  for (const auto& lane : pbmap_.lane()) {
    const auto& lane_id = lane.id().id();
    if (forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end()) {
      ADEBUG << "Ignored lane id: " << lane_id
             << " because its type is NOT CITY_DRIVING.";
      continue;
    }

    /**
     * 对于遍历的每一个车道其对应的节点去创建边
     * **/
    const auto& from_node = graph_.node(node_index_map_[lane_id]);

    /**
     * 增加当前节点向前的边
     * **/
    AddEdge(from_node, lane.successor_id(), Edge::FORWARD);

    /**
     * 
     * DEFINE_double(min_length_for_lane_change, 1.0,
     *         "meters, which is 100 feet.  Minimum distance needs to travel on "
     *         "a lane before making a lane change. Recommended by "
     *         "https://www.oregonlaws.org/ors/811.375");
     * 
     * 如果车道长度小于最小变道长度，则直接进入下一个循环
     * 
     * **/
    if (lane.length() < FLAGS_min_length_for_lane_change) {
      continue;
    }

    /**
     * 如果该车道有左边界且允许变道，增加向左边邻居变道的边
     * **/
    if (lane.has_left_boundary() && IsAllowedToCross(lane.left_boundary())) {
      AddEdge(from_node, lane.left_neighbor_forward_lane_id(), Edge::LEFT);
    }

    /**
     * 如果该车道有右边界且允许变道，增加向右边邻居变道的边
     * **/
    if (lane.has_right_boundary() && IsAllowedToCross(lane.right_boundary())) {
      AddEdge(from_node, lane.right_neighbor_forward_lane_id(), Edge::RIGHT);
    }
  }

  /**
   * 如果输出路径下的文件不是以bin && txt为后缀名，则显示错误信息
   * **/
  if (!absl::EndsWith(dump_topo_file_path_, ".bin") &&
      !absl::EndsWith(dump_topo_file_path_, ".txt")) {
    AERROR << "Failed to dump topo data into file, incorrect file type "
           << dump_topo_file_path_;
    return false;
  }

  auto type_pos = dump_topo_file_path_.find_last_of(".") + 1;
  std::string bin_file = dump_topo_file_path_.replace(type_pos, 3, "bin");
  std::string txt_file = dump_topo_file_path_.replace(type_pos, 3, "txt");

  /**
   * 输出拓扑地图graph_到bin,txt
   * **/
  if (!cyber::common::SetProtoToASCIIFile(graph_, txt_file)) {
    AERROR << "Failed to dump topo data into file " << txt_file;
    return false;
  }
  AINFO << "Txt file is dumped successfully. Path: " << txt_file;
  if (!cyber::common::SetProtoToBinaryFile(graph_, bin_file)) {
    AERROR << "Failed to dump topo data into file " << bin_file;
    return false;
  }
  AINFO << "Bin file is dumped successfully. Path: " << bin_file;
  return true;
}

/**
 * 获取边的ID
 * **/
std::string GraphCreator::GetEdgeID(const std::string& from_id,
                                    const std::string& to_id) {
  return from_id + "->" + to_id;
}

/**
 * 增加边
 * **/
void GraphCreator::AddEdge(const Node& from_node,
                           const RepeatedPtrField<Id>& to_node_vec,
                           const Edge::DirectionType& type) {
  for (const auto& to_id : to_node_vec) {
    if (forbidden_lane_id_set_.find(to_id.id()) !=
        forbidden_lane_id_set_.end()) {
      ADEBUG << "Ignored lane [id = " << to_id.id();
      continue;
    }
    const std::string edge_id = GetEdgeID(from_node.lane_id(), to_id.id());
    if (showed_edge_id_set_.count(edge_id) != 0) {
      continue;
    }
    
    showed_edge_id_set_.insert(edge_id);
    const auto& iter = node_index_map_.find(to_id.id());
    if (iter == node_index_map_.end()) {
      continue;
    }
    const auto& to_node = graph_.node(iter->second);
    edge_creator::GetPbEdge(from_node, to_node, type, routing_conf_,
                            graph_.add_edge());
  }
}

/**
 * 判断是否是有效的掉头，根据最小转弯半径而定
 * **/
bool GraphCreator::IsValidUTurn(const hdmap::Lane& lane, const double radius) {
  if (lane.turn() != hdmap::Lane::U_TURN) {  // not a u-turn
    return false;
  }
  // approximate the radius from start point, middle point and end point.
  if (lane.central_curve().segment().empty() ||
      !lane.central_curve().segment(0).has_line_segment()) {
    return false;
  }
  std::vector<PointENU> points;
  for (const auto& segment : lane.central_curve().segment()) {
    points.insert(points.end(), segment.line_segment().point().begin(),
                  segment.line_segment().point().end());
  }
  if (points.empty()) {
    return false;
  }
  Vec2d p1{points.front().x(), points.front().y()};
  const auto& mid = points[points.size() / 2];
  Vec2d p2{mid.x(), mid.y()};
  Vec2d p3{points.back().x(), points.back().y()};
  Vec2d q1 = ((p1 + p2) / 2);                  // middle of p1---p2
  Vec2d q2 = (p2 - p1).rotate(M_PI / 2) + q1;  // perpendicular to p1-p2
  Vec2d q3 = ((p2 + p3) / 2);                  // middle of p2 -- p3
  Vec2d q4 = (p3 - p2).rotate(M_PI / 2) + q3;  // perpendicular to p2-p3
  const double s1 = CrossProd(q3, q1, q2);
  if (std::fabs(s1) < kMathEpsilon) {  // q3 is the circle center
    return q3.DistanceTo(p1) >= radius;
  }
  const double s2 = CrossProd(q4, q1, q2);
  if (std::fabs(s2) < kMathEpsilon) {  // q4 is the circle center
    return q4.DistanceTo(p1) >= radius;
  }
  if (std::fabs(s1 - s2) < kMathEpsilon) {  // parallel case, a wide u-turn
    return true;
  }
  Vec2d center = q3 + (q4 - q3) * s1 / (s1 - s2);
  return p1.DistanceTo(center) >= radius;
}

/**
 * 初始化被禁止的车道(类型不是city_driving的)
 * enum LaneType {               //车道类型
 *   NONE = 1;                   //无
 *   CITY_DRIVING = 2;           //城市道路
 *   BIKING = 3;                 //自行车
 *   SIDEWALK = 4;               //人行道
 *   PARKING = 5;                //停车
 * };
 * **/
void GraphCreator::InitForbiddenLanes() {
  for (const auto& lane : pbmap_.lane()) {
    if (lane.type() != hdmap::Lane::CITY_DRIVING) {
      forbidden_lane_id_set_.insert(lane.id().id());
    }
  }
}

}  // namespace routing
}  // namespace apollo
