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

#include "modules/map/pnc_map/pnc_map.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "google/protobuf/text_format.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/routing/common/routing_gflags.h"

DEFINE_double(
    look_backward_distance, 50,
    "look backward this distance when creating reference line from routing");

DEFINE_double(look_forward_short_distance, 180,
              "short look forward this distance when creating reference line "
              "from routing when ADC is slow");
DEFINE_double(
    look_forward_long_distance, 250,
    "look forward this distance when creating reference line from routing");

namespace apollo {
namespace hdmap {

using apollo::common::PointENU;
using apollo::common::VehicleState;
using apollo::common::util::PointFactory;
using apollo::routing::RoutingResponse;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace

PncMap::PncMap(const HDMap *hdmap) : hdmap_(hdmap) {}

const hdmap::HDMap *PncMap::hdmap() const { return hdmap_; }

LaneWaypoint PncMap::ToLaneWaypoint(
    const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return LaneWaypoint(lane, waypoint.s());
}

double PncMap::LookForwardDistance(double velocity) {
  /**
   * DEFINE_double(look_forward_time_sec, 8.0,
   *           "look forward time times adc speed to calculate this distance "
   *          "when creating reference line from routing");
   * **/
  auto forward_distance = velocity * FLAGS_look_forward_time_sec;

  /**
   * DEFINE_double(look_forward_short_distance, 180,
   *            "short look forward this distance when creating reference line "
   *            "from routing when ADC is slow");
   *
   * DEFINE_double(look_forward_long_distance, 250,
   *     "look forward this distance when creating reference line from routing")
   * **/
  return forward_distance > FLAGS_look_forward_short_distance
             ? FLAGS_look_forward_long_distance
             : FLAGS_look_forward_short_distance;
}

LaneSegment PncMap::ToLaneSegment(const routing::LaneSegment &segment) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return LaneSegment(lane, segment.start_s(), segment.end_s());
}

void PncMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }
  // Search backwards when the car is driven backward on the route.
  /**
   * 情况1. 车道倒车，后向查找，下一个查询点waypoint对应的索引查找
   * **/
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index >
             cur_index) {
    --next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index ==
             cur_index &&
         adc_waypoint_.s <
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  // Search forwards

  /**
   * 情况2. 车道前进，前向查找，下一个查询点waypoint对应的索引查找
   * **/
  //  第1步，查找最近包含有waypoint的LaneSegment
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index <
             cur_index) {
    ++next_routing_waypoint_index_;
  }

  // 第2步，查找下一个最近的waypoint对应的索引
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index ==
             routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >=
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }

  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

std::vector<routing::LaneWaypoint> PncMap::FutureRouteWaypoints() const {
  const auto &waypoints = routing_.routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

void PncMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear();
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

/**
 * 这里的更新VehicleState并不是更新车辆的位置、速度这些状态，而是根据自车状态，
 * 从routing中查找到最近的道路点adc_waypoint，
 * 并根据adc_waypoint来在route_indices_中查找到当前行驶至了routing的哪条LaneSegment上，
 * 并将index赋给adc_route_index_（由GetWaypointIndex()函数计算得出）。
 * 另外，计算自车下一个必经的waypoint点，
 * 并赋给next_routing_waypoint_index_（由UpdateNextRoutingWaypointIndex()函数计算得出）。
 * **/
bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  if (!ValidateRouting(routing_)) {
    AERROR << "The routing is invalid when updating vehicle state.";
    return false;
  }

  /**
   * DEFINE_double(replan_lateral_distance_threshold, 0.5,
   *          "The lateral distance threshold of replan");
   * DEFINE_double(replan_longitudinal_distance_threshold, 2.5,
   *          "The longitudinal distance threshold of replan");
   *
   * **/
  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) >
       FLAGS_replan_lateral_distance_threshold +
           FLAGS_replan_longitudinal_distance_threshold)) {
    // Position is reset, but not replan.
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  /**
   * 计算当前车辆在对应车道上的投影adc_waypoint_
   * **/
  adc_state_ = vehicle_state;
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    AERROR << "Failed to get waypoint from routing with point: "
           << "(" << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    return false;
  }

  /**
   * 计算车辆投影点所在LaneSegment在route_indices_的索引adc_route_index_
   * **/
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }

  // Track how many routing request waypoints the adc have passed.
  // 更新next_routing_waypoint_index_
  UpdateNextRoutingWaypointIndex(route_index);
  adc_route_index_ = route_index;

  UpdateRoutingRange(adc_route_index_);

  if (routing_waypoint_index_.empty()) {
    AERROR << "No routing waypoint index.";
    return false;
  }

  // 如果next_routing_waypoint_index_是终点的索引，表示已经到达目的地。
  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) {
    stop_for_destination_ = true;
  }
  return true;
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &routing) const {
  return IsNewRouting(routing_, routing);
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &prev,
                          const routing::RoutingResponse &routing) {
  if (!ValidateRouting(routing)) {
    ADEBUG << "The provided routing is invalid.";
    return false;
  }
  return !common::util::IsProtoEqual(prev, routing);
}

bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  range_lane_ids_.clear();

  /**
   * 成员变量由下面定义：
   *
   * struct RouteIndex {
   * LaneSegment segment;
   * std::array<int, 3> index;
   * };
   * std::vector<RouteIndex> route_indices_;
   *
   * 其中
   * segment=LaneSegment的id
   * index={RoadSegment索引, Passage索引, LaneSegment索引}
   * **/
  route_indices_.clear();
  all_lane_ids_.clear();

  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        route_indices_.back().segment =
            ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};
      }
    }
  }

  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1;
  next_routing_waypoint_index_ = 0;

  // 更新range_lane_ids_
  UpdateRoutingRange(adc_route_index_);

  routing_waypoint_index_.clear();
  const auto &request_waypoints = routing.routing_request().waypoint();
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }

  int i = 0;

  /**
   * routing_waypoint_index_保存着request_waypoints所在的lane
   * **/
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    /**
     * request_waypoints.Get(i))是否在route_indices_[j].segment中
     *
     * **/
    while (i < request_waypoints.size() &&
           RouteSegments::WithinLaneSegment(route_indices_[j].segment,
                                            request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(
          LaneWaypoint(route_indices_[j].segment.lane,
                       request_waypoints.Get(i).s()),
          j);
      ++i;
    }
  }

  routing_ = routing;
  adc_waypoint_ = LaneWaypoint();
  stop_for_destination_ = false;
  return true;
}

const routing::RoutingResponse &PncMap::routing_response() const {
  return routing_;
}

bool PncMap::ValidateRouting(const RoutingResponse &routing) {
  const int num_road = routing.road_size();
  if (num_road == 0) {
    AERROR << "Route is empty.";
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

int PncMap::SearchForwardWaypointIndex(int start,
                                       const LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (
      i < static_cast<int>(route_indices_.size()) &&
      !RouteSegments::WithinLaneSegment(route_indices_[i].segment, waypoint)) {
    ++i;
  }
  return i;
}

int PncMap::SearchBackwardWaypointIndex(int start,
                                        const LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                     waypoint)) {
    --i;
  }
  return i;
}

int PncMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int PncMap::GetWaypointIndex(const LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

// 将passage上所有segment(LaneSegment),组装成RouteSegments
bool PncMap::PassageToSegments(routing::Passage passage,
                               RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  for (const auto &lane : passage.segment()) {
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

std::vector<int> PncMap::GetNeighborPassages(const routing::RoadSegment &road,
                                             int start_passage) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  std::vector<int> result;

  // 当前通道(Passage)
  const auto &source_passage = road.passage(start_passage);
  result.emplace_back(start_passage);

  /**
   * 如果当前通道(Passage)是直行道(change_lane_type==routing::FORWARD)，无法变道，
   * 那么直接返回车辆所在的车道
   * **/
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }

  /**
   * 如果当前通道已经准备退出(can_exit=True)
   * 车辆已经准备进入下一个Passage，不需要变道，直接返回车辆所在的车道
   * **/
  if (source_passage.can_exit()) {  // No need to change lane
    return result;
  }

  RouteSegments source_segments;
  // 得到source_passage上所有LaneSegment,组装成source_segments
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return result;
  }

  /**
   * 如果下一个查询点(routing_waypoint_index_[next_routing_waypoint_index_].waypoint)
   * 在当前通道中，不需要变道，直接返回车辆所在的车道
   * **/
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(
          routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
    return result;
  }

  /**
   * 如果车辆在左转车道或者右转车道，从高精地图hdmap中
   * 查询当前车道对应左侧或者右侧的所有车道线，
   * 然后去和当前RoadSegment.passage()去做对比，找到两者共同包含的车道，就是最终的邻接车道
   * **/
  std::unordered_set<std::string> neighbor_lanes;
  // 当前passage是左转通道
  if (source_passage.change_lane_type() == routing::LEFT) {
    // 查询当前Passage中每个LaneSegment所在车道的邻接左车道
    for (const auto &segment : source_segments) {
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } else if (source_passage.change_lane_type() == routing::RIGHT) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }

  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {
      // 查询当前RoadSegment中所有Passage::LaneSegment的所属车道，有交集就添加到结果中
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i);
        break;
      }
    }
  }

  return result;
}

bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              std::list<RouteSegments> *const route_segments) {
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());

  /**
   * DEFINE_double(look_backward_distance, 50,
   *  "look backward this distance when creating reference line from routing");
   * **/
  /**
   * 默认情况下
   * look_forward_distance=180或250
   * look_backward_distance=50
   * **/
  double look_backward_distance = FLAGS_look_backward_distance;
  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments);
}

/**
 * 这是pnc map最重要的功能，从函数参数分析，
 * 可以看到GetRouteSegments接受的参数最重要的是车辆的状态(包含车辆的位置，速度，偏航角，加速度等信息)，
 * backward_length和forward_length是路径短生成过程中前向与后向修正距离
 *
 * 而返回的是短期内(注意是短期内，长期不确定因素太多，无法实现)车辆的运动轨迹route_segments
 * 每个元素都是代表当前车辆的一种运动方案。
 *
 * 默认情况下
 * look_forward_distance=180或250
 * look_backward_distance=50
 *
 * 对每个邻接车道做一个是否可驶入的检查，并做道路段截取。
 * 也就是制定出无人车在当前情况下可能行驶的区域，每个可行驶通道将划分出一个道路区间，
 * 道路区间的长度由前向查询距离forward_length和后向查询距离backward_length决定，
 * 短期内规划的可行驶道路段长度为forward_length+backward_length。
 * 最终的路由段RouteSegments生成只需要对每个邻接可驶入的通道进行Segments生成即可。
 * **/
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments) {
  /**
   * 这里的无人车状态不是无人车自身坐标，速度等信息，
   * 而是在上述更新路由信息过程中得到的route_indices_中，
   * 无人车在哪个LaneSegment中，距离无人车最近的下一个查询点waypoint的信息。
   *
   * 步骤1：更新pnc map中无人车状态
   * **/
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }

  /**
   *  struct RouteIndex {
   *    LaneSegment segment;
   *    std::array<int, 3> index;
   *  };
   *  std::vector<RouteIndex> route_indices_;
   * **/
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0];     // RoadSegment
  const int passage_index = route_index[1];  // Passage
  const auto &road = routing_.road(road_index);
  // Raw filter to find all neighboring passages
  /**
   * 查询当前位置下，附近的通道(passage)
   *
   * 步骤2：计算临近通道
   * **/
  auto drive_passages = GetNeighborPassages(road, passage_index);

  // 步骤3：创建车辆当前可行驶区域
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index);
    RouteSegments segments;
    // 将passage上所有segment(LaneSegment),组装成segments(RouteSegments)
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
    // 步骤3.1: 将当前车辆的坐标投影到Passage
    const PointENU nearest_point =
        index == passage_index
            ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
            : PointFactory::ToPointENU(adc_state_);

    common::SLPoint sl;
    LaneWaypoint segment_waypoint;
    // nearest_point(adc_state_)在segments上的投影
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }

    // 步骤3.2: 检查Passage是否可驶入
    if (index != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }

    /**
     * 这部分就是对上述通过可驶入合法性检查的车道进行道路段的生成，
     * 同时使用backward_length和backward_length前后扩展道路段。
     * **/
    // 步骤3.3: 生成RouteSegmens
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();
    /**
     * 原本车辆在passage中的投影点累计距离为sl.s
     * (注意这个s是投影点在passage段起点的累计距离，而非整个road的累计距离)，
     * 扩展后前向增加forward_length，后向增加backward_length
     * **/
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }

    /**
     * 在Step 3.1-3.3中，我们已经完成了一条passage对应的路由段生成，
     * 最终就需要添加这个路由段的一些属性**/
    // 步骤3.4:设置RouteSegments属性
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }

    // 是否可以退出通道(最后一段Segment决定)
    route_segments->back().SetCanExit(passage.can_exit());
    // 下一步的动作(最后一段Segment决定)
    route_segments->back().SetNextAction(passage.change_lane_type());
    // RouteSegments的id和是否是目的地
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);
    // 设置上时刻的状态
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(routing::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(routing::LEFT);
    }
  }

  return !route_segments->empty();
}

/**
 * 步骤1：根据当前车辆坐标(x,y)以及速度方向heading，
 *  去高精地图hd map查询车辆附近的车道(车道方向必须与heading方向夹角在90度以内，
 *  意味着都是相同方向的车道，超过90度可能是反向车道)
 *
 * 步骤2：计算查询得到的车道和range_lane_ids_或者all_lane_ids_的交集，
 *  也就是查找RoutingResponse.road().passage()中的附近的车道。
 *
 * 步骤3：对过滤得到的车道进行车辆坐标投影，可计算车辆到各个车道的距离，
 *  取最小距离的车道作为最终的投影车道，得到adc_waypoint_的车道id和累积距离s，
 *  其实也可以计算投影点，但是这里没有计算。
 * **/
bool PncMap::GetNearestPointFromRouting(const VehicleState &state,
                                        LaneWaypoint *waypoint) const {
  const double kMaxDistance = 10.0;  // meters.
  const double kHeadingBuffer = M_PI / 10.0;
  waypoint->lane = nullptr;
  std::vector<LaneInfoConstPtr> lanes;
  const auto point = PointFactory::ToPointENU(state);

  // 步骤1
  const int status =
      hdmap_->GetLanesWithHeading(point, kMaxDistance, state.heading(),
                                  M_PI / 2.0 + kHeadingBuffer, &lanes);
  ADEBUG << "lanes:" << lanes.size();
  if (status < 0) {
    AERROR << "Failed to get lane from point: " << point.ShortDebugString();
    return false;
  }
  if (lanes.empty()) {
    AERROR << "No valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }

  std::vector<LaneInfoConstPtr> valid_lanes;
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](LaneInfoConstPtr ptr) {
                 return range_lane_ids_.count(ptr->lane().id().id()) > 0;
               });
  // 步骤2
  if (valid_lanes.empty()) {
    std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
                 [&](LaneInfoConstPtr ptr) {
                   return all_lane_ids_.count(ptr->lane().id().id()) > 0;
                 });
  }

  // Get nearest_waypoints for current position
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &lane : valid_lanes) {
    if (range_lane_ids_.count(lane->id().id()) == 0) {
      continue;
    }

    {
      double s = 0.0;
      double l = 0.0;

      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        AERROR << "fail to get projection";
        return false;
      }
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
    }

    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    if (distance < min_distance) {
      min_distance = distance;
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "Failed to get projection for map_point: "
               << map_point.DebugString();
        return false;
      }
      waypoint->lane = lane;
      waypoint->s = s;
    }

    ADEBUG << "distance" << distance;
  }

  if (waypoint->lane == nullptr) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

LaneInfoConstPtr PncMap::GetRouteSuccessor(LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

LaneInfoConstPtr PncMap::GetRoutePredecessor(LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (size_t i = 1; i < route_indices_.size(); ++i) {
    auto &lane = route_indices_[i].segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

bool PncMap::ExtendSegments(const RouteSegments &segments,
                            const common::PointENU &point, double look_backward,
                            double look_forward,
                            RouteSegments *extended_segments) {
  common::SLPoint sl;
  LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

/**
 * 注意这个s是投影点在passage段起点的累计距离，而非整个road的累计距离)，扩展后前向增加forward_length，
 * 后向增加backward_length
 * **/
bool PncMap::ExtendSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  /**
   * 当车辆投影点所在车道的后向查询起始点小于0，即s1.s() - backward_length < 0，
   * 此时就需要查询first_lane_segment对应的车道前置部分进行截取，
   * 如果车道前置部分仍然不够长度fabs(s1.s() + backward_length)，
   * 那么就需要加入这条车道的前置车道继续截取。
   * **/
  // 当后向查询起始点小于0，说明需要用到这条lane的前置lane
  if (start_s < 0) {
    const auto &first_segment = *segments.begin();
    // 或者passage的第一个LaneSegment的所属车道
    auto lane = first_segment.lane;
    double s = first_segment.start_s;
    // extend_s为需要从前置车道中截取的道路段长度，初始化为-start_s
    double extend_s = -start_s;

    /**
     * LaneSegment:
     * LaneInfoConstPtr lane = nullptr;
     * double start_s = 0.0;
     * double end_s = 0.0;
     * **/
    std::vector<LaneSegment> extended_lane_segments;
    // 每次循环(截取)以后extend_s都会减小，直至到0
    while (extend_s > kRouteEpsilon) {
      // s < 0，则需要在查询这条lane对应的前置车道，进行截取
      if (s <= kRouteEpsilon) {
        // 获取当前lane的前置车道
        lane = GetRoutePredecessor(lane);
        if (lane == nullptr ||
            unique_lanes.find(lane->id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else {  // 如果s > 0，此时就可以从这条前置lane中截取道路段
        const double length = std::min(s, extend_s);
        // 截取道路段
        extended_lane_segments.emplace_back(lane, s - length, s);
        // 更新extend_s，如果extend_s>0，说明还需要继续寻找前置道路段截取
        extend_s -= length;
        s -= length;
        unique_lanes.insert(lane->id().id());
      }
    }

    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }

  bool found_loop = false;

  /**
   * 下面部分就是正常的passage中LaneSegment截取，根据start_s和end_s
   *
   * router_s代表已经累计截取到了的LaneSegment长度，
   * 如果当前正在截取第3个LaneSegment，那么router_s就是前两个LaneSegment的长度和
   * **/
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    /**
     * 计算当前LaneSegment需要截取的start_s和end_s，非最后一段，
     * start_s和end_s就是这个LaneSegment的start_s和end_s，意味着整段截取
     * **/
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      /**
       * 有前置车道的，如果前置最后一段的前置车道和当前LaneSegment的车道相同，
       * 那么需要合并(修改end_s即可)；否则新建一段加入list
       * **/
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() ==
              lane_segment.lane->id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->id().id()) ==
                 unique_lanes.end()) {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      } else {
        found_loop = true;
        break;
      }
    }

    /**
     * 判断是否截取结束，如果结束了那么可以退出，否则就需要继续截取，
     * 当最后循环最后一次最后一个LaneSegment还是没有结束，那么就需要新增加后置车道继续处理
     * **/
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }

  if (found_loop) {
    return true;
  }

  // Extend the trajectory towards the end of the trajectory.

  /**
   * 常规的passage中LaneSegment截取结束后，如果router_s仍然小于end_s，
   * 就说明车道截取还未结束，还有一段长度end_s - router_s的道路段未被截取，
   * 此时passage中的LaneSegment已经全部截取完了，所以需要访问最后一个LaneSegment对应的lane，
   * 需要继续截取这条lane的后续部分，如果后续部分长度仍然不够，就需要加入这条lane的后接车道继续截取。
   * **/
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    // 查找最后一个LaneSegment对应的车道，继续从该车道截取
    if (back.lane->total_length() > back.end_s) {
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }

  auto last_lane = segments.back().lane;
  while (router_s < end_s - kRouteEpsilon) {
    last_lane = GetRouteSuccessor(last_lane);
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->id().id()) != unique_lanes.end()) {
      break;
    }
    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->id().id());
    router_s += length;
  }

  return true;
}

void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                const double end_s,
                                std::vector<MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points->emplace_back(segment.start() + segment.unit_direction() *
                                                   (start_s - accumulate_s),
                             lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

}  // namespace hdmap
}  // namespace apollo
