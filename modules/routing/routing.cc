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

#include <limits>
#include <unordered_map>

#include "modules/routing/routing.h"

#include "modules/common/util/point_factory.h"
#include "modules/routing/common/routing_gflags.h"

namespace apollo {
namespace routing {

using apollo::common::ErrorCode;
using apollo::common::PointENU;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

std::string Routing::Name() const { return FLAGS_routing_node_name; }

Routing::Routing()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::ROUTING) {}

apollo::common::Status Routing::Init() {
  /**
   * 默认情况下：
   * routing_map_file=modules/map/data/demo/routing_map.txt
   * **/
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  navigator_ptr_.reset(new Navigator(routing_map_file));

  /**
   * 默认情况下：
   * 使用odules/map/data/demo/routing_map.txt填充Map map_;(HDMapImpl成员变量)
   * 
   * 用来查找routing request请求的点距离最近的lane，
   * 并且返回对应的lane id，这里很好理解，比如你在小区里面，需要打车，
   * 需要找到最近的乘车点，说直白点，就是找到最近的路。
   * **/
  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

  return apollo::common::Status::OK();
}

apollo::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return apollo::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";
  monitor_logger_buffer_.INFO("Routing started");
  return apollo::common::Status::OK();
}

std::vector<RoutingRequest> Routing::FillLaneInfoIfMissing(
    const RoutingRequest& routing_request) {
  std::vector<RoutingRequest> fixed_requests;
  std::unordered_map<int, std::vector<LaneWaypoint>>
      additional_lane_waypoint_map;

  RoutingRequest fixed_request(routing_request);
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    LaneWaypoint lane_waypoint(routing_request.waypoint(i));
    if (lane_waypoint.has_id()) {
      continue;
    }

    // fill lane info when missing
    const auto point =
        common::util::PointFactory::ToPointENU(lane_waypoint.pose());
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    // look for lanes with bigger radius if not found
    /**
     * 防止选择的点距离车道线较远,无法匹配，增加6米范围的冗余
     * 寻找在这个范围内的车道线信息
     * **/
    constexpr double kRadius = 0.3;
    for (int i = 0; i < 20; ++i) {
      /**
       * 通过已建立好的kdtree，快速搜索到满足条件的车道线信息
       * **/
      hdmap_->GetLanes(point, kRadius + i * kRadius, &lanes);
      if (lanes.size() > 0) {
        break;
      }
    }

    if (lanes.empty()) {
      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      return fixed_requests;  // return empty vector
    }

    /**
     * 搜索过程中，存在因车道重叠而增加的lane信息，
     * 将该部分车道信息也增加如修正后的请求信息（fixed_requests）中，
     * 到此完成输入请求点信息的修正，用于后续进行路径搜索
     * **/
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      double l = 0.0;
      lanes[j]->GetProjection({point.x(), point.y()}, &s, &l);

      if (j == 0) {
        auto waypoint_info = fixed_request.mutable_waypoint(i);
        waypoint_info->set_id(lanes[j]->id().id());
        waypoint_info->set_s(s);
      } else {
        // additional candidate lanes
        LaneWaypoint new_lane_waypoint(lane_waypoint);
        new_lane_waypoint.set_id(lanes[j]->id().id());
        new_lane_waypoint.set_s(s);
        additional_lane_waypoint_map[i].push_back(new_lane_waypoint);
      }

    }
  }


  // first routing_request
  fixed_requests.push_back(fixed_request);

  // additional routing_requests because of lane overlaps
  // 排列组合
  for (const auto& m : additional_lane_waypoint_map) { 
    size_t cur_size = fixed_requests.size();
    for (size_t i = 0; i < cur_size; ++i) {
      // use index to iterate while keeping push_back
      for (const auto& lane_waypoint : m.second) {
        RoutingRequest new_request(fixed_requests[i]);
        auto waypoint_info = new_request.mutable_waypoint(m.first);
        waypoint_info->set_id(lane_waypoint.id());
        waypoint_info->set_s(lane_waypoint.s());
        fixed_requests.push_back(new_request);
      }
    }
  }

  for (const auto& fixed_request : fixed_requests) {
    ADEBUG << "Fixed routing request:" << fixed_request.DebugString();
  }
  
  return fixed_requests;
}

bool Routing::GetParkingID(const PointENU& parking_point,
                           std::string* parking_space_id) {
  // search current parking space id associated with parking point.
  constexpr double kDistance = 0.01;  // meter
  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  if (hdmap_->GetParkingSpaces(parking_point, kDistance, &parking_spaces) ==
      0) {
    *parking_space_id = parking_spaces.front()->id().id();
    return true;
  }
  return false;
}

bool Routing::FillParkingID(RoutingResponse* routing_response) {
  const auto& routing_request = routing_response->routing_request();
  const bool has_parking_info = routing_request.has_parking_info();
  const bool has_parking_id =
      has_parking_info && routing_request.parking_info().has_parking_space_id();
  // return early when has parking_id
  if (has_parking_id) {
    return true;
  }
  // set parking space ID when
  //  has parking info && has parking point && NOT has parking space id && get
  //  ID successfully
  if (has_parking_info && routing_request.parking_info().has_parking_point()) {
    const PointENU parking_point =
        routing_request.parking_info().parking_point();
    std::string parking_space_id;
    if (GetParkingID(parking_point, &parking_space_id)) {
      routing_response->mutable_routing_request()
          ->mutable_parking_info()
          ->set_parking_space_id(parking_space_id);
      return true;
    }
  }
  ADEBUG << "Failed to fill parking ID";
  return false;
}

bool Routing::Process(const std::shared_ptr<RoutingRequest>& routing_request,
                      RoutingResponse* const routing_response) {
  CHECK_NOTNULL(routing_response);
  AINFO << "Get new routing request:" << routing_request->DebugString();

  /**
   * 补充RoutingRequest信息，实际上是根据RoutingResponse中的waypoint，找到与其对应的lane
   * 做排列组合生成更多的RoutingResponse
   * **/
  const auto& fixed_requests = FillLaneInfoIfMissing(*routing_request);

  double min_routing_length = std::numeric_limits<double>::max();
  for (const auto& fixed_request : fixed_requests) {
    RoutingResponse routing_response_temp;

    /**
     * 找到一条路径
     * **/
    if (navigator_ptr_->SearchRoute(fixed_request, &routing_response_temp)) {
      const double routing_length =
          routing_response_temp.measurement().distance();

      /**
       * 更新最短路径
       * **/    
      if (routing_length < min_routing_length) {
        routing_response->CopyFrom(routing_response_temp);
        min_routing_length = routing_length;
      }
    }
    FillParkingID(routing_response);
  }


  if (min_routing_length < std::numeric_limits<double>::max()) {
    monitor_logger_buffer_.INFO("Routing success!");
    return true;
  }

  AERROR << "Failed to search route with navigator.";
  monitor_logger_buffer_.WARN("Routing failed! " +
                              routing_response->status().msg());
  return false;
}

}  // namespace routing
}  // namespace apollo
