# Planning源码解析

## 类图

```mermaid
classDiagram


class PlanningComponent{
	traffic_light_reader_
	routing_reader_
	pad_msg_reader_
	relative_map_reader_
	story_telling_reader_
	planning_writer_
	rerouting_writer_
	planning_learning_data_writer_
	
	traffic_light_
	routing_
	pad_msg_
	relative_map_
	stories_
	local_view_
	planning_base_
	injector_
	message_process_
	
	+Proc(prediction_obstacles,chassis,localization_estimate)
}


PlanningComponent*--TrafficLightDetection
class TrafficLightDetection{
	
}

PlanningComponent*--DependencyInjector
class DependencyInjector{
	
}


PlanningComponent*--PlanningBase
class PlanningBase{
	<<interface>>
	local_view_
	hdmap_
	frame_
	planner_
	last_publishable_trajectory_
	planner_dispatcher_
	injector_
}

PlanningComponent*--MessageProcess
class MessageProcess{
	
}
```

```mermaid
classDiagram

class PlanningBase{
	<<interface>>
	local_view_
	hdmap_
	frame_
	planner_
	last_publishable_trajectory_
	planner_dispatcher_
	injector_
	-Init(config)
	+RunOnce(local_view,ptr_trajectory_pb)
}

PlanningBase*--PlannerDispatcher
class PlannerDispatcher{
	<<interface>>
}



PlanningBase*--PublishableTrajectory
class PublishableTrajectory{
	
}

PlanningBase*--DependencyInjector
class DependencyInjector{
	planning_context_
	frame_history_
	history_
	ego_info_
	vehicle_state_
	learning_based_data_
}

PlanningBase*--Planner
class Planner{
	<<interface>>
	scenario_manager_
	scenario_
	+Plan(planning_init_point,frame,ptr_computed_trajectory) 
}

PlanningBase..TaskFactory
class TaskFactory{
	
}
```







```mermaid
classDiagram

class PlanningBase{
	<<interface>>
	local_view_
	hdmap_
	frame_
	planner_
	last_publishable_trajectory_
	planner_dispatcher_
	injector_
}

PlanningBase<--OnLanePlanning
class OnLanePlanning{
	last_routing_
	reference_line_provider_
	planning_smoother_
	+RunOnce(local_view,ptr_trajectory_pb)
}

PlanningBase*--Planner
class Planner{
	<<interface>>
	scenario_manager_
	scenario_
	+Plan(planning_init_point,frame,ptr_computed_trajectory) 
}


PlanningBase<--NaviPlanning
class NaviPlanning{
	last_vehicle_config_
	target_lane_id_
	reference_line_provider_
	+RunOnce(local_view,trajectory_pb)
}

```



```mermaid
classDiagram

class ReferenceLineProvider{
	smoother_
	smoother_config_
  pnc_map_
  relative_map_
  vehicle_state_
  routing_
  reference_lines_
  route_segments_
  last_calculation_time_
  reference_line_history_
  route_segments_history_
  task_future_
  vehicle_state_provider_
  
  +UpdateRoutingResponse(routing)
  +UpdateVehicleState(vehicle_state)
  +Start()
  +Stop()
  +GetReferenceLines(reference_lines,segments)
  +UpdatedReferenceLine()
}



```





```mermaid
classDiagram

class Planner{
	<<interface>>
	scenario_manager_
	scenario_
	+Plan(planning_init_point,frame,ptr_computed_trajectory) 
}

Planner<--PlannerWithReferenceLine
class PlannerWithReferenceLine{
	<<interface>>
	+Plan(planning_init_point,frame,ptr_computed_trajectory) 
}

PlannerWithReferenceLine<--PublicRoadPlanner
class PublicRoadPlanner{
	+Plan(planning_init_point,frame,ptr_computed_trajectory) 
}

PlannerWithReferenceLine<--NaviPlanner
class NaviPlanner{
	+Plan(planning_init_point,frame,ptr_computed_trajectory) 
}

PlannerWithReferenceLine<--RTKReplayPlanner
class RTKReplayPlanner{
	+Plan(planning_init_point,frame,ptr_computed_trajectory) 
}

PlannerWithReferenceLine<--LatticePlanner
class LatticePlanner{
	+Plan(planning_init_point,frame,ptr_computed_trajectory) 
}

```









```mermaid
classDiagram

class DependencyInjector{
	planning_context_
	frame_history_
	history_
	ego_info_
	vehicle_state_
	learning_based_data_
}

DependencyInjector*--PlanningContext
class PlanningContext{
	
}

DependencyInjector*--EgoInfo
class EgoInfo{
	
}

DependencyInjector*--FrameHistory
class FrameHistory{
	
}

DependencyInjector*--History
class History{
	
}

DependencyInjector*--VehicleStateProvider
class VehicleStateProvider{
	
}

DependencyInjector*--LearningBasedData
class LearningBasedData{
	
}

 
```





```mermaid
classDiagram

class TaskFactory{
	task_factory_
	default_task_configs_
}
```

```c++
std::vector<std::shared_ptr<Curve1d>> Trajectory1DBundle 
```



```mermaid
classDiagram

class Curve1d{
	<<interface>>
  +Evaluate(order,param)
  +ParamLength()
  +ToString()
}

Curve1d<--LatticeTrajectory1d
class LatticeTrajectory1d{
	ptr_trajectory1d_
	+Evaluate(order,param)
  +ParamLength()
  +ToString()
}


Curve1d<--PolynomialCurve1d
class PolynomialCurve1d{
	
}

LatticeTrajectory1d*--QuarticPolynomialCurve1d
PolynomialCurve1d<--QuarticPolynomialCurve1d
class QuarticPolynomialCurve1d{
	
}
```

```c++
class Frame {
  static DrivingAction pad_msg_driving_action_;
  uint32_t sequence_num_ = 0;
  LocalView local_view_;
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;

  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;

  std::unordered_map<std::string, const perception::TrafficLight *>
      traffic_lights_;

  // current frame published trajectory
  ADCTrajectory current_frame_planned_trajectory_;

  // current frame path for future possible speed fallback
  DiscretizedPath current_frame_planned_path_;

  const ReferenceLineProvider *reference_line_provider_ = nullptr;

  OpenSpaceInfo open_space_info_;

  std::vector<routing::LaneWaypoint> future_route_waypoints_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};
```



