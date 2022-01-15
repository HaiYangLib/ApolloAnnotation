# Planning概要说明

在Apollo的平台上，规划分为三种模式：

**OnLanePlanning（车道规划，可用于城区及高速公路各种复杂道路）**

**NaviPlanning（导航规划，主要用于高速公路）**

**OpenSpacePlanning （自主泊车和狭窄路段的掉头，Apollo6.0没有）**

包含四种具体规划算法：

**PublicRoadPlanner（默认规划器）**

**LatticePlanner、NaviPlanner（主要用于高速公路场景）**

**RTKPlanner（循迹算法，一般不用）**

