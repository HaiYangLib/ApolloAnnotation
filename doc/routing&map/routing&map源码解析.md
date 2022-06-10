# Routing&Map

## Routing模块初始化

Routing模块通过`Init`方法来初始化。在初始化时，会创建Navigator对象以及加载地图，相关代码如下：

```c++
bool RoutingComponent::Init() {
  // routing_response_topic: "/apollo/routing_response"    
  response_writer_ = node_->CreateWriter<RoutingResponse>(attr);
  // routing_response_history_topic: "/apollo/routing_response_history"
  response_history_writer_ = node_->CreateWriter<RoutingResponse>(attr_history);
  ...........
  return routing_.Init().ok() && routing_.Start().ok();
}
```

`Routing routing_`是`RoutingComponent`最重要的成员变量,其初始化过程:

```c++
apollo::common::Status Routing::Init() {
  /**
   * 默认情况下：
   * routing_map_file=modules/map/data/demo/routing_map.txt
   * **/
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  navigator_ptr_.reset(new Navigator(routing_map_file));
  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  return apollo::common::Status::OK();
}
```

之后会执行`Process`主流程:

```c++
bool Process(const std::shared_ptr<RoutingRequest> &routing_request,
               RoutingResponse *const routing_response);
```