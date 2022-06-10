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

#include "cyber/common/file.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/topo_creator/graph_creator.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::routing::RoutingConfig routing_conf;

  /**
   *
   * DEFINE_string(routing_conf_file,
   *          "/apollo/modules/routing/conf/routing_config.pb.txt",
   *          "default routing conf data file");

  * base_speed: 4.167
  * left_turn_penalty: 50.0
  * right_turn_penalty: 20.0
  * uturn_penalty: 100.0
  * change_penalty: 500.0
  * base_changing_length: 50.0
  * topic_config {
  *   routing_response_topic: "/apollo/routing_response"
  *   routing_response_history_topic: "/apollo/routing_response_history"
  * }
   * **/
  ACHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_routing_conf_file,
                                                 &routing_conf))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

  const auto base_map = apollo::hdmap::BaseMapFile();
  const auto routing_map = apollo::hdmap::RoutingMapFile();

  apollo::routing::GraphCreator creator(base_map, routing_map, routing_conf);
  ACHECK(creator.Create()) << "Create routing topo failed!";

  AINFO << "Create routing topo successfully from " << base_map << " to "
        << routing_map;
  return 0;
}
