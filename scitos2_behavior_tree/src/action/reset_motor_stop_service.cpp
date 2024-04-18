// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "scitos2_behavior_tree/action/reset_motor_stop_service.hpp"

namespace scitos2_behavior_tree
{

ResetMotorStopService::ResetMotorStopService(
  const std::string & service_node_name, const BT::NodeConfiguration & conf)
: BtServiceNode<scitos2_msgs::srv::ResetMotorStop>(service_node_name, conf)
{
}

}  // namespace scitos2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<scitos2_behavior_tree::ResetMotorStopService>("ResetMotorStop");
}
