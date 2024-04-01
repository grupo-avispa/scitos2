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

#ifndef SCITOS2_BEHAVIOR_TREE__ACTION__RESET_MOTOR_STOP_SERVICE_HPP_
#define SCITOS2_BEHAVIOR_TREE__ACTION__RESET_MOTOR_STOP_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "scitos2_msgs/srv/reset_motor_stop.hpp"

namespace scitos2_behavior_tree
{

using nav2_behavior_tree::BtServiceNode;

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps scitos_msgs::srv::ResetMotorStop
 */
class ResetMotorStopService : public BtServiceNode<scitos2_msgs::srv::ResetMotorStop>
{
public:
  /**
   * @brief A constructor for scitos2_behavior_tree::ResetMotorStopService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ResetMotorStopService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;
};

}  // namespace scitos2_behavior_tree

#endif  // SCITOS2_BEHAVIOR_TREE__ACTION__RESET_MOTOR_STOP_SERVICE_HPP_
