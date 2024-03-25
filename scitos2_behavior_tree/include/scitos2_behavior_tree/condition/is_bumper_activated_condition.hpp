// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#ifndef SCITOS2_BEHAVIOR_TREE__CONDITION__IS_BUMPER_ACTIVATED_CONDITION_HPP_
#define SCITOS2_BEHAVIOR_TREE__CONDITION__IS_BUMPER_ACTIVATED_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "scitos2_msgs/msg/bumper_status.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace scitos2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that listens to a bumper topic and
 * returns SUCCESS when the bumper is activated and FAILURE otherwise
 */
class IsBumperActivatedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for scitos2_behavior_tree::IsBumperActivatedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsBumperActivatedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBumperActivatedCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "bumper_topic", std::string("/bumper"), "Bumper topic")
    };
  }

private:
  /**
   * @brief Callback function for bumper topic
   * @param msg Shared pointer to scitos_msgs::msg::BumperStatus message
   */
  void bumperCallback(scitos2_msgs::msg::BumperStatus::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<scitos2_msgs::msg::BumperStatus>::SharedPtr bumper_sub_;
  std::string bumper_topic_;
  bool is_bumper_activated_;
};

}  // namespace scitos2_behavior_tree

#endif  // SCITOS2_BEHAVIOR_TREE__CONDITION__IS_BUMPER_ACTIVATED_CONDITION_HPP_
