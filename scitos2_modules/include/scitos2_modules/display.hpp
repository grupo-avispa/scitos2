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

#ifndef SCITOS2_MODULES__DISPLAY_HPP_
#define SCITOS2_MODULES__DISPLAY_HPP_

// MIRA
#include <fw/Framework.h>

// C++
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// SCITOS2
#include "scitos2_core/module.hpp"
#include "scitos2_msgs/msg/menu_entry.hpp"

namespace scitos2_modules
{

/**
 * @class scitos2_modules::Display
 * @brief Module for interfacing the status on the mini embedded display.
 *
 */
class Display : public scitos2_core::Module
{
public:
  /**
   * @brief Construct for scitos2_modules::Display
   */
  Display() = default;

  /**
   * @brief Destructor for scitos2_modules::Display
   */
  ~Display() override = default;

  /**
   * @brief Configure the module.
   *
   * @param parent WeakPtr to node
   * @param name Name of plugin
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name) override;

  /**
   * @brief Cleanup the module state machine.
   */
  void cleanup() override;

  /**
   * @brief Activate the module state machine.
   */
  void activate() override;

  /**
   * @brief Deactivate the module state machine.
   */
  void deactivate() override;

protected:
  /**
   * @brief Callback executed when a parameter change is detected.
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Callback for menu data.
   *
   * @param data Menu data
   */
  void menuDataCallback(mira::ChannelRead<uint8> data);

  // MIRA Authority
  std::shared_ptr<mira::Authority> authority_;

  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("Display")};

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::MenuEntry>>
  display_data_pub_;

  bool user_menu_enabled_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__DISPLAY_HPP_
