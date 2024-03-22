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

#ifndef SCITOS2_MODULES__CHARGER_HPP_
#define SCITOS2_MODULES__CHARGER_HPP_

// MIRA
#include <fw/Framework.h>
#include <robot/BatteryState.h>

// C++
#include <memory>
#include <mutex>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

// SCITOS2
#include "scitos2_core/module.hpp"
#include "scitos2_msgs/msg/charger_status.hpp"
#include "scitos2_msgs/srv/save_persistent_errors.hpp"

namespace scitos2_modules
{

/**
 * @class scitos2_modules::Charger
 * @brief Module for communicating with the charger and battery.
 *
 */
class Charger : public scitos2_core::Module
{
public:
  /**
   * @brief Construct for scitos2_modules::Charger
   */
  Charger() = default;

  /**
   * @brief Destructor for scitos2_modules::Charger
   */
  ~Charger() override = default;

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
   * @brief Callback for battery data.
   *
   * @param data Battery data
   */
  void batteryDataCallback(mira::ChannelRead<mira::robot::BatteryState> data);

  /**
   * @brief Callback for charger status.
   *
   * @param data Charger status
   */
  void chargerStatusCallback(mira::ChannelRead<uint8> data);

  /**
   * @brief Service callback for saving persistent errors.
   *
   * @param request Request for saving persistent errors
   * @param response Response for saving persistent errors
   * @return bool If the persistent errors were saved
   */
  bool savePersistentErrors(
    const std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Request> request,
    std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Response> response);

  // MIRA Authority
  std::shared_ptr<mira::Authority> authority_;

  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("Charger")};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>>
  battery_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::ChargerStatus>>
  charger_pub_;

  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::SavePersistentErrors>>
  save_persistent_errors_service_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__CHARGER_HPP_
