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

#ifndef SCITOS2_MODULES__CHARGER_HPP_
#define SCITOS2_MODULES__CHARGER_HPP_

// MIRA
#include <fw/Framework.h>
#include <robot/BatteryState.h>

// C++
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

// ROS
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

// SCITOS2
#include "scitos2_mira_utils/mira_module.hpp"
#include "scitos2_msgs/msg/charger_status.hpp"
#include "scitos2_msgs/srv/save_persistent_errors.hpp"

namespace scitos2_modules
{

/**
 * @class scitos2_modules::Charger
 * @brief Module for communicating with the charger and battery.
 *
 */
class Charger : public scitos2_mira_utils::MiraModule
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
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) override;

  /**
   * @brief Cleanup the module state machine.
   */
  void cleanup() override;

  /**
   * @brief Activate the module state machine.
   */
  bool activate() override;

  /**
   * @brief Deactivate the module state machine.
   */
  bool deactivate() override;

  /**
   * @brief Add the status of the module to the diagnostics of the node.
   *
   * @param array The diagnostic array to add the status to
   */
  void diagnostics(diagnostic_msgs::msg::DiagnosticArray & array) override;

protected:
  /**
   * @brief Convert the name of a battery chemistry to a BatteryState technology value.
   *
   * @param technology Name of the chemistry (e.g. "LIFE")
   * @return uint8_t BatteryState::POWER_SUPPLY_TECHNOLOGY_* value, UNKNOWN if the name is wrong
   */
  static uint8_t parseTechnology(const std::string & technology);

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

  /**
   * @brief Convert MIRA BatteryState to ROS BatteryState.
   *
   * @param state BatteryState from MIRA
   * @param timestamp Timestamp of the state
   * @return sensor_msgs::msg::BatteryState BatteryState for ROS
   */
  sensor_msgs::msg::BatteryState miraToRosBatteryState(
    const mira::robot::BatteryState & state, const mira::Time & timestamp);

  /**
   * @brief Convert MIRA ChargerStatus to ROS ChargerStatus.
   *
   * @param status ChargerStatus from MIRA
   * @param timestamp Timestamp of the status
   * @return scitos2_msgs::msg::ChargerStatus ChargerStatus for ROS
   */
  scitos2_msgs::msg::ChargerStatus miraToRosChargerStatus(
    const uint8 & status, const mira::Time & timestamp);

  /**
   * @brief Fill the diagnostic status of the battery and the charger.
   *
   * @param stat Status to fill
   */
  void fillDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);


  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>>
  battery_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::ChargerStatus>>
  charger_pub_;

  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::SavePersistentErrors>>
  save_persistent_errors_service_;

  std::string robot_base_frame_{"base_link"};
  // Design capacity of the battery in Ah. Depends on the physical battery model fitted to
  // the robot, there is no way to query it from MIRA.
  double design_capacity_{40.0};
  // Chemistry of the battery as a BatteryState::POWER_SUPPLY_TECHNOLOGY_* value
  uint8_t power_supply_technology_{sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE};

  // Time without data from MIRA after which the diagnostics report an error
  static constexpr std::chrono::seconds kDataTimeout{5};
  // Battery charge below which the diagnostics report a warning, in the range [0, 1]
  static constexpr float kLowBatteryPercentage{0.15f};

  // Guards the last data received, written by the MIRA callbacks and read by the updater
  std::mutex diagnostics_mutex_;
  sensor_msgs::msg::BatteryState last_battery_;
  scitos2_msgs::msg::ChargerStatus last_charger_;
  std::chrono::steady_clock::time_point last_battery_time_;
  std::chrono::steady_clock::time_point last_charger_time_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__CHARGER_HPP_
