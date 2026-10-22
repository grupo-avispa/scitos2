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

// C++
#include <algorithm>
#include <chrono>
#include <limits>

#include "scitos2_modules/charger.hpp"

namespace scitos2_modules
{

using std::placeholders::_1;
using std::placeholders::_2;

void Charger::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
  auto node = configureMira(parent, name);
  int mira_rpc_timeout = 1000;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mira_rpc_timeout",
    rclcpp::ParameterValue(mira_rpc_timeout), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Milliseconds to wait for MIRA to answer a service or property call"));
  node->get_parameter(plugin_name_ + ".mira_rpc_timeout", mira_rpc_timeout);
  authority_->setTimeout(std::chrono::milliseconds(std::max(mira_rpc_timeout, 1)));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_base_frame",
    rclcpp::ParameterValue(robot_base_frame_), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the base frame of the robot"));
  node->get_parameter(plugin_name_ + ".robot_base_frame", robot_base_frame_);

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".design_capacity",
    rclcpp::ParameterValue(design_capacity_), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Design capacity of the battery in Ah, for the fitted battery model"));
  node->get_parameter(plugin_name_ + ".design_capacity", design_capacity_);

  // Create ROS publishers
  battery_pub_ = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery", rclcpp::SystemDefaultsQoS());
  charger_pub_ = node->create_publisher<scitos2_msgs::msg::ChargerStatus>(
    "charger_status", rclcpp::SystemDefaultsQoS());

  // Create MIRA subscribers
  authority_->subscribe<mira::robot::BatteryState>(
    "/robot/charger/Battery", std::bind(&Charger::batteryDataCallback, this, _1));
  authority_->subscribe<uint8>(
    "/robot/charger/ChargerStatus", std::bind(&Charger::chargerStatusCallback, this, _1));

  // Create ROS services
  save_persistent_errors_service_ = node->create_service<scitos2_msgs::srv::SavePersistentErrors>(
    "charger/save_persistent_errors", std::bind(&Charger::savePersistentErrors, this, _1, _2));

  RCLCPP_INFO(logger_, "Configured module : %s", plugin_name_.c_str());
}

void Charger::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type scitos2_module::Charger", plugin_name_.c_str());
  cleanupMira();
  battery_pub_.reset();
  charger_pub_.reset();
  save_persistent_errors_service_.reset();
}

bool Charger::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type scitos2_module::Charger", plugin_name_.c_str());
  battery_pub_->on_activate();
  charger_pub_->on_activate();

  if (!startMira()) {
    return false;
  }
  return true;
}

bool Charger::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type scitos2_module::Charger", plugin_name_.c_str());
  battery_pub_->on_deactivate();
  charger_pub_->on_deactivate();
  return true;
}

void Charger::batteryDataCallback(mira::ChannelRead<mira::robot::BatteryState> data)
{
  sensor_msgs::msg::BatteryState battery = miraToRosBatteryState(*data, data->timestamp);
  battery_pub_->publish(battery);

  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  last_battery_ = battery;
  last_battery_time_ = std::chrono::steady_clock::now();
}

void Charger::chargerStatusCallback(mira::ChannelRead<uint8> data)
{
  scitos2_msgs::msg::ChargerStatus charger = miraToRosChargerStatus(*data, data->timestamp);
  charger_pub_->publish(charger);

  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  last_charger_ = charger;
  last_charger_time_ = std::chrono::steady_clock::now();
}

void Charger::diagnostics(diagnostic_msgs::msg::DiagnosticArray & array)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;
  stat.name = "Charger";
  stat.hardware_id = "scitos2";
  fillDiagnostics(stat);
  array.status.push_back(stat);
}

void Charger::fillDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  sensor_msgs::msg::BatteryState battery;
  scitos2_msgs::msg::ChargerStatus charger;
  std::chrono::steady_clock::time_point battery_time, charger_time;
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    battery = last_battery_;
    charger = last_charger_;
    battery_time = last_battery_time_;
    charger_time = last_charger_time_;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool has_battery = battery_time != std::chrono::steady_clock::time_point{} &&
  now - battery_time < kDataTimeout;
  const bool has_charger = charger_time != std::chrono::steady_clock::time_point{} &&
  now - charger_time < kDataTimeout;

  if (!has_battery) {
    stat.summary(DiagnosticStatus::ERROR, "No battery data received from MIRA");
  } else if (has_charger && charger.internal_error_flag) {
    stat.summary(DiagnosticStatus::ERROR, "Charger internal error");
  } else if (has_charger && charger.empty) {
    stat.summary(DiagnosticStatus::ERROR, "Battery empty");
  } else if (battery.percentage < kLowBatteryPercentage) {
    stat.summary(DiagnosticStatus::WARN, "Battery low");
  } else {
    stat.summary(DiagnosticStatus::OK, "Battery OK");
  }

  if (has_battery) {
    stat.add("Voltage (V)", battery.voltage);
    stat.add("Current (A)", battery.current);
    stat.add("Percentage", battery.percentage);
    stat.add("Charging", battery.power_supply_status ==
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
  }
  if (has_charger) {
    stat.add("Charger full", charger.full);
    stat.add("Charger derating", charger.derating);
    stat.add("Charger internal error", charger.internal_error_flag);
  }
}

bool Charger::savePersistentErrors(
  const std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Request> request,
  std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Response> response)
{
  RCLCPP_INFO_STREAM(logger_, "Saving persistent error log to '" << request->filename << "'");

  // Call mira service
  response->success = authority_->callService(
    "savePersistentErrors", std::optional<std::string>(request->filename));
  if (!response->success) {
    response->message = "MIRA service call failed";
  }
  return response->success;
}

sensor_msgs::msg::BatteryState Charger::miraToRosBatteryState(
  const mira::robot::BatteryState & state, const mira::Time & timestamp)
{
  sensor_msgs::msg::BatteryState battery;

  battery.header.frame_id = robot_base_frame_;
  battery.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  battery.voltage = state.voltage;
  battery.temperature = std::numeric_limits<float>::quiet_NaN();
  // ROS convention: negative current means discharging. battery.charge is a magnitude
  // (remaining Ah estimated from time-to-empty), not a signed flow, so it intentionally
  // uses the raw MIRA current rather than the ROS-signed one.
  battery.current = -state.current;
  battery.charge = (state.lifeTime == -1) ? std::numeric_limits<float>::quiet_NaN() :
    (static_cast<float>(state.lifeTime) / 60.0 * state.current);
  battery.capacity = std::numeric_limits<float>::quiet_NaN();
  battery.design_capacity = design_capacity_;
  battery.percentage = (state.lifePercent == 255) ? std::numeric_limits<float>::quiet_NaN() :
    static_cast<float>(state.lifePercent) / 100.0;

  if (state.charging) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (state.lifePercent == 100) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  } else if (state.powerSupplyPresent) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else if (battery.current < 0) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  } else {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }

  battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
  battery.present = (state.cellVoltage).empty() ? false : true;
  battery.cell_voltage = state.cellVoltage;
  battery.cell_temperature.resize(
    battery.cell_voltage.size(), std::numeric_limits<float>::quiet_NaN());
  battery.location = "Slot 1";
  battery.serial_number = "Unknown";

  return battery;
}

scitos2_msgs::msg::ChargerStatus Charger::miraToRosChargerStatus(
  const uint8 & status,
  const mira::Time & timestamp)
{
  scitos2_msgs::msg::ChargerStatus charger;

  charger.header.frame_id = robot_base_frame_;
  charger.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  charger.charging = static_cast<bool>(status & 1);
  charger.empty = static_cast<bool>(status & (1 << 1));
  charger.full = static_cast<bool>(status & (1 << 2));
  charger.derating = static_cast<bool>(status & (1 << 3));
  charger.charging_disabled = static_cast<bool>(status & (1 << 4));
  charger.const_volt_mode = static_cast<bool>(status & (1 << 5));
  charger.const_current_mode = static_cast<bool>(status & (1 << 6));
  charger.internal_error_flag = static_cast<bool>(status & (1 << 7));

  return charger;
}

}  // namespace scitos2_modules

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(scitos2_modules::Charger, scitos2_core::Module)
