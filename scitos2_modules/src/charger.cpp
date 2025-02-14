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
#include <limits>

#include "scitos2_modules/charger.hpp"

namespace scitos2_modules
{

using std::placeholders::_1;
using std::placeholders::_2;

void Charger::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
  // Declare and read parameters
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  logger_ = node->get_logger();
  authority_ = std::make_shared<mira::Authority>();
  authority_->checkin("/", plugin_name_);

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
  authority_.reset();
  battery_pub_.reset();
  charger_pub_.reset();
  save_persistent_errors_service_.reset();
}

void Charger::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type scitos2_module::Charger", plugin_name_.c_str());
  battery_pub_->on_activate();
  charger_pub_->on_activate();

  try {
    authority_->start();
  } catch (const mira::Exception & ex) {
    RCLCPP_ERROR(logger_, "Failed to start scitos2_module::Charger. Exception: %s", ex.what());
    return;
  }
}

void Charger::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type scitos2_module::Charger", plugin_name_.c_str());
  authority_->checkout();
  battery_pub_->on_deactivate();
  charger_pub_->on_deactivate();
}

void Charger::batteryDataCallback(mira::ChannelRead<mira::robot::BatteryState> data)
{
  sensor_msgs::msg::BatteryState battery = miraToRosBatteryState(*data, data->timestamp);
  battery_pub_->publish(battery);
}

void Charger::chargerStatusCallback(mira::ChannelRead<uint8> data)
{
  scitos2_msgs::msg::ChargerStatus charger = miraToRosChargerStatus(*data, data->timestamp);
  charger_pub_->publish(charger);
}

bool Charger::savePersistentErrors(
  const std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Request> request,
  std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Response> response)
{
  RCLCPP_INFO_STREAM(logger_, "Saving persistent error log to '" << request->filename << "'");

  // Call mira service
  return call_mira_service(
    authority_, "savePersistentErrors", std::optional<std::string>(request->filename));
}

sensor_msgs::msg::BatteryState Charger::miraToRosBatteryState(
  const mira::robot::BatteryState & state, const mira::Time & timestamp)
{
  sensor_msgs::msg::BatteryState battery;

  battery.header.frame_id = "base_link";
  battery.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  battery.voltage = state.voltage;
  battery.temperature = std::numeric_limits<float>::quiet_NaN();
  battery.current = -state.current;
  battery.charge = (state.lifeTime == -1) ? std::numeric_limits<float>::quiet_NaN() :
    (static_cast<float>(state.lifeTime) / 60.0 * state.current);
  battery.capacity = std::numeric_limits<float>::quiet_NaN();
  battery.design_capacity = 40.0;
  battery.percentage = (state.lifePercent == 255) ? std::numeric_limits<float>::quiet_NaN() :
    static_cast<float>(state.lifePercent) / 100.0;

  if (state.charging) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (battery.percentage == 1.0) {
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

  charger.header.frame_id = "base_link";
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
