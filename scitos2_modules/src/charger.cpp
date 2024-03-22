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

// C++
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "scitos2_modules/charger.hpp"

namespace scitos2_modules
{

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
  authority_ = std::make_shared<mira::Authority>("/", plugin_name_);

  // Create ROS publishers
  battery_pub_ = node->create_publisher<sensor_msgs::msg::BatteryState>("battery", 1);
  charger_pub_ = node->create_publisher<scitos2_msgs::msg::ChargerStatus>(
    "charger_status", 1);

  // Create MIRA subscribers
  authority_->subscribe<mira::robot::BatteryState>(
    "/robot/charger/Battery", &Charger::batteryDataCallback, this);
  authority_->subscribe<uint8>(
    "/robot/charger/ChargerStatus", &Charger::chargerStatusCallback, this);

  // Create ROS services
  save_persistent_errors_service_ = node->create_service<scitos2_msgs::srv::SavePersistentErrors>(
    "charger/save_persistent_errors",
    std::bind(
      &Charger::savePersistentErrors, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(logger_, "Configured module : %s", plugin_name_.c_str());
}

void Charger::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up module : %s of type scitos2_module::Charger",
    plugin_name_.c_str());
  authority_->checkout();
  authority_.reset();
  battery_pub_.reset();
  charger_pub_.reset();
}

void Charger::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating module : %s of type scitos2_module::Charger",
    plugin_name_.c_str());
  battery_pub_->on_activate();
  charger_pub_->on_activate();
}

void Charger::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating module : %s of type scitos2_module::Charger",
    plugin_name_.c_str());
  battery_pub_->on_deactivate();
  charger_pub_->on_deactivate();
}

void Charger::batteryDataCallback(mira::ChannelRead<mira::robot::BatteryState> data)
{
  // Get time data from mira
  rclcpp::Time time = rclcpp::Time(data->timestamp.toUnixNS());

  sensor_msgs::msg::BatteryState battery;

  battery.header.stamp = time;
  battery.header.frame_id = "base_link";
  battery.voltage = data->voltage;
  battery.temperature = std::numeric_limits<float>::quiet_NaN();
  battery.current = -data->current;
  battery.charge = (data->lifeTime == -1) ? std::numeric_limits<float>::quiet_NaN() :
    (static_cast<float>(data->lifeTime) / 60.0 * battery.current);
  battery.capacity = std::numeric_limits<float>::quiet_NaN();
  battery.design_capacity = 40.0;
  battery.percentage = (data->lifePercent == 255) ? std::numeric_limits<float>::quiet_NaN() :
    static_cast<float>(data->lifePercent) / 100.0;

  if (data->charging) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (battery.percentage == 1.0) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  } else if (data->powerSupplyPresent) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else if (battery.current < 0) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  } else {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }

  battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
  battery.present = (data->cellVoltage).empty() ? false : true;
  battery.cell_voltage = data->cellVoltage;
  battery.cell_temperature.resize(
    battery.cell_voltage.size(),
    std::numeric_limits<float>::quiet_NaN());
  battery.location = "Slot 1";
  battery.serial_number = "Unknown";

  battery_pub_->publish(battery);
}

void Charger::chargerStatusCallback(mira::ChannelRead<uint8> data)
{
  // Get time data from mira
  rclcpp::Time time = rclcpp::Time(data->timestamp.toUnixNS());

  scitos2_msgs::msg::ChargerStatus charger;

  charger.header.stamp = time;
  charger.header.frame_id = "base_link";
  charger.charging = (*data) & 1;
  charger.empty = (*data) & (1 << 1);
  charger.full = (*data) & (1 << 2);
  charger.derating = (*data) & (1 << 3);
  charger.charging_disabled = (*data) & (1 << 4);
  charger.const_volt_mode = (*data) & (1 << 5);
  charger.const_current_mode = (*data) & (1 << 6);
  charger.internal_error_flag = (*data) & (1 << 7);

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

}  // namespace scitos2_modules

PLUGINLIB_EXPORT_CLASS(scitos2_modules::Charger, scitos2_core::Module)
