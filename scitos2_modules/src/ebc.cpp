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
#include <string>

#include "scitos2_modules/ebc.hpp"

namespace scitos2_modules
{

using rcl_interfaces::msg::ParameterType;

const std::vector<EbcBoolPort> EBC::kBoolPorts = {
  {"mcu_5v_enabled", "MainControlUnit.EBC_5V.Enabled", "Enable / disable 5V enabled at MCU"},
  {"mcu_12v_enabled", "MainControlUnit.EBC_12V.Enabled", "Enable / disable 12V enabled at MCU"},
  {"mcu_24v_enabled", "MainControlUnit.EBC_24V.Enabled", "Enable / disable 24V enabled at MCU"},
  {"port0_5v_enabled", "EBC7.Port0_5V.Enabled", "Enable / disable 5V enabled at port 0"},
  {"port0_12v_enabled", "EBC7.Port0_12V.Enabled", "Enable / disable 12V enabled at port 0"},
  {"port0_24v_enabled", "EBC7.Port0_24V.Enabled", "Enable / disable 24V enabled at port 0"},
  {"port1_5v_enabled", "EBC7.Port1_5V.Enabled", "Enable / disable 5V enabled at port 1"},
  {"port1_12v_enabled", "EBC7.Port1_12V.Enabled", "Enable / disable 12V enabled at port 1"},
  {"port1_24v_enabled", "EBC7.Port1_24V.Enabled", "Enable / disable 24V enabled at port 1"},
};

const std::vector<EbcCurrentPort> EBC::kCurrentPorts = {
  {"mcu_5v_max_current", "MainControlUnit.EBC_5V.MaxCurrent",
    "Maximum current for MCU 5V in A", 2.5, 2.5},
  {"mcu_12v_max_current", "MainControlUnit.EBC_12V.MaxCurrent",
    "Maximum current for MCU 12V in A", 2.5, 2.5},
  {"mcu_24v_max_current", "MainControlUnit.EBC_24V.MaxCurrent",
    "Maximum current for MCU 24V in A", 2.5, 2.5},
  {"port0_5v_max_current", "EBC7.Port0_5V.MaxCurrent",
    "Maximum current for port 0 5V in A", 2.5, 2.5},
  {"port0_12v_max_current", "EBC7.Port0_12V.MaxCurrent",
    "Maximum current for port 0 12V in A", 2.5, 2.5},
  {"port0_24v_max_current", "EBC7.Port0_24V.MaxCurrent",
    "Maximum current for port 0 24V in A", 2.5, 2.5},
  // Port 1 5V is limited to 2.5A like every other 5V port; a previous version declared it
  // with a 2.5A range but validated it in the dynamic callback up to 4.0A, an unreachable
  // and misleading divergence since rcl already rejects anything above 2.5A before the
  // callback runs
  {"port1_5v_max_current", "EBC7.Port1_5V.MaxCurrent",
    "Maximum current for port 1 5V in A", 2.5, 2.5},
  {"port1_12v_max_current", "EBC7.Port1_12V.MaxCurrent",
    "Maximum current for port 1 12V in A", 2.5, 4.0},
  {"port1_24v_max_current", "EBC7.Port1_24V.MaxCurrent",
    "Maximum current for port 1 24V in A", 4.0, 4.0},
};

void EBC::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
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

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mira_robot_resource",
    rclcpp::ParameterValue(mira_robot_resource_), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The MIRA resource that exposes the robot's services and properties"));
  node->get_parameter(plugin_name_ + ".mira_robot_resource", mira_robot_resource_);

  for (const auto & port : kBoolPorts) {
    declare_parameter_if_not_declared(
      node, plugin_name_ + "." + port.param,
      rclcpp::ParameterValue(true),
      rcl_interfaces::msg::ParameterDescriptor().set__description(port.description));
    bool enabled = true;
    node->get_parameter(plugin_name_ + "." + port.param, enabled);
    port_enabled_[port.param] = enabled;
    RCLCPP_INFO(
      logger_, "The parameter %s is set to: [%s]", port.param.c_str(),
      enabled ? "true" : "false");
  }

  for (const auto & port : kCurrentPorts) {
    declare_parameter_if_not_declared(
      node, plugin_name_ + "." + port.param,
      rclcpp::ParameterValue(port.default_value), rcl_interfaces::msg::ParameterDescriptor()
      .set__description(port.description)
      .set__floating_point_range(
        {rcl_interfaces::msg::FloatingPointRange()
          .set__from_value(0.0)
          .set__to_value(port.max_current)
          .set__step(0.5)}
    ));
    double current = port.default_value;
    node->get_parameter(plugin_name_ + "." + port.param, current);
    port_max_current_[port.param] = current;
    RCLCPP_INFO(logger_, "The parameter %s is set to: [%f]", port.param.c_str(), current);
  }

  // Callback for monitor changes in parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&EBC::dynamicParametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Configured module : %s", plugin_name_.c_str());
}

void EBC::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type scitos2_module::EBC", plugin_name_.c_str());
  authority_.reset();
}

void EBC::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type scitos2_module::EBC", plugin_name_.c_str());

  try {
    authority_->start();
  } catch (const mira::Exception & ex) {
    RCLCPP_ERROR(logger_, "Failed to start scitos2_module::EBC. Exception: %s", ex.what());
    return;
  }

  // MIRA parameters can only be written once the authority has started
  for (const auto & port : kBoolPorts) {
    if (!set_mira_param(
        authority_, port.mira_key, port_enabled_[port.param] ? "true" : "false", logger_))
    {
      RCLCPP_ERROR(logger_, "Failed to set the %s MIRA parameter", port.param.c_str());
    }
  }
  for (const auto & port : kCurrentPorts) {
    if (!set_mira_param(
        authority_, port.mira_key, std::to_string(port_max_current_[port.param]), logger_))
    {
      RCLCPP_ERROR(logger_, "Failed to set the %s MIRA parameter", port.param.c_str());
    }
  }
}

void EBC::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type scitos2_module::EBC", plugin_name_.c_str());
  authority_->checkout();
}

rcl_interfaces::msg::SetParametersResult EBC::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_BOOL) {
      auto it = std::find_if(
        kBoolPorts.begin(), kBoolPorts.end(), [&](const EbcBoolPort & port) {
          return name == plugin_name_ + "." + port.param;
        });
      if (it != kBoolPorts.end()) {
        port_enabled_[it->param] = parameter.as_bool();
        set_mira_param(authority_, it->mira_key, parameter.as_bool() ? "true" : "false", logger_);
        RCLCPP_INFO(
          logger_, "The parameter %s is set to: [%s]", it->param.c_str(),
          parameter.as_bool() ? "true" : "false");
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      auto it = std::find_if(
        kCurrentPorts.begin(), kCurrentPorts.end(), [&](const EbcCurrentPort & port) {
          return name == plugin_name_ + "." + port.param;
        });
      if (it != kCurrentPorts.end()) {
        if (parameter.as_double() < 0.0 || parameter.as_double() > it->max_current) {
          result.successful = false;
          result.reason = it->param + " must be between 0.0 and " +
            std::to_string(it->max_current);
          return result;
        }
        port_max_current_[it->param] = parameter.as_double();
        set_mira_param(
          authority_, it->mira_key, std::to_string(parameter.as_double()), logger_);
        RCLCPP_INFO(
          logger_, "The parameter %s is set to: [%f]", it->param.c_str(), parameter.as_double());
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace scitos2_modules

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(scitos2_modules::EBC, scitos2_core::Module)
