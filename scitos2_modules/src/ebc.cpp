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

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "scitos2_modules/ebc.hpp"

namespace scitos2_modules
{

using rcl_interfaces::msg::ParameterType;

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

  bool port_enabled;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mcu_5v_enabled",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 5V enabled at MCU"));
  node->get_parameter(plugin_name_ + ".mcu_5v_enabled", port_enabled);
  set_mira_param(authority_, "MainControlUnit.EBC_5V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter mcu_5v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mcu_12v_enabled",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 12V enabled at MCU"));
  node->get_parameter(plugin_name_ + ".mcu_12v_enabled", port_enabled);
  set_mira_param(authority_, "MainControlUnit.EBC_12V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter mcu_12v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mcu_24v_enabled",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 24V enabled at MCU"));
  node->get_parameter(plugin_name_ + ".mcu_24v_enabled", port_enabled);
  set_mira_param(authority_, "MainControlUnit.EBC_24V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter mcu_24v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port0_5v_enabled",
    rclcpp::ParameterValue(true),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 5V enabled at port 0"));
  node->get_parameter(plugin_name_ + ".port0_5v_enabled", port_enabled);
  set_mira_param(authority_, "EBC7.Port0_5V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter port0_5v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port0_12v_enabled",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 12V enabled at port 0"));
  node->get_parameter(plugin_name_ + ".port0_12v_enabled", port_enabled);
  set_mira_param(authority_, "EBC7.Port0_12V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter port0_12v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port0_24v_enabled",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 24V enabled at port 0"));
  node->get_parameter(plugin_name_ + ".port0_24v_enabled", port_enabled);
  set_mira_param(authority_, "EBC7.Port0_24V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter port0_24v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port1_5v_enabled",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 5V enabled at port 1"));
  node->get_parameter(plugin_name_ + ".port1_5v_enabled", port_enabled);
  set_mira_param(authority_, "EBC7.Port1_5V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter port1_5v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port1_12v_enabled",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 12V enabled at port 1"));
  node->get_parameter(plugin_name_ + ".port1_12v_enabled", port_enabled);
  set_mira_param(authority_, "EBC7.Port1_12V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter port1_12v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port1_24v_enabled",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable 24V enabled at port 1"));
  node->get_parameter(plugin_name_ + ".port1_24v_enabled", port_enabled);
  set_mira_param(authority_, "EBC7.Port1_24V.Enabled", port_enabled ? "true" : "false");
  RCLCPP_INFO(
    logger_, "The parameter port1_24v_enabled is set to: [%s]",
    port_enabled ? "true" : "false");

  float port_max_current;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mcu_5v_max_current",
    rclcpp::ParameterValue(2.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for MCU 5V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.5)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".mcu_5v_max_current", port_max_current);
  set_mira_param(authority_, "MainControlUnit.EBC_5V.MaxCurrent", std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter mcu_5v_max_current is set to: [%f]", port_max_current);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mcu_12v_max_current",
    rclcpp::ParameterValue(2.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for MCU 12V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.5)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".mcu_12v_max_current", port_max_current);
  set_mira_param(
    authority_, "MainControlUnit.EBC_12V.MaxCurrent",
    std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter mcu_12v_max_current is set to: [%f]", port_max_current);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mcu_24v_max_current",
    rclcpp::ParameterValue(2.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for MCU 24V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.5)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".mcu_24v_max_current", port_max_current);
  set_mira_param(
    authority_, "MainControlUnit.EBC_24V.MaxCurrent",
    std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter mcu_24v_max_current is set to: [%f]", port_max_current);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port0_5v_max_current",
    rclcpp::ParameterValue(2.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for port 0 5V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.5)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".port0_5v_max_current", port_max_current);
  set_mira_param(authority_, "EBC7.Port0_5V.MaxCurrent", std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter port0_5v_max_current is set to: [%f]", port_max_current);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port0_12v_max_current",
    rclcpp::ParameterValue(2.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for port 0 12V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.5)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".port0_12v_max_current", port_max_current);
  set_mira_param(authority_, "EBC7.Port0_12V.MaxCurrent", std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter port0_12v_max_current is set to: [%f]", port_max_current);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port0_24v_max_current",
    rclcpp::ParameterValue(2.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for port 0 24V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.5)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".port0_24v_max_current", port_max_current);
  set_mira_param(authority_, "EBC7.Port0_24V.MaxCurrent", std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter port0_24v_max_current is set to: [%f]", port_max_current);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port1_5v_max_current",
    rclcpp::ParameterValue(2.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for port 1 5V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(2.5)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".port1_5v_max_current", port_max_current);
  set_mira_param(authority_, "EBC7.Port1_5V.MaxCurrent", std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter port1_5v_max_current is set to: [%f]", port_max_current);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port1_12v_max_current",
    rclcpp::ParameterValue(2.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for port 1 12V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(4.0)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".port1_12v_max_current", port_max_current);
  set_mira_param(authority_, "EBC7.Port1_12V.MaxCurrent", std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter port1_12v_max_current is set to: [%f]", port_max_current);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port1_24v_max_current",
    rclcpp::ParameterValue(4.0), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Maximum current for port 1 24V in A")
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(0.0)
        .set__to_value(4.0)
        .set__step(0.5)}
  ));
  node->get_parameter(plugin_name_ + ".port1_24v_max_current", port_max_current);
  set_mira_param(authority_, "EBC7.Port1_24V.MaxCurrent", std::to_string(port_max_current));
  RCLCPP_INFO(logger_, "The parameter port1_24v_max_current is set to: [%f]", port_max_current);

  // Callback for monitor changes in parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&EBC::dynamicParametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Configured module : %s", plugin_name_.c_str());
}

void EBC::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up module : %s of type scitos2_module::EBC",
    plugin_name_.c_str());
  authority_.reset();
}

void EBC::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating module : %s of type scitos2_module::EBC",
    plugin_name_.c_str());
  authority_->start();
}

void EBC::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating module : %s of type scitos2_module::EBC",
    plugin_name_.c_str());
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
      if (name == plugin_name_ + ".mcu_5v_enabled") {
        set_mira_param(
          authority_, "MainControlUnit.EBC_5V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter mcu_5v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      } else if (name == plugin_name_ + ".mcu_12v_enabled") {
        set_mira_param(
          authority_, "MainControlUnit.EBC_12V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter mcu_12v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      } else if (name == plugin_name_ + ".mcu_24v_enabled") {
        set_mira_param(
          authority_, "MainControlUnit.EBC_24V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter mcu_24v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      } else if (name == plugin_name_ + ".port0_5v_enabled") {
        set_mira_param(
          authority_, "EBC7.Port0_5V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter port0_5v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      } else if (name == plugin_name_ + ".port0_12v_enabled") {
        set_mira_param(
          authority_, "EBC7.Port0_12V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter port0_12v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      } else if (name == plugin_name_ + ".port0_24v_enabled") {
        set_mira_param(
          authority_, "EBC7.Port0_24V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter port0_24v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      } else if (name == plugin_name_ + ".port1_5v_enabled") {
        set_mira_param(
          authority_, "EBC7.Port1_5V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter port1_5v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      } else if (name == plugin_name_ + ".port1_12v_enabled") {
        set_mira_param(
          authority_, "EBC7.Port1_12V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter port1_12v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      } else if (name == plugin_name_ + ".port1_24v_enabled") {
        set_mira_param(
          authority_, "EBC7.Port1_24V.Enabled", parameter.as_bool() ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter port1_24v_enabled is set to: [%s]",
          parameter.as_bool() ? "true" : "false");
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".mcu_5v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 2.5) {
          set_mira_param(
            authority_, "MainControlUnit.EBC_5V.MaxCurrent", std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter mcu_5v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter mcu_5v_max_current is out of range");
        }
      } else if (name == plugin_name_ + ".mcu_12v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 2.5) {
          set_mira_param(
            authority_, "MainControlUnit.EBC_12V.MaxCurrent",
            std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter mcu_12v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter mcu_12v_max_current is out of range");
        }
      } else if (name == plugin_name_ + ".mcu_24v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 2.5) {
          set_mira_param(
            authority_, "MainControlUnit.EBC_24V.MaxCurrent",
            std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter mcu_24v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter mcu_24v_max_current is out of range");
        }
      } else if (name == plugin_name_ + ".port0_5v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 2.5) {
          set_mira_param(
            authority_, "EBC7.Port0_5V.MaxCurrent", std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter port0_5v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter port0_5v_max_current is out of range");
        }
      } else if (name == plugin_name_ + ".port0_12v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 2.5) {
          set_mira_param(
            authority_, "EBC7.Port0_12V.MaxCurrent", std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter port0_12v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter port0_12v_max_current is out of range");
        }
      } else if (name == plugin_name_ + ".port0_24v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 2.5) {
          set_mira_param(
            authority_, "EBC7.Port0_24V.MaxCurrent", std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter port0_24v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter port0_24v_max_current is out of range");
        }
      } else if (name == plugin_name_ + ".port1_5v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 4.0) {
          set_mira_param(
            authority_, "EBC7.Port1_5V.MaxCurrent", std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter port1_5v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter port1_5v_max_current is out of range");
        }
      } else if (name == plugin_name_ + ".port1_12v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 4.0) {
          set_mira_param(
            authority_, "EBC7.Port1_12V.MaxCurrent", std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter port1_12v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter port1_12v_max_current is out of range");
        }
      } else if (name == plugin_name_ + ".port1_24v_max_current") {
        if (parameter.as_double() >= 0.0 && parameter.as_double() <= 4.0) {
          set_mira_param(
            authority_, "EBC7.Port1_24V.MaxCurrent", std::to_string(parameter.as_double()));
          RCLCPP_INFO(
            logger_, "The parameter port1_24v_max_current is set to: [%f]",
            parameter.as_double());
        } else {
          RCLCPP_WARN(logger_, "The parameter port1_24v_max_current is out of range");
        }
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace scitos2_modules

PLUGINLIB_EXPORT_CLASS(scitos2_modules::EBC, scitos2_core::Module)
