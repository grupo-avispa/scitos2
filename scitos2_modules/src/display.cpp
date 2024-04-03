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

#include "scitos2_modules/display.hpp"

namespace scitos2_modules
{

using rcl_interfaces::msg::ParameterType;

void Display::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
  // Declare and read parameters
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  authority_ = std::make_shared<mira::Authority>("/", name);

  // Create publisher
  display_data_pub_ = node->create_publisher<scitos2_msgs::msg::MenuEntry>("user_menu_selected", 1);

  // Callback for monitor changes in parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Display::dynamicParametersCallback, this, std::placeholders::_1));

  // Create MIRA subscriber
  authority_->subscribe<uint8>(
    "/robot/StatusDisplayUserMenuEvent", &Display::menuDataCallback, this);

  // Declare and read parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".user_menu_enabled",
    rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable / disable the user menu entry"));
  node->get_parameter(plugin_name_ + ".user_menu_enabled", user_menu_enabled_);
  RCLCPP_INFO(
    logger_, "The parameter user_menu_enabled is set to: [%s]",
    user_menu_enabled_ ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".menu_name",
    rclcpp::ParameterValue("User Menu"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the user menu entry in the main menu of the status display"));
  node->get_parameter(plugin_name_ + ".menu_name", menu_name_);
  RCLCPP_INFO(logger_, "The parameter menu_name is set to: [%s]", menu_name_);

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".menu_entry_name_1",
    rclcpp::ParameterValue("Entry 1"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "The name of the first sub menu entry in the user menu of the status "));
  node->get_parameter(plugin_name_ + ".menu_entry_name_1", menu_entry_name_1_);
  RCLCPP_INFO(
    logger_, "The parameter menu_entry_name_1 is set to: [%s]", menu_entry_name_1_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".menu_entry_name_2",
    rclcpp::ParameterValue("Entry 2"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "The name of the second sub menu entry in the user menu of the status "));
  node->get_parameter(plugin_name_ + ".menu_entry_name_2", menu_entry_name_2_);
  RCLCPP_INFO(
    logger_, "The parameter menu_entry_name_2 is set to: [%s]", menu_entry_name_2_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".menu_entry_name_3",
    rclcpp::ParameterValue("Entry 3"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "The name of the third sub menu entry in the user menu of the status "));
  node->get_parameter(plugin_name_ + ".menu_entry_name_3", menu_entry_name_3_);
  RCLCPP_INFO(
    logger_, "The parameter menu_entry_name_3 is set to: [%s]", menu_entry_name_3_.c_str());

  // Change the menu entries
  changeMenuEntries();
}

void Display::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up module : %s of type scitos2_module::Display",
    plugin_name_.c_str());
  authority_->checkout();
  authority_.reset();
  display_data_pub_.reset();
}

void Display::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating module : %s of type scitos2_module::Display",
    plugin_name_.c_str());
  display_data_pub_->on_activate();
}

void Display::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating module : %s of type scitos2_module::Display",
    plugin_name_.c_str());
  display_data_pub_->on_deactivate();
}

rcl_interfaces::msg::SetParametersResult Display::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".user_menu_enabled") {
        user_menu_enabled_ = parameter.as_bool();
        RCLCPP_INFO(
          logger_, "The parameter user_menu_enabled is set to: [%s]",
          user_menu_enabled_ ? "true" : "false");
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".menu_name") {
        menu_name_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter menu_name is set to: [%s]", menu_name_.c_str());
      } else if (name == plugin_name_ + ".menu_entry_name_1") {
        menu_entry_name_1_ = parameter.as_string();
        RCLCPP_INFO(
          logger_, "The parameter menu_entry_name_1 is set to: [%s]", menu_entry_name_1_.c_str());
      } else if (name == plugin_name_ + ".menu_entry_name_2") {
        menu_entry_name_2_ = parameter.as_string();
        RCLCPP_INFO(
          logger_, "The parameter menu_entry_name_2 is set to: [%s]", menu_entry_name_2_.c_str());
      } else if (name == plugin_name_ + ".menu_entry_name_3") {
        menu_entry_name_3_ = parameter.as_string();
        RCLCPP_INFO(
          logger_, "The parameter menu_entry_name_3 is set to: [%s]", menu_entry_name_3_.c_str());
      }
    }
  }

  changeMenuEntries();

  result.successful = true;
  return result;
}

void Display::menuDataCallback(mira::ChannelRead<uint8> data)
{
  scitos2_msgs::msg::MenuEntry msg;
  msg.header.stamp = clock_->now();
  msg.entry = data->value();
  display_data_pub_->publish(msg);
}

void Display::changeMenuEntries()
{
  if (user_menu_enabled_) {
    set_mira_param(authority_, "StatusDisplay.EnableUserMenu", "true");
    set_mira_param(authority_, "StatusDisplay.UserMenuName", menu_name_);
    set_mira_param(authority_, "StatusDisplay.UserMenuEntryName1", menu_entry_name_1_);
    set_mira_param(authority_, "StatusDisplay.UserMenuEntryName2", menu_entry_name_2_);
    set_mira_param(authority_, "StatusDisplay.UserMenuEntryName3", menu_entry_name_3_);
  } else {
    set_mira_param(authority_, "StatusDisplay.EnableUserMenu", "false");
  }
}

}  // namespace scitos2_modules

PLUGINLIB_EXPORT_CLASS(scitos2_modules::Display, scitos2_core::Module)
