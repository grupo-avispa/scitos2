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

#include "scitos2_modules/inspector.hpp"

#include <string>

namespace scitos2_modules
{

using std::placeholders::_1;
using std::placeholders::_2;

void Inspector::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
  auto node = configureMira(parent, name);

  get_property_service_ = node->create_service<scitos2_msgs::srv::GetProperty>(
    "mira/get_property", std::bind(&Inspector::getProperty, this, _1, _2));
  set_property_service_ = node->create_service<scitos2_msgs::srv::SetProperty>(
    "mira/set_property", std::bind(&Inspector::setProperty, this, _1, _2));
  call_json_service_ = node->create_service<scitos2_msgs::srv::CallJson>(
    "mira/call_json", std::bind(&Inspector::callJson, this, _1, _2));

  RCLCPP_INFO(logger_, "Configured module : %s", plugin_name_.c_str());
}

void Inspector::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type scitos2_module::Inspector", plugin_name_.c_str());
  get_property_service_.reset();
  set_property_service_.reset();
  call_json_service_.reset();
  cleanupMira();
}

bool Inspector::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type scitos2_module::Inspector", plugin_name_.c_str());
  if (!startMira()) {
    return false;
  }
  is_active_ = true;
  return true;
}

bool Inspector::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type scitos2_module::Inspector", plugin_name_.c_str());
  is_active_ = false;
  return true;
}

void Inspector::getProperty(
  const std::shared_ptr<scitos2_msgs::srv::GetProperty::Request> request,
  std::shared_ptr<scitos2_msgs::srv::GetProperty::Response> response)
{
  if (!is_active_) {
    response->message = "The module is not active";
    return;
  }
  response->success = authority_->getParam(request->name, response->value);
  if (!response->success) {
    response->message = "Unable to read the property '" + request->name + "'";
  }
}

void Inspector::setProperty(
  const std::shared_ptr<scitos2_msgs::srv::SetProperty::Request> request,
  std::shared_ptr<scitos2_msgs::srv::SetProperty::Response> response)
{
  if (!is_active_) {
    response->message = "The module is not active";
    return;
  }
  response->success = authority_->setParam(request->name, request->value);
  if (!response->success) {
    response->message = "Unable to write the property '" + request->name + "'";
  }
}

void Inspector::callJson(
  const std::shared_ptr<scitos2_msgs::srv::CallJson::Request> request,
  std::shared_ptr<scitos2_msgs::srv::CallJson::Response> response)
{
  if (!is_active_) {
    response->message = "The module is not active";
    return;
  }
  response->success = authority_->callJson(
    request->service, request->method, request->params, response->result, response->message);
}

}  // namespace scitos2_modules

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(scitos2_modules::Inspector, scitos2_core::Module)
