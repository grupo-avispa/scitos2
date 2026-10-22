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

#include "scitos2_mira_utils/mira_module.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace scitos2_mira_utils
{

rclcpp_lifecycle::LifecycleNode::SharedPtr MiraModule::configureMira(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  logger_ = node->get_logger();
  authority_ = std::make_shared<MiraAuthority>(logger_);
  authority_->checkin(plugin_name_);

  std::string mira_robot_resource = "/robot/Robot";
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mira_robot_resource",
    rclcpp::ParameterValue(mira_robot_resource), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The MIRA resource that exposes the robot's services and properties"));
  node->get_parameter(plugin_name_ + ".mira_robot_resource", mira_robot_resource);
  authority_->setResource(mira_robot_resource);

  int mira_rpc_timeout = 1000;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mira_rpc_timeout",
    rclcpp::ParameterValue(mira_rpc_timeout), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Milliseconds to wait for MIRA to answer a service or property call"));
  node->get_parameter(plugin_name_ + ".mira_rpc_timeout", mira_rpc_timeout);
  authority_->setTimeout(std::chrono::milliseconds(std::max(mira_rpc_timeout, 1)));

  return node;
}

bool MiraModule::startMira()
{
  try {
    authority_->start();
  } catch (const mira::Exception & ex) {
    RCLCPP_ERROR(
      logger_, "Failed to start the MIRA authority of '%s'. Exception: %s",
      plugin_name_.c_str(), ex.what());
    return false;
  }
  return true;
}

void MiraModule::cleanupMira()
{
  if (authority_) {
    try {
      authority_->checkout();
    } catch (const mira::Exception & ex) {
      RCLCPP_ERROR(
        logger_, "Failed to check out the MIRA authority of '%s'. Exception: %s",
        plugin_name_.c_str(), ex.what());
    }
  }
  authority_.reset();
}

}  // namespace scitos2_mira_utils
