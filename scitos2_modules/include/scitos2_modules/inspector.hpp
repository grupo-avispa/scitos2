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

#ifndef SCITOS2_MODULES__INSPECTOR_HPP_
#define SCITOS2_MODULES__INSPECTOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "scitos2_mira_utils/mira_module.hpp"
#include "scitos2_msgs/srv/call_json.hpp"
#include "scitos2_msgs/srv/get_property.hpp"
#include "scitos2_msgs/srv/set_property.hpp"

namespace scitos2_modules
{

/**
 * @class scitos2_modules::Inspector
 * @brief Module that gives access to any property and any method of the robot through generic
 * ROS services, to explore and tweak it from the command line without writing code.
 *
 * The services give full access to the robot from the whole ROS network, so they are only
 * created with the parameter `enabled` set to true.
 */
class Inspector : public scitos2_mira_utils::MiraModule
{
public:
  /**
   * @brief Construct for scitos2_modules::Inspector
   */
  Inspector() = default;

  /**
   * @brief Destructor for scitos2_modules::Inspector
   */
  ~Inspector() override = default;

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

protected:
  /**
   * @brief Service to read a property of the robot.
   *
   * @param request Name of the property
   * @param response Value of the property
   */
  void getProperty(
    const std::shared_ptr<scitos2_msgs::srv::GetProperty::Request> request,
    std::shared_ptr<scitos2_msgs::srv::GetProperty::Response> response);

  /**
   * @brief Service to write a property of the robot.
   *
   * @param request Name and new value of the property
   * @param response Result of the write
   */
  void setProperty(
    const std::shared_ptr<scitos2_msgs::srv::SetProperty::Request> request,
    std::shared_ptr<scitos2_msgs::srv::SetProperty::Response> response);

  /**
   * @brief Service to call a method of a MIRA service with JSON.
   *
   * @param request Service, method and parameters of the call
   * @param response Result of the call
   */
  void callJson(
    const std::shared_ptr<scitos2_msgs::srv::CallJson::Request> request,
    std::shared_ptr<scitos2_msgs::srv::CallJson::Response> response);

  /** @brief Whether the services exist, from the parameter `enabled`. */
  bool enabled_{false};

  /** @brief Whether the services answer: they are created at configure time. */
  bool is_active_{false};

  /** @brief Service to read a property. */
  rclcpp::Service<scitos2_msgs::srv::GetProperty>::SharedPtr get_property_service_;

  /** @brief Service to write a property. */
  rclcpp::Service<scitos2_msgs::srv::SetProperty>::SharedPtr set_property_service_;

  /** @brief Service to call a method with JSON. */
  rclcpp::Service<scitos2_msgs::srv::CallJson>::SharedPtr call_json_service_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__INSPECTOR_HPP_
