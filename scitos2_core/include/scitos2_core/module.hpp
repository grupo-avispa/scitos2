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

#ifndef SCITOS2_CORE__MODULE_HPP_
#define SCITOS2_CORE__MODULE_HPP_

#include <memory>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace scitos2_core
{

/**
 * @class scitos2_core::Module
 * @brief Base class for all Scitos modules (Drive, Charger, Display, etc).
 *
 * A pure plugin interface with no MIRA dependency: a module *has* a
 * scitos2_mira_utils::MiraAuthority to talk to MIRA if it needs to, rather than *being* one
 * through a mixin. This keeps scitos2_core usable by non-MIRA modules too, and keeps the MIRA
 * call helpers (now on MiraAuthority) out of every module's inheritance chain and testable on
 * their own.
 */
class Module
{
public:
  using Ptr = std::shared_ptr<scitos2_core::Module>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Module() {}

  /**
   * @param parent pointer to user's node
   * @param name Name of the module
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) = 0;

  /**
   * @brief Method to cleanup resources.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active the module and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive the module and any threads involved in execution.
   */
  virtual void deactivate() = 0;

protected:
  /**
   * @brief Declares static ROS 2 parameter and sets it to a given value if it was not already
   * declared. Kept here (rather than routed through nav2_util) since it has no dependency of
   * its own and scitos2_charging_dock's use of nav2_util for costmap/docking utilities is not
   * a reason to pull nav2_util into every module that just wants to declare a parameter.
   *
   * @param node A node in which given parameter to be declared
   * @param param_name The name of parameter
   * @param default_value Parameter value to initialize with
   * @param parameter_descriptor Parameter descriptor (optional)
   */
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, default_value, parameter_descriptor);
    }
  }
};

}  // namespace scitos2_core

#endif  // SCITOS2_CORE__MODULE_HPP_
