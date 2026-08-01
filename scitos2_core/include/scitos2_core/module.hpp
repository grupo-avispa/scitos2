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

#include <fw/Authority.h>
#include <rpc/RPCError.h>

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace scitos2_core
{

/**
 * @class scitos2_core::Module
 * @brief Base class for all Scitos modules (Drive, Charger, Display, etc).
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
  // Skip this method from coverage report because it only calls MIRA services
  // LCOV_EXCL_START
  /**
   * @brief Call a MIRA service with a timeout of 1 second.
   *
   * @param authority The MIRA authority
   * @param service_name The name of the service
   * @param logger Logger to attribute error/debug messages to. Defaults to a generic "MIRA"
   * logger; pass the calling module's own logger_ to make failures traceable to their origin.
   * @return bool If the service was called successfully
   */
  bool call_mira_service(
    const std::weak_ptr<mira::Authority> & authority, std::string service_name,
    const rclcpp::Logger & logger = rclcpp::get_logger("MIRA"))
  {
    // Convert weak_ptr to shared_ptr
    auto sharedAuthority = authority.lock();
    if (!sharedAuthority) {
      return false;
    }

    // Check if the authority is valid or if the service exists
    if (!sharedAuthority->isValid() || !sharedAuthority->existsService(mira_robot_resource_)) {
      RCLCPP_ERROR_ONCE(
        logger, "MIRA authority is not valid or resource '%s' does not exist",
        mira_robot_resource_.c_str());
      return false;
    }

    try {
      mira::RPCFuture<void> rpc =
        sharedAuthority->callService<void>(mira_robot_resource_, service_name);
      rpc.timedWait(mira::Duration::seconds(1));
      rpc.get();
      RCLCPP_DEBUG(logger, "MIRA service '%s' called successfully", service_name.c_str());
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        logger, "MIRA RPC error caught when calling the service '%s': %s",
        service_name.c_str(), e.what());
      return false;
    }
    return true;
  }

  /**
   * @brief Call a MIRA service with a timeout of 1 second.
   *
   * @param authority The MIRA authority
   * @param service_name The name of the service
   * @param request The request to send. Empty by default
   * @param logger Logger to attribute error/debug messages to. Defaults to a generic "MIRA"
   * logger; pass the calling module's own logger_ to make failures traceable to their origin.
   * @return bool If the service was called successfully
   */
  template<typename T>
  bool call_mira_service(
    const std::weak_ptr<mira::Authority> & authority, std::string service_name,
    std::optional<T> request = std::nullopt,
    const rclcpp::Logger & logger = rclcpp::get_logger("MIRA"))
  {
    // Convert weak_ptr to shared_ptr
    auto sharedAuthority = authority.lock();
    if (!sharedAuthority) {
      return false;
    }

    // Check if the authority is valid or if the service exists
    if (!sharedAuthority->isValid() || !sharedAuthority->existsService(mira_robot_resource_)) {
      RCLCPP_ERROR_ONCE(
        logger, "MIRA authority is not valid or resource '%s' does not exist",
        mira_robot_resource_.c_str());
      return false;
    }

    try {
      mira::RPCFuture<void> rpc;
      if (request.has_value()) {
        rpc = sharedAuthority->callService<void>(
          mira_robot_resource_, service_name, request.value());
      } else {
        rpc = sharedAuthority->callService<void>(mira_robot_resource_, service_name);
      }
      rpc.timedWait(mira::Duration::seconds(1));
      rpc.get();
      RCLCPP_DEBUG(logger, "MIRA service '%s' called successfully", service_name.c_str());
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        logger, "MIRA RPC error caught when calling the service '%s': %s",
        service_name.c_str(), e.what());
      return false;
    }
    return true;
  }

  /**
   * @brief Set a MIRA parameter.
   *
   * @param authority The MIRA authority
   * @param param_name The name of the parameter
   * @param value The value to set
   * @param logger Logger to attribute error/debug messages to. Defaults to a generic "MIRA"
   * logger; pass the calling module's own logger_ to make failures traceable to their origin.
   * @return bool If the parameter was set successfully
   */
  bool set_mira_param(
    const std::weak_ptr<mira::Authority> & authority, std::string param_name,
    std::string value, const rclcpp::Logger & logger = rclcpp::get_logger("MIRA"))
  {
    // Convert weak_ptr to shared_ptr
    auto sharedAuthority = authority.lock();
    if (!sharedAuthority) {
      return false;
    }

    // Check if the authority is valid or if the service exists
    if (!sharedAuthority->isValid() || !sharedAuthority->existsService(mira_robot_resource_)) {
      RCLCPP_ERROR_ONCE(
        logger, "MIRA authority is not valid or resource '%s' does not exist",
        mira_robot_resource_.c_str());
      return false;
    }

    try {
      mira::RPCFuture<void> rpc = sharedAuthority->callService<void>(
        mira_robot_resource_ + "#builtin", std::string("setProperty"), param_name, value);
      rpc.timedWait(mira::Duration::seconds(1));
      rpc.get();
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        logger, "MIRA RPC error caught when setting parameter '%s': %s",
        param_name.c_str(), e.what());
      return false;
    }
    return true;
  }

  /**
   * @brief Get the value of a MIRA parameter.
   *
   * @param authority The MIRA authority
   * @param param_name The name of the parameter
   * @param logger Logger to attribute error/debug messages to. Defaults to a generic "MIRA"
   * logger; pass the calling module's own logger_ to make failures traceable to their origin.
   * @return std::string The value of the parameter
   */
  std::string get_mira_param(
    const std::weak_ptr<mira::Authority> & authority, std::string param_name,
    const rclcpp::Logger & logger = rclcpp::get_logger("MIRA"))
  {
    // Convert weak_ptr to shared_ptr
    auto sharedAuthority = authority.lock();
    if (!sharedAuthority) {
      return "";
    }

    // Check if the authority is valid or if the service exists
    if (!sharedAuthority->isValid() || !sharedAuthority->existsService(mira_robot_resource_)) {
      RCLCPP_ERROR_ONCE(
        logger, "MIRA authority is not valid or resource '%s' does not exist",
        mira_robot_resource_.c_str());
      return "";
    }

    try {
      mira::RPCFuture<std::string> rpc = sharedAuthority->callService<std::string>(
        mira_robot_resource_ + "#builtin", std::string("getProperty"), param_name);
      rpc.timedWait(mira::Duration::seconds(1));
      return rpc.get();
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        logger, "MIRA RPC error caught when getting parameter '%s': %s",
        param_name.c_str(), e.what());
      return "";
    }
  }
  // LCOV_EXCL_STOP

  // MIRA resource that exposes the robot's services and properties. Configurable per module
  // in case the robot's MIRA XML configuration names it differently than the default.
  std::string mira_robot_resource_{"/robot/Robot"};

/**
 * @brief Declares static ROS2 parameter and sets it to a given value if it was not already declared.
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
