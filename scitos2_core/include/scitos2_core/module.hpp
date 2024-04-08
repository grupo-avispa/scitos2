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

#include "rclcpp/rclcpp.hpp"
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
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name) = 0;

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
   * @brief Call a MIRA service with a timeout of 1 second.
   *
   * @param authority The MIRA authority
   * @param service_name The name of the service
   * @return bool If the service was called successfully
   */
  bool call_mira_service(const std::weak_ptr<mira::Authority> & authority, std::string service_name)
  {
    // Convert weak_ptr to shared_ptr
    auto sharedAuthority = authority.lock();
    if (!sharedAuthority) {
      return false;
    }

    // Check if the authority is valid or if the service exists
    if (!sharedAuthority->isValid() || !sharedAuthority->existsService("/robot/Robot")) {
      RCLCPP_ERROR_ONCE(
        rclcpp::get_logger("MIRA"),
        "MIRA authority is not valid or service does not exist");
      return false;
    }

    try {
      mira::RPCFuture<void> rpc = sharedAuthority->callService<void>("/robot/Robot", service_name);
      rpc.timedWait(mira::Duration::seconds(1));
      rpc.get();
      RCLCPP_DEBUG(rclcpp::get_logger("MIRA"), "service_name: %i", true);
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("MIRA"),
        "MIRA RPC error caught when calling the service: %s", e.what() );
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
   * @return bool If the service was called successfully
   */
  template<typename T>
  bool call_mira_service(
    const std::weak_ptr<mira::Authority> & authority, std::string service_name,
    std::optional<T> request = std::nullopt)
  {
    // Convert weak_ptr to shared_ptr
    auto sharedAuthority = authority.lock();
    if (!sharedAuthority) {
      return false;
    }

    // Check if the authority is valid or if the service exists
    if (!sharedAuthority->isValid() || !sharedAuthority->existsService("/robot/Robot")) {
      RCLCPP_ERROR_ONCE(
        rclcpp::get_logger("MIRA"),
        "MIRA authority is not valid or service does not exist");
      return false;
    }

    try {
      mira::RPCFuture<void> rpc;
      if (request.has_value()) {
        rpc = sharedAuthority->callService<void>("/robot/Robot", service_name, request.value());
      } else {
        rpc = sharedAuthority->callService<void>("/robot/Robot", service_name);
      }
      rpc.timedWait(mira::Duration::seconds(1));
      rpc.get();
      RCLCPP_DEBUG(rclcpp::get_logger("MIRA"), "service_name: %i", true);
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("MIRA"),
        "MIRA RPC error caught when calling the service: %s", e.what() );
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
   * @return bool If the parameter was set successfully
   */
  bool set_mira_param(
    const std::weak_ptr<mira::Authority> & authority, std::string param_name,
    std::string value)
  {
    // Convert weak_ptr to shared_ptr
    auto sharedAuthority = authority.lock();
    if (!sharedAuthority) {
      return false;
    }

    // Check if the authority is valid or if the service exists
    if (!sharedAuthority->isValid() || !sharedAuthority->existsService("/robot/Robot")) {
      RCLCPP_ERROR_ONCE(
        rclcpp::get_logger("MIRA"),
        "MIRA authority is not valid or service does not exist");
      return false;
    }

    try {
      mira::RPCFuture<void> rpc = sharedAuthority->callService<void>(
        "/robot/Robot#builtin", std::string("setProperty"), param_name, value);
      rpc.timedWait(mira::Duration::seconds(1));
      rpc.get();
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("MIRA"),
        "MIRA RPC error caught when setting parameter: %s", e.what() );
      return false;
    }
    return true;
  }

  /**
   * @brief Get the value of a MIRA parameter.
   *
   * @param authority The MIRA authority
   * @param param_name The name of the parameter
   * @return std::string The value of the parameter
   */
  std::string get_mira_param(
    const std::weak_ptr<mira::Authority> & authority,
    std::string param_name)
  {
    // Convert weak_ptr to shared_ptr
    auto sharedAuthority = authority.lock();
    if (!sharedAuthority) {
      return "";
    }

    // Check if the authority is valid or if the service exists
    if (!sharedAuthority->isValid() || !sharedAuthority->existsService("/robot/Robot")) {
      RCLCPP_ERROR_ONCE(
        rclcpp::get_logger("MIRA"),
        "MIRA authority is not valid or service does not exist");
      return "";
    }

    try {
      mira::RPCFuture<std::string> rpc = sharedAuthority->callService<std::string>(
        "/robot/Robot#builtin", std::string("getProperty"), param_name);
      rpc.timedWait(mira::Duration::seconds(1));
      return rpc.get();
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("MIRA"),
        "MIRA RPC error caught when getting parameter: %s", e.what() );
      return "";
    }
  }
};

}  // namespace scitos2_core

#endif  // SCITOS2_CORE__MODULE_HPP_
