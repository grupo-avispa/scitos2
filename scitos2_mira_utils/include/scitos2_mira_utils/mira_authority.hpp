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

#ifndef SCITOS2_MIRA_UTILS__MIRA_AUTHORITY_HPP_
#define SCITOS2_MIRA_UTILS__MIRA_AUTHORITY_HPP_

#include <fw/Authority.h>
#include <rpc/RPCError.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/logger.hpp"

namespace scitos2_mira_utils
{

/**
 * @class scitos2_mira_utils::MiraAuthority
 * @brief Encapsulates a MIRA authority: the checkin/start/checkout lifecycle, channel
 * subscriptions, and service/parameter calls against a configurable robot resource.
 *
 * This used to be a set of protected helpers mixed into scitos2_core::Module, which forced
 * scitos2_core (the plugin interface) to depend on MIRA and made the helpers untestable
 * (LCOV_EXCL'd). Pulling it out into its own compiled class means a module *has* an authority
 * instead of *being* one, and it can be tested with a fake authority instead of a live MIRA
 * framework.
 */
class MiraAuthority
{
public:
  /**
   * @brief Construct a MiraAuthority. Does not check in yet, see checkin().
   *
   * @param logger Logger used to attribute error/debug messages to their caller
   */
  explicit MiraAuthority(const rclcpp::Logger & logger = rclcpp::get_logger("MIRA"));

  ~MiraAuthority();

  /**
   * @brief Set the MIRA resource that exposes the robot's services and properties.
   * Must be called before the first service/parameter call; defaults to "/robot/Robot".
   *
   * @param resource The MIRA resource name
   */
  void setResource(const std::string & resource);

  /**
   * @brief Check in the authority under "/" with the given name.
   * @param name The name of the authority (typically the plugin name)
   */
  void checkin(const std::string & name);

  /**
   * @brief Start the authority. Must be called before any service/parameter call can succeed.
   */
  void start();

  /**
   * @brief Check out the authority.
   */
  void checkout();

  /**
   * @brief Subscribe to a MIRA channel.
   *
   * @tparam T The type of the channel data
   * @param channel The name of the channel
   * @param callback The callback to invoke when new data arrives
   */
  template<typename T>
  void subscribe(const std::string & channel, std::function<void(mira::ChannelRead<T>)> callback)
  {
    authority_->subscribe<T>(channel, callback);
  }

  /**
   * @brief Call a MIRA service with a timeout of 1 second, with no request payload.
   *
   * @param service_name The name of the service
   * @return bool If the service was called successfully
   */
  bool callService(const std::string & service_name);

  /**
   * @brief Call a MIRA service with a timeout of 1 second.
   *
   * @tparam T The type of the request payload
   * @param service_name The name of the service
   * @param request The request to send. Empty by default
   * @return bool If the service was called successfully
   */
  template<typename T>
  bool callService(const std::string & service_name, std::optional<T> request = std::nullopt)
  {
    if (!isReady()) {
      RCLCPP_ERROR_ONCE(
        logger_, "MIRA authority is not valid or resource '%s' does not exist",
        resource_.c_str());
      return false;
    }

    try {
      mira::RPCFuture<void> rpc;
      if (request.has_value()) {
        rpc = authority_->callService<void>(resource_, service_name, request.value());
      } else {
        rpc = authority_->callService<void>(resource_, service_name);
      }
      rpc.timedWait(mira::Duration::seconds(1));
      rpc.get();
      RCLCPP_DEBUG(logger_, "MIRA service '%s' called successfully", service_name.c_str());
    } catch (mira::XRPC & e) {
      RCLCPP_WARN(
        logger_, "MIRA RPC error caught when calling the service '%s': %s",
        service_name.c_str(), e.what());
      return false;
    }
    return true;
  }

  /**
   * @brief Set a MIRA parameter.
   *
   * @param param_name The name of the parameter
   * @param value The value to set
   * @return bool If the parameter was set successfully
   */
  bool setParam(const std::string & param_name, const std::string & value);

  /**
   * @brief Get the value of a MIRA parameter.
   *
   * @param param_name The name of the parameter
   * @return std::string The value of the parameter, or an empty string on failure
   */
  std::string getParam(const std::string & param_name);

protected:
  /**
   * @brief Whether the authority is valid and the configured resource exists.
   * @return bool If a service/parameter call is expected to succeed
   */
  bool isReady() const;

  std::shared_ptr<mira::Authority> authority_;
  std::string resource_{"/robot/Robot"};
  rclcpp::Logger logger_;
};

}  // namespace scitos2_mira_utils

#endif  // SCITOS2_MIRA_UTILS__MIRA_AUTHORITY_HPP_
