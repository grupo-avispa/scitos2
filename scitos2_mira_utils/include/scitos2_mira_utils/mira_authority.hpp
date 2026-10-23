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

// Authority.h enforces (via #error) that it is only ever included through Framework.h
#include <fw/Framework.h>
#include <rpc/RPCError.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

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
   * @brief Get the MIRA resource whose services and properties are called.
   * @return const std::string & The resource name
   */
  const std::string & resource() const
  {
    return resource_;
  }

  /**
   * @brief Check in the authority under a MIRA namespace with the given name.
   *
   * The default namespace keeps the authorities of ROS together (`/ros2/drive`, `/ros2/imu`...),
   * apart from the units of the robot, and recognizable in `miracenter`.
   *
   * @param name The name of the authority (typically the plugin name)
   * @param mira_namespace The MIRA namespace of the authority
   */
  void checkin(const std::string & name, const std::string & mira_namespace = "/ros2");

  /**
   * @brief Start the authority. Must be called before any service/parameter call can succeed.
   * Starting an authority that is already started is a no-op, so a module can be activated again
   * after a deactivation without touching the MIRA side.
   */
  void start();

  /**
   * @brief Check out the authority. Does nothing if it is not checked in.
   */
  void checkout();

  /**
   * @brief Set how long a service/parameter call waits for the answer of MIRA before giving up.
   * Defaults to 1 second.
   *
   * @param timeout The timeout of the calls. Must be positive
   */
  void setTimeout(const std::chrono::milliseconds & timeout);

  /**
   * @brief Wait for the configured resource to be available in MIRA.
   *
   * @param timeout The maximum time to wait
   * @return bool True if the resource exists, false if the time ran out
   */
  bool waitForResource(const std::chrono::milliseconds & timeout);

  /**
   * @brief Whether the authority is valid and the configured resource exists.
   * @return bool If a service/parameter call is expected to succeed
   */
  bool isReady() const;

  /**
   * @brief Wait for a channel to be published in MIRA, without creating it.
   *
   * @param channel The name of the channel
   * @param timeout The maximum time to wait
   * @return bool True if the channel exists, false if the time ran out
   */
  bool waitForChannel(const std::string & channel, const std::chrono::milliseconds & timeout);

  /**
   * @brief Subscribe to a MIRA channel.
   *
   * @tparam T The type of the channel data
   * @param channel The name of the channel
   * @param callback The callback to invoke when new data arrives
   * @param independent_thread Run the callback in its own thread, so that a slow callback of
   * the authority does not delay it. The callback must then protect the state it shares
   */
  template<typename T>
  void subscribe(
    const std::string & channel, std::function<void(mira::ChannelRead<T>)> callback,
    bool independent_thread = false)
  {
    authority_->subscribe<T>(channel, callback, independent_thread);
  }

  /**
   * @brief Call a MIRA service, with no request payload.
   *
   * @param service_name The name of the service
   * @return bool If the service was called successfully
   */
  bool callService(const std::string & service_name);

  /**
   * @brief Call a MIRA service.
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
      if (!rpc.timedWait(timeout_)) {
        RCLCPP_WARN(
          logger_, "MIRA service '%s' timed out after %ld ms", service_name.c_str(),
          static_cast<long>(timeout_.total_milliseconds()));  // NOLINT
        return false;
      }
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
   * @brief Call a MIRA service without waiting for the answer.
   *
   * The call is kept until the next one. If the previous call has not been answered by then it is
   * dropped, so a stream of commands (e.g. velocities) never accumulates behind a slow robot, and
   * the caller is never blocked. The error of a previous call is reported on the next one.
   *
   * @tparam T The type of the request payload
   * @param service_name The name of the service
   * @param request The request to send
   * @return bool If the call was sent
   */
  template<typename T>
  bool callServiceAsync(const std::string & service_name, const T & request)
  {
    if (!isReady()) {
      RCLCPP_ERROR_ONCE(
        logger_, "MIRA authority is not valid or resource '%s' does not exist",
        resource_.c_str());
      return false;
    }

    std::lock_guard<std::mutex> lock(async_mutex_);
    try {
      checkAsyncResult(service_name);
      pending_rpc_ = authority_->callService<void>(resource_, service_name, request);
      has_pending_rpc_ = true;
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

  /**
   * @brief Get the value of a MIRA parameter, telling a failure from an empty value.
   *
   * @param param_name The name of the parameter
   * @param value The value of the parameter. Untouched on failure
   * @return bool If the parameter was read successfully
   */
  bool getParam(const std::string & param_name, std::string & value);

  /**
   * @brief Call a method of any MIRA service with JSON as the payload.
   *
   * @param service The MIRA service (e.g. "/robot/Robot")
   * @param method The method of the service
   * @param params The parameters of the method as a JSON array. Empty if there are none
   * @param result The result of the call, as JSON. Untouched on failure
   * @param error The reason of the failure, if any
   * @return bool If the call was answered
   */
  bool callJson(
    const std::string & service, const std::string & method, const std::string & params,
    std::string & result, std::string & error);

protected:
  /**
   * @brief Report the error of the previous asynchronous call, if it finished with one, and
   * warn if it was never answered. The caller must hold async_mutex_.
   *
   * @param service_name The name of the service of the new call
   */
  void checkAsyncResult(const std::string & service_name);

  std::shared_ptr<mira::Authority> authority_;
  std::string resource_{"/robot/Robot"};
  mira::Duration timeout_{mira::Duration::seconds(1)};
  std::mutex async_mutex_;
  mira::RPCFuture<void> pending_rpc_;
  bool has_pending_rpc_{false};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
  bool checked_in_{false};
  bool started_{false};
  rclcpp::Logger logger_;
};

}  // namespace scitos2_mira_utils

#endif  // SCITOS2_MIRA_UTILS__MIRA_AUTHORITY_HPP_
