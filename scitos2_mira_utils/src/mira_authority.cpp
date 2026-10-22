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

#include "scitos2_mira_utils/mira_authority.hpp"

#include <chrono>
#include <thread>

#include "rclcpp/logging.hpp"

namespace scitos2_mira_utils
{

MiraAuthority::MiraAuthority(const rclcpp::Logger & logger)
: logger_(logger)
{
  authority_ = std::make_shared<mira::Authority>();
}

MiraAuthority::~MiraAuthority()
{
}

void MiraAuthority::setResource(const std::string & resource)
{
  resource_ = resource;
}

void MiraAuthority::checkin(const std::string & name)
{
  authority_->checkin("/", name);
  checked_in_ = true;
}

void MiraAuthority::start()
{
  if (started_) {
    return;
  }
  authority_->start();
  started_ = true;
}

void MiraAuthority::checkout()
{
  if (!checked_in_) {
    return;
  }
  // Mark it first: whatever happens in checkout() the authority can't be reused afterwards
  checked_in_ = false;
  started_ = false;
  authority_->checkout();
}

void MiraAuthority::setTimeout(const std::chrono::milliseconds & timeout)
{
  timeout_ = mira::Duration::milliseconds(timeout.count());
}

bool MiraAuthority::waitForResource(const std::chrono::milliseconds & timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!isReady()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return true;
}

bool MiraAuthority::callService(const std::string & service_name)
{
  if (!isReady()) {
    RCLCPP_ERROR_ONCE(
      logger_, "MIRA authority is not valid or resource '%s' does not exist", resource_.c_str());
    return false;
  }

  try {
    mira::RPCFuture<void> rpc = authority_->callService<void>(resource_, service_name);
    if (!rpc.timedWait(timeout_)) {
      RCLCPP_WARN(
        logger_, "MIRA service '%s' timed out after %ld ms",
        service_name.c_str(),
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

bool MiraAuthority::setParam(const std::string & param_name, const std::string & value)
{
  if (!isReady()) {
    RCLCPP_ERROR_ONCE(
      logger_, "MIRA authority is not valid or resource '%s' does not exist", resource_.c_str());
    return false;
  }

  try {
    mira::RPCFuture<void> rpc = authority_->callService<void>(
      resource_ + "#builtin", std::string("setProperty"), param_name, value);
    if (!rpc.timedWait(timeout_)) {
      RCLCPP_WARN(
        logger_, "MIRA service '%s' timed out after %ld ms",
        param_name.c_str(),
        static_cast<long>(timeout_.total_milliseconds()));  // NOLINT
      return false;
    }
    rpc.get();
  } catch (mira::XRPC & e) {
    RCLCPP_WARN(
      logger_, "MIRA RPC error caught when setting parameter '%s': %s",
      param_name.c_str(), e.what());
    return false;
  }
  return true;
}

std::string MiraAuthority::getParam(const std::string & param_name)
{
  if (!isReady()) {
    RCLCPP_ERROR_ONCE(
      logger_, "MIRA authority is not valid or resource '%s' does not exist", resource_.c_str());
    return "";
  }

  try {
    mira::RPCFuture<std::string> rpc = authority_->callService<std::string>(
      resource_ + "#builtin", std::string("getProperty"), param_name);
    if (!rpc.timedWait(timeout_)) {
      RCLCPP_WARN(
        logger_, "MIRA service '%s' timed out after %ld ms",
        param_name.c_str(),
        static_cast<long>(timeout_.total_milliseconds()));  // NOLINT
      return "";
    }
    return rpc.get();
  } catch (mira::XRPC & e) {
    RCLCPP_WARN(
      logger_, "MIRA RPC error caught when getting parameter '%s': %s",
      param_name.c_str(), e.what());
    return "";
  }
}

bool MiraAuthority::getParam(const std::string & param_name, std::string & value)
{
  if (!isReady()) {
    RCLCPP_ERROR_ONCE(
      logger_, "MIRA authority is not valid or resource '%s' does not exist", resource_.c_str());
    return false;
  }

  try {
    mira::RPCFuture<std::string> rpc = authority_->callService<std::string>(
      resource_ + "#builtin", std::string("getProperty"), param_name);
    if (!rpc.timedWait(timeout_)) {
      RCLCPP_WARN(
        logger_, "MIRA service '%s' timed out after %ld ms", param_name.c_str(),
        static_cast<long>(timeout_.total_milliseconds()));  // NOLINT
      return false;
    }
    value = rpc.get();
  } catch (mira::XRPC & e) {
    RCLCPP_WARN(
      logger_, "MIRA RPC error caught when getting parameter '%s': %s",
      param_name.c_str(), e.what());
    return false;
  }
  return true;
}

bool MiraAuthority::callJson(
  const std::string & service, const std::string & method, const std::string & params,
  std::string & result, std::string & error)
{
  if (!authority_ || !authority_->isValid() || !authority_->existsService(service)) {
    error = "MIRA service '" + service + "' does not exist";
    return false;
  }

  try {
    mira::json::Value json_params;
    if (!params.empty()) {
      mira::json::read(params, json_params);
    }
    mira::RPCFuture<mira::JSONRPCResponse> rpc =
      authority_->callServiceJSON(service, method, json_params);
    if (!rpc.timedWait(timeout_)) {
      error = "MIRA service '" + service + "' timed out";
      return false;
    }
    result = mira::json::write(rpc.get());
  } catch (const std::exception & e) {
    // Malformed JSON and the errors reported by the remote side end up here
    error = e.what();
    return false;
  }
  return true;
}

bool MiraAuthority::isReady() const
{
  return authority_ && authority_->isValid() && authority_->existsService(resource_);
}

}  // namespace scitos2_mira_utils
