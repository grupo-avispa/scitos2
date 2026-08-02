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
}

void MiraAuthority::start()
{
  authority_->start();
}

void MiraAuthority::checkout()
{
  authority_->checkout();
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
    rpc.timedWait(mira::Duration::seconds(1));
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
    rpc.timedWait(mira::Duration::seconds(1));
    return rpc.get();
  } catch (mira::XRPC & e) {
    RCLCPP_WARN(
      logger_, "MIRA RPC error caught when getting parameter '%s': %s",
      param_name.c_str(), e.what());
    return "";
  }
}

bool MiraAuthority::isReady() const
{
  return authority_ && authority_->isValid() && authority_->existsService(resource_);
}

}  // namespace scitos2_mira_utils
