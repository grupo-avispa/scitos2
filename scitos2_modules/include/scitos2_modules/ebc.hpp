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

#ifndef SCITOS2_MODULES__EBC_HPP_
#define SCITOS2_MODULES__EBC_HPP_

// MIRA
#include <fw/Framework.h>

// C++
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "scitos2_core/module.hpp"

namespace scitos2_modules
{

/**
 * @brief A MIRA boolean property exposed as a ROS parameter to enable/disable an EBC power port.
 */
struct EbcBoolPort
{
  std::string param;
  std::string mira_key;
  std::string description;
};

/**
 * @brief A MIRA numeric property exposed as a ROS parameter for an EBC power port max current.
 * The declared parameter range (0, max_current] is what rcl enforces before the dynamic
 * parameters callback is even invoked, so it must match whatever the callback validates.
 */
struct EbcCurrentPort
{
  std::string param;
  std::string mira_key;
  std::string description;
  double default_value;
  double max_current;
};

/**
 * @class scitos2_modules::EBC
 * @brief Module for the EBC power board control.
 *
 */
class EBC : public scitos2_core::Module
{
public:
  /**
   * @brief Construct for scitos2_modules::EBC
   */
  EBC() = default;

  /**
   * @brief Destructor for scitos2_modules::EBC
   */
  ~EBC() override = default;

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
  void activate() override;

  /**
   * @brief Deactivate the module state machine.
   */
  void deactivate() override;

protected:
  /**
   * @brief Callback executed when a parameter change is detected.
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // MIRA Authority
  std::shared_ptr<mira::Authority> authority_;

  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("EBC")};

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Table of EBC power ports, single source of truth for both the declared parameter
  // descriptor and the dynamic parameters callback validation
  static const std::vector<EbcBoolPort> kBoolPorts;
  static const std::vector<EbcCurrentPort> kCurrentPorts;

  // Current value of each port parameter, keyed by EbcBoolPort::param / EbcCurrentPort::param.
  // Written in activate(), after the authority has started, since MIRA parameters cannot be
  // set before that.
  std::map<std::string, bool> port_enabled_;
  std::map<std::string, double> port_max_current_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__EBC_HPP_
