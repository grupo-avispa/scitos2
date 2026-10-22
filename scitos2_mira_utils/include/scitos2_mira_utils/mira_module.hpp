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

#ifndef SCITOS2_MIRA_UTILS__MIRA_MODULE_HPP_
#define SCITOS2_MIRA_UTILS__MIRA_MODULE_HPP_

#include <memory>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "scitos2_core/module.hpp"
#include "scitos2_mira_utils/mira_authority.hpp"

namespace scitos2_mira_utils
{

/**
 * @class scitos2_mira_utils::MiraModule
 * @brief Base class of the modules that talk to MIRA through a MiraAuthority.
 *
 * It owns the part that every one of them repeated: create the authority and check it in,
 * read the parameters that say which MIRA resource to call and how long to wait for it, start
 * it, and check it out when the module is cleaned up. It lives here and not in scitos2_core so
 * that the module interface stays free of MIRA.
 */
class MiraModule : public scitos2_core::Module
{
public:
  /**
   * @brief Virtual destructor.
   */
  ~MiraModule() override = default;

protected:
  /**
   * @brief Create the authority and read the parameters `mira_robot_resource` and
   * `mira_rpc_timeout` of the module. Call it first in `configure()`.
   *
   * @param parent WeakPtr to the node
   * @param name Name of the plugin, used as the name of the authority and as the prefix of the
   * parameters
   * @return rclcpp_lifecycle::LifecycleNode::SharedPtr The node, locked
   * @throw std::runtime_error If the node does not exist anymore
   */
  rclcpp_lifecycle::LifecycleNode::SharedPtr configureMira(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name);

  /**
   * @brief Start the authority. A second call is harmless.
   *
   * @return bool True if the authority is started, false if MIRA refused it
   */
  bool startMira();

  /**
   * @brief Check the authority out and release it. Call it in `cleanup()`.
   */
  void cleanupMira();

  /** @brief MIRA authority of the module. */
  std::shared_ptr<MiraAuthority> authority_;

  /** @brief Name of the plugin, which is also the prefix of its parameters. */
  std::string plugin_name_;

  /** @brief Logger of the node that owns the module. */
  rclcpp::Logger logger_{rclcpp::get_logger("MIRA module")};
};

}  // namespace scitos2_mira_utils

#endif  // SCITOS2_MIRA_UTILS__MIRA_MODULE_HPP_
