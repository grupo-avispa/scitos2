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

#ifndef SCITOS2_MIRA__MIRA_FRAMEWORK_HPP_
#define SCITOS2_MIRA__MIRA_FRAMEWORK_HPP_

// MIRA
#include <fw/Framework.h>

// C++
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

// Scitos2
#include "scitos2_core/module.hpp"
#include "scitos2_core/sink_logger.hpp"

namespace scitos2_mira
{

/**
 * @class scitos2_mira::MiraFramework
 * @brief Main class that handles the MIRA framework and the modules.
 *
 */
class MiraFramework : public nav2_util::LifecycleNode
{
public:
  using ModuleMap = std::unordered_map<std::string, scitos2_core::Module::Ptr>;

  /**
   * @brief Construct a new laser Segmentation object
   *
   * @param options Node options
   */
  explicit MiraFramework(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the laser Segmentation object
   *
   */
  ~MiraFramework();

protected:
  /**
   * @brief Configures the modules parameters and member variables
   *
   * Configures modules plugin.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize module
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates member variables
   *
   * Activates the modules
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   *
   * Deactivates the modules.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   *
   * module clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // MIRA framework
  std::unique_ptr<mira::Framework> framework_;

  // Modules Plugins
  pluginlib::ClassLoader<scitos2_core::Module> module_loader_;
  ModuleMap modules_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> module_ids_;
  std::vector<std::string> module_types_;
  std::string module_ids_concat_;
};

}  // namespace scitos2_mira

#endif  // SCITOS2_MIRA__MIRA_FRAMEWORK_HPP_
