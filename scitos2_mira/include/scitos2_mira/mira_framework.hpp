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
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

// Scitos2
#include "scitos2_core/module.hpp"
#include "scitos2_mira_utils/mira_authority.hpp"
#include "scitos2_mira_utils/sink_logger.hpp"

namespace scitos2_mira
{

/**
 * @class scitos2_mira::MiraFramework
 * @brief Main class that handles the MIRA framework and the modules.
 *
 */
class MiraFramework : public nav2::LifecycleNode
{
public:
  using ModuleMap = std::unordered_map<std::string, scitos2_core::Module::Ptr>;

  /**
   * @brief Construct a new Mira Framework object
   *
   * @param options Node options
   */
  explicit MiraFramework(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Mira Framework object
   *
   */
  ~MiraFramework();

  /**
   * @brief Build the command line arguments of the MIRA framework from the parameters of the node.
   *
   * @param port Port where MIRA listens, or 0 to leave it disabled
   * @param known_frameworks Other MIRA frameworks to connect to, as host:port
   * @param variables Variables of the MIRA configuration, as name=value
   * @param save_processed_config File where MIRA saves the resolved configuration, or empty
   * @return std::vector<std::string> The arguments, starting with the program name
   */
  static std::vector<std::string> buildMiraArguments(
    int port, const std::vector<std::string> & known_frameworks,
    const std::vector<std::string> & variables, const std::string & save_processed_config);

protected:
  /**
   * @brief Severity of the messages of MIRA to log.
   *
   * @param log_level Level set by the user (0 critical ... 5 trace), or -1 to follow the level
   * of the ROS logger of the node
   * @return mira::SeverityLevel The severity to use
   */
  mira::SeverityLevel miraSeverity(int log_level) const;

  /**
   * @brief Configures the modules plugin.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize module
   * plugin
   */
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates the modules.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates the modules.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Wait until the MIRA robot resource exists, up to the configured timeout.
   * Only warns if it does not appear.
   */
  void waitForRobotResource();

  /**
   * @brief Create a diagnostics message.
   */
  diagnostic_msgs::msg::DiagnosticArray createDiagnostics();

  /**
   * @brief Load a module plugin by type name. Virtual to allow overriding in tests.
   */
  virtual scitos2_core::Module::Ptr loadModule(const std::string & type);

  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // MIRA framework
  std::unique_ptr<mira::Framework> framework_;
  bool loaded_;
  bool started_{false};
  std::string robot_resource_{"/robot/Robot"};
  int resource_timeout_{5000};

  // Module Plugins
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
