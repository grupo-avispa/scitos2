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

#ifndef SCITOS2_CHARGING_DOCK__DOCK_SAVER_HPP_
#define SCITOS2_CHARGING_DOCK__DOCK_SAVER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "scitos2_msgs/srv/save_dock.hpp"
#include "scitos2_msgs/srv/change_force.hpp"
#include "scitos2_charging_dock/perception.hpp"

namespace scitos2_charging_dock
{

/**
 * @class scitos2_charging_dock::DockSaver
 * @brief A class that provides dock saving methods and services
 */
class DockSaver : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the scitos2_charging_dock::DockSaver
   * @param options Additional options to control creation of the node.
   */
  explicit DockSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the scitos2_charging_dock::DockServer
   */
  ~DockSaver();

  /**
   * @brief Sets up dock saving service
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when node switched to active state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when node switched to inactive state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when it is required node clean-up
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback for saving the dock to a PCD file.
   *
   * @param request SaveDock service request
   * @param response SaveDock service response
   * @return bool True if the dock was saved successfully
   */
  bool saveDockCallback(
    const std::shared_ptr<scitos2_msgs::srv::SaveDock::Request> request,
    std::shared_ptr<scitos2_msgs::srv::SaveDock::Response> response);

protected:
  // The timeout for saving the dock in service
  std::shared_ptr<rclcpp::Duration> save_dock_timeout_;

  // Perception module to interact with scans and point clouds
  std::shared_ptr<Perception> perception_;

  // The name of the service for saving a dock from topic
  const std::string save_dock_service_name_{"save_dock"};
  // A service to save the dock to a file at run time (SaveDock)
  rclcpp::Service<scitos2_msgs::srv::SaveDock>::SharedPtr save_dock_service_;
};

}  // namespace scitos2_charging_dock

#endif  // SCITOS2_CHARGING_DOCK__DOCK_SAVER_HPP_
