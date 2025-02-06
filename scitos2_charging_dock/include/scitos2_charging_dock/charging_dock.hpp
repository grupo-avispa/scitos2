// Copyright (c) 2024 Open Navigation LLC
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

#ifndef SCITOS2_CHARGING_DOCK__CHARGING_DOCK_HPP_
#define SCITOS2_CHARGING_DOCK__CHARGING_DOCK_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "opennav_docking_core/charging_dock.hpp"
#include "opennav_docking/pose_filter.hpp"
#include "scitos2_charging_dock/perception.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

namespace scitos2_charging_dock
{

class ChargingDock : public opennav_docking_core::ChargingDock
{
public:
  /**
   * @brief Constructor
   */
  ChargingDock()
  : opennav_docking_core::ChargingDock()
  {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() {}

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() {}

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate() {}

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock with pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  virtual geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame);

  /**
   * @brief Method to obtain the refined pose of the dock, based on sensor
   * @param pose The initial estimate of the dock pose.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id);

  /**
   * @copydoc opennav_docking_core::ChargingDock::isCharging
   */
  virtual bool isCharging();

  /**
   * @copydoc opennav_docking_core::ChargingDock::isDocked
   */
  virtual bool isDocked();

  /**
   * @copydoc opennav_docking_core::ChargingDock::disableCharging
   */
  virtual bool disableCharging();

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  virtual bool hasStoppedCharging();

protected:
  // Subscribe to a scan topic
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_dock_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_pub_;
  // It will contain latest message
  sensor_msgs::msg::LaserScan scan_;
  // This is the actual dock pose once it has the specified translation/rotation applied
  geometry_msgs::msg::PoseStamped dock_pose_;

  // Subscribe to battery message, used to determine if charging
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  bool is_charging_;

  // An external reference (sensor_msgs::LaserScan) is used to detect dock
  double external_detection_timeout_;
  tf2::Quaternion external_detection_rotation_;
  double external_detection_translation_x_;
  double external_detection_translation_y_;

  // Filtering of detected poses
  std::shared_ptr<opennav_docking::PoseFilter> filter_;

  // External pose reference, this is the distance threshold
  double docking_threshold_;
  // Offset for staging pose relative to dock pose
  double staging_x_offset_;
  double staging_yaw_offset_;

  // Perception
  std::unique_ptr<Perception> perception_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};

}  // namespace scitos2_charging_dock

#endif  // SCITOS2_CHARGING_DOCK__CHARGING_DOCK_HPP_
