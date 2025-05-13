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

#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "opennav_docking/utils.hpp"
#include "scitos2_charging_dock/charging_dock.hpp"
#include "scitos2_charging_dock/segmentation.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace scitos2_charging_dock
{

void ChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;
  is_charging_ = false;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Parameters for optional external detection of dock pose
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_timeout", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_x", rclcpp::ParameterValue(-0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_pitch", rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_roll", rclcpp::ParameterValue(-1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".filter_coef", rclcpp::ParameterValue(0.1));

  // This is how close robot should get to pose
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold", rclcpp::ParameterValue(0.05));

  // Staging pose configuration
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_x_offset", rclcpp::ParameterValue(-0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));

  node_->get_parameter(name + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name + ".external_detection_translation_y", external_detection_translation_y_);
  double yaw, pitch, roll;
  node_->get_parameter(name + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name + ".external_detection_rotation_roll", roll);
  external_detection_rotation_.setEuler(pitch, roll, yaw);
  node_->get_parameter(name + ".docking_threshold", docking_threshold_);
  node_->get_parameter(name + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name + ".staging_yaw_offset", staging_yaw_offset_);

  dock_direction_ = opennav_docking_core::DockDirection::FORWARD;

  // Setup perception
  perception_ = std::make_unique<Perception>(node_, name, tf2_buffer_);

  // Setup filter
  double filter_coef;
  node_->get_parameter(name + ".filter_coef", filter_coef);
  filter_ = std::make_unique<opennav_docking::PoseFilter>(filter_coef, external_detection_timeout_);

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery", 1,
    [this](
      const sensor_msgs::msg::BatteryState::SharedPtr state) {
      is_charging_ =
      (state->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
    });

  dock_pose_.header.stamp = rclcpp::Time(0);
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
      scan_ = *scan;
    });

  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
  filtered_dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "filtered_dock_pose", 1);
  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);
}

geometry_msgs::msg::PoseStamped ChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  // Send the initial estimate to the perception module
  perception_->setInitialEstimate(pose, frame);

  // Compute the staging pose with given offsets
  const double yaw = tf2::getYaw(pose.orientation);
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose.header.frame_id = frame;
  staging_pose.header.stamp = node_->now();
  staging_pose.pose = pose;
  staging_pose.pose.position.x += cos(yaw) * staging_x_offset_;
  staging_pose.pose.position.y += sin(yaw) * staging_x_offset_;
  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, yaw + staging_yaw_offset_);
  staging_pose.pose.orientation = tf2::toMsg(orientation);

  // Publish staging pose for debugging purposes
  staging_pose_pub_->publish(staging_pose);
  return staging_pose;
}

bool ChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string /*id*/)
{
  // Get current detections, transform to frame, and apply offsets
  geometry_msgs::msg::PoseStamped detected = perception_->getDockPose(scan_);

  // Validate that external pose is new enough
  auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (node_->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(node_->get_logger(), "Lost detection or did not detect: timeout exceeded");
    return false;
  }

  // Transform detected pose into fixed frame. Note that the argument pose
  // is the output of detection, but also acts as the initial estimate
  // and contains the frame_id of docking
  if (detected.header.frame_id != pose.header.frame_id) {
    try {
      if (!tf2_buffer_->canTransform(
          pose.header.frame_id, detected.header.frame_id,
          detected.header.stamp, rclcpp::Duration::from_seconds(0.2)))
      {
        RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
        return false;
      }
      tf2_buffer_->transform(detected, detected, pose.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
      return false;
    }
  }

  // Filter the detected pose
  detected = filter_->update(detected);
  filtered_dock_pose_pub_->publish(detected);

  // Rotate the just the orientation, then remove roll/pitch
  geometry_msgs::msg::PoseStamped just_orientation;
  just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation = detected.pose.orientation;
  tf2::doTransform(just_orientation, just_orientation, transform);

  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, tf2::getYaw(just_orientation.pose.orientation));
  dock_pose_.pose.orientation = tf2::toMsg(orientation);

  // Construct dock_pose_ by applying translation/rotation
  dock_pose_.header = detected.header;
  dock_pose_.pose.position = detected.pose.position;
  const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
  dock_pose_.pose.position.x += cos(yaw) * external_detection_translation_x_ -
    sin(yaw) * external_detection_translation_y_;
  dock_pose_.pose.position.y += sin(yaw) * external_detection_translation_x_ +
    cos(yaw) * external_detection_translation_y_;
  dock_pose_.pose.position.z = 0.0;

  // Publish & return dock pose for debugging purposes
  dock_pose_pub_->publish(dock_pose_);
  pose = dock_pose_;
  return true;
}

bool ChargingDock::isCharging()
{
  return is_charging_;
}

bool ChargingDock::isDocked()
{
  return is_charging_;
}

bool ChargingDock::disableCharging()
{
  return true;
}

bool ChargingDock::hasStoppedCharging()
{
  return !isCharging();
}

}  // namespace scitos2_charging_dock

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(scitos2_charging_dock::ChargingDock, opennav_docking_core::ChargingDock)
