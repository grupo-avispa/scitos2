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

// C++
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "scitos2_modules/imu.hpp"

namespace scitos2_modules
{

using std::placeholders::_1;
using std::placeholders::_2;

void IMU::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
  // Declare and read parameters
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  logger_ = node->get_logger();
  authority_ = std::make_shared<mira::Authority>();
  authority_->checkin("/", plugin_name_);

  // Declare and read parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_base_frame",
    rclcpp::ParameterValue("base_link"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the base frame of the robot"));
  node->get_parameter(plugin_name_ + ".robot_base_frame", robot_base_frame_);
  RCLCPP_INFO(logger_, "The parameter robot_base_frame is set to: [%s]", robot_base_frame_.c_str());

  // Create ROS publishers
  imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", 1);

  // Create MIRA subscribers
  authority_->subscribe<mira::Point3f>(
    "/robot/Acceleration", std::bind(&IMU::accelerationDataCallback, this, _1));
  authority_->subscribe<mira::Point3f>(
    "/robot/Gyroscope", std::bind(&IMU::gyroscopeDataCallback, this, _1));

  RCLCPP_INFO(logger_, "Configured module : %s", plugin_name_.c_str());

  // Initialize the IMU message
  imu_msg_.header.frame_id = robot_base_frame_;
  imu_msg_.orientation_covariance[0] = -1;
  imu_msg_.angular_velocity_covariance[0] = -1;

  timer_ = node->create_wall_timer(
    std::chrono::milliseconds(10), [this]() {
      std::lock_guard<std::mutex> lock_data(mutex_);
      imu_pub_->publish(imu_msg_);
    });
}

void IMU::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type scitos2_module::Imu", plugin_name_.c_str());
  authority_.reset();
  imu_pub_.reset();
  timer_.reset();
}

void IMU::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type scitos2_module::IMU", plugin_name_.c_str());
  imu_pub_->on_activate();
  authority_->start();
}

void IMU::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type scitos2_module::IMU", plugin_name_.c_str());
  authority_->checkout();
  imu_pub_->on_deactivate();
  timer_.reset();
}

void IMU::accelerationDataCallback(mira::ChannelRead<mira::Point3f> data)
{
  std::lock_guard<std::mutex> lock_data(mutex_);
  imu_msg_.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
  // TODO(ajtudela): Check if the units are correct
  imu_msg_.linear_acceleration.x = data->x();
  imu_msg_.linear_acceleration.y = data->y();
  imu_msg_.linear_acceleration.z = data->z();
}

void IMU::gyroscopeDataCallback(mira::ChannelRead<mira::Point3f> data)
{
  std::lock_guard<std::mutex> lock_data(mutex_);
  imu_msg_.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
  // TODO(ajtudela): Check if the units are correct
  imu_msg_.angular_velocity.x = data->x();
  imu_msg_.angular_velocity.y = data->y();
  imu_msg_.angular_velocity.z = data->z();
}

}  // namespace scitos2_modules

PLUGINLIB_EXPORT_CLASS(scitos2_modules::IMU, scitos2_core::Module)
