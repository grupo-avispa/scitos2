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

#ifndef SCITOS2_MODULES__IMU_HPP_
#define SCITOS2_MODULES__IMU_HPP_

// MIRA
#include <fw/Framework.h>
#include <geometry/Point.h>

// C++
#include <memory>
#include <mutex>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// SCITOS2
#include "scitos2_core/module.hpp"

namespace scitos2_modules
{

/**
 * @class scitos2_modules::IMU
 * @brief Module for communicating with the IMU.
 *
 */
class IMU : public scitos2_core::Module
{
public:
  /**
   * @brief Construct for scitos2_modules::IMU
   */
  IMU() = default;

  /**
   * @brief Destructor for scitos2_modules::IMU
   */
  ~IMU() override = default;

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
   * @brief Callback for Acceleration data.
   *
   * @param data Acceleration data
   */
  void accelerationDataCallback(mira::ChannelRead<mira::Point3f> data);

  /**
   * @brief Callback for Gyroscope data.
   *
   * @param data Gyroscope data
   */
  void gyroscopeDataCallback(mira::ChannelRead<mira::Point3f> data);

  // MIRA Authority
  std::shared_ptr<mira::Authority> authority_;

  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("IMU")};

  std::string robot_base_frame_;
  sensor_msgs::msg::Imu imu_msg_;
  std::mutex mutex_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>>
  imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__IMU_HPP_
