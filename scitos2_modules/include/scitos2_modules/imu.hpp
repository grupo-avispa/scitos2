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
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

// ROS
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// SCITOS2
#include "scitos2_mira_utils/mira_module.hpp"

namespace scitos2_modules
{

/**
 * @class scitos2_modules::IMU
 * @brief Module for communicating with the IMU.
 *
 */
class IMU : public scitos2_mira_utils::MiraModule
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
  bool activate() override;

  /**
   * @brief Deactivate the module state machine.
   */
  bool deactivate() override;

  /**
   * @brief Add the status of the module to the diagnostics of the node.
   *
   * @param array The diagnostic array to add the status to
   */
  void diagnostics(diagnostic_msgs::msg::DiagnosticArray & array) override;

protected:
  /**
   * @brief Callback for Acceleration data. The units and ranges are +- 2g.
   *
   * @param data Acceleration data
   */
  void accelerationDataCallback(mira::ChannelRead<mira::Point3f> data);

  /**
   * @brief Callback for Gyroscope data. The units and ranges are +- 250 degree/sec.
   *
   * @param data Gyroscope data
   */
  void gyroscopeDataCallback(mira::ChannelRead<mira::Point3f> data);

  /**
   * @brief Convert MIRA Acceleration Point3f to ROS Vector3.
   *
   * @param acceleration Acceleration data from MIRA
   * @return geometry_msgs::msg::Vector3 Acceleration data for ROS
   */
  geometry_msgs::msg::Vector3 miraToRosAcceleration(const mira::Point3f & acceleration);

  /**
   * @brief Convert MIRA Gyroscope Point3f to ROS Vector3.
   *
   * @param gyroscope Gyroscope data from MIRA
   * @return geometry_msgs::msg::Vector3 Gyroscope data for ROS
   */
  geometry_msgs::msg::Vector3 miraToRosGyroscope(const mira::Point3f & gyroscope);

  /**
   * @brief Publish the IMU message once a new acceleration and a new angular velocity have
   * arrived since the last one. The caller must hold mutex_.
   */
  void publishIfComplete();

  /**
   * @brief Fill the diagnostic status of the IMU.
   *
   * @param stat Status to fill
   */
  void fillDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);


  std::string robot_base_frame_, imu_topic_;
  sensor_msgs::msg::Imu imu_msg_;
  // Guards imu_msg_ and the fields below, written from the two MIRA data callbacks
  std::mutex mutex_;
  std::atomic<bool> is_active_{false};
  bool has_acceleration_{false};
  bool has_gyroscope_{false};
  int64_t latest_stamp_ns_{0};
  std::chrono::steady_clock::time_point last_publish_time_;
  uint64_t published_samples_{0};

  // Time without data from MIRA after which the diagnostics report an error
  static constexpr std::chrono::seconds kDataTimeout{2};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>>
  imu_pub_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__IMU_HPP_
