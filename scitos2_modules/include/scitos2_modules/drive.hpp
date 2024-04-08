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

#ifndef SCITOS2_MODULES__DRIVE_HPP_
#define SCITOS2_MODULES__DRIVE_HPP_

// MIRA
#include <fw/Framework.h>
#include <robot/Odometry.h>

// C++
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

// SCITOS2
#include "scitos2_core/module.hpp"
#include "scitos2_msgs/msg/barrier_status.hpp"
#include "scitos2_msgs/msg/bumper_status.hpp"
#include "scitos2_msgs/msg/drive_status.hpp"
#include "scitos2_msgs/msg/emergency_stop_status.hpp"
#include "scitos2_msgs/msg/mileage.hpp"
#include "scitos2_msgs/msg/rfid_tag.hpp"
#include "scitos2_msgs/srv/change_force.hpp"
#include "scitos2_msgs/srv/emergency_stop.hpp"
#include "scitos2_msgs/srv/enable_motors.hpp"
#include "scitos2_msgs/srv/enable_rfid.hpp"
#include "scitos2_msgs/srv/reset_barrier_stop.hpp"
#include "scitos2_msgs/srv/reset_motor_stop.hpp"
#include "scitos2_msgs/srv/reset_odometry.hpp"
#include "scitos2_msgs/srv/suspend_bumper.hpp"

namespace scitos2_modules
{

uint64 MAGNETIC_BARRIER_RFID_CODE = 0xabababab;

/**
 * @class scitos2_modules::Drive
 * @brief Module for interfacing all related to drive: odometry, motor controller, state etc.
 *
 */
class Drive : public scitos2_core::Module
{
public:
  /**
   * @brief Construct for scitos2_modules::Drive
   */
  Drive() = default;

  /**
   * @brief Destructor for scitos2_modules::Drive
   */
  ~Drive() override = default;

  /**
   * @brief Configure the module.
   *
   * @param parent WeakPtr to node
   * @param name Name of plugin
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name) override;

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
   * @brief Callback executed when the odometry data is received.
   *
   * @param data Odometry data
   */
  void odometryDataCallback(mira::ChannelRead<mira::robot::Odometry2> data);

  /**
   * @brief Callback executed when the bumper data is received.
   *
   * @param data Bumper data
   */
  void bumperDataCallback(mira::ChannelRead<bool> data);

  /**
   * @brief Callback executed when the mileage data is received.
   *
   * @param data Mileage data
   */
  void mileageDataCallback(mira::ChannelRead<float> data);

  /**
   * @brief Callback executed when the drive status is received.
   *
   * @param data Drive status
   */
  void driveStatusCallback(mira::ChannelRead<uint32> data);

  /**
   * @brief Callback executed when the RFID data is received.
   *
   * @param data RFID data
   */
  void rfidStatusCallback(mira::ChannelRead<uint64> data);

  /**
   * @brief Callback executed when velocity command is received.
   *
   * @param msg Velocity command
   */
  void velocityCommandCallback(const geometry_msgs::msg::Twist & msg);

  /**
   * @brief Service to change the force.
   *
   * @param request Force change request
   * @param response Force change response
   * @return bool If the force was changed
   */
  bool changeForce(
    const std::shared_ptr<scitos2_msgs::srv::ChangeForce::Request> request,
    std::shared_ptr<scitos2_msgs::srv::ChangeForce::Response> response);

  /**
   * @brief Emergency stop service callback.
   *
   * @param request Emergency stop request
   * @param response Emergency stop response
   * @return bool If the emergency stop was successful
   */
  bool emergencyStop(
    const std::shared_ptr<scitos2_msgs::srv::EmergencyStop::Request> request,
    std::shared_ptr<scitos2_msgs::srv::EmergencyStop::Response> response);

  /**
   * @brief Enable motors service callback.
   *
   * @param request Enable motors request
   * @param response Enable motors response
   * @return bool If the motors were successfully enabled
   */
  bool enableMotors(
    const std::shared_ptr<scitos2_msgs::srv::EnableMotors::Request> request,
    std::shared_ptr<scitos2_msgs::srv::EnableMotors::Response> response);

  /**
   * @brief Enable RFID service callback.
   *
   * @param request Enable RFID request
   * @param response Enable RFID response
   * @return bool If the RFID was successfully enabled
   */
  bool enableRfid(
    const std::shared_ptr<scitos2_msgs::srv::EnableRfid::Request> request,
    std::shared_ptr<scitos2_msgs::srv::EnableRfid::Response> response);

  /**
   * @brief Reset barrier stop service callback.
   *
   * @param request Reset barrier stop request
   * @param response Reset barrier stop response
   * @return bool If the barrier stop was successfully reset
   */
  bool resetBarrierStop(
    const std::shared_ptr<scitos2_msgs::srv::ResetBarrierStop::Request> request,
    std::shared_ptr<scitos2_msgs::srv::ResetBarrierStop::Response> response);

  /**
   * @brief Reset motor stop service callback.
   *
   * @param request Reset motor stop request
   * @param response Reset motor stop response
   * @return bool If the motor stop was successfully reset
   */
  bool resetMotorStop(
    const std::shared_ptr<scitos2_msgs::srv::ResetMotorStop::Request> request,
    std::shared_ptr<scitos2_msgs::srv::ResetMotorStop::Response> response);

  /**
   * @brief Reset odometry service callback.
   *
   * @param request Reset odometry request
   * @param response Reset odometry response
   * @return bool If the odometry was successfully reset
   */
  bool resetOdometry(
    const std::shared_ptr<scitos2_msgs::srv::ResetOdometry::Request> request,
    std::shared_ptr<scitos2_msgs::srv::ResetOdometry::Response> response);

  /**
   * @brief Suspend bumper service callback.
   *
   * @param request Suspend bumper request
   * @param response Suspend bumper response
   * @return bool If the bumper was successfully suspended
   */
  bool suspendBumper(
    const std::shared_ptr<scitos2_msgs::srv::SuspendBumper::Request> request,
    std::shared_ptr<scitos2_msgs::srv::SuspendBumper::Response> response);

  /**
   * @brief Check if the emergency stop button is released.
   *
   * @param msg EmergencyStopStatus status message
   * @return bool If the emergency stop button is released
   */
  bool isEmergencyStopReleased(scitos2_msgs::msg::EmergencyStopStatus msg);

  /**
   * @brief Check if the code is a barrier code.
   *
   * @param code Rfid code
   * @return bool If the code is a barrier code
   */
  bool isBarrierCode(uint64 code) {return code == MAGNETIC_BARRIER_RFID_CODE;}

  /**
   * @brief Reset tje motor stop if the bumper is activated after a certain time.
   *
   * @param current_time
   */
  void resetMotorStopAfterTimeout(rclcpp::Time current_time);

  /**
   * @brief Create the bumper markers.
   *
   */
  visualization_msgs::msg::MarkerArray createBumperMarkers();

  /**
   * @brief Callback executed when a parameter change is detected.
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Convert MIRA Odometry2 to ROS Odometry.
   *
   * @param odometry Odometry from MIRA
   * @param timestamp Timestamp of the state
   * @return nav_msgs::msg::Odometry Odometry for ROS
   */
  nav_msgs::msg::Odometry miraToRosOdometry(
    const mira::robot::Odometry2 & odometry, const mira::Time & timestamp);

  /**
   * @brief Convert MIRA Odometry2 to ROS TF.
   *
   * @param odometry Odometry from MIRA
   * @param timestamp Timestamp of the state
   * @return geometry_msgs::msg::TransformStamped Transform for ROS
   */
  geometry_msgs::msg::TransformStamped miraToRosTf(
    const mira::robot::Odometry2 & odometry, const mira::Time & timestamp);

  /**
   * @brief Convert MIRA DriveStatus to ROS DriveStatus.
   *
   * @param status DriveStatus from MIRA
   * @param timestamp Timestamp of the state
   * @return scitos2_msgs::msg::DriveStatus DriveStatus for ROS
   */
  scitos2_msgs::msg::DriveStatus miraToRosDriveStatus(
    const uint32 & status, const mira::Time & timestamp);

  /**
   * @brief Convert MIRA DriveStatus to ROS EmergencyStopStatus.
   *
   * @param status EmergencyStopStatus from MIRA
   * @param timestamp Timestamp of the state
   * @return scitos2_msgs::msg::EmergencyStopStatus EmergencyStopStatus for ROS
   */
  scitos2_msgs::msg::EmergencyStopStatus miraToRosEmergencyStopStatus(
    const uint32 & status, const mira::Time & timestamp);

  /**
   * @brief Convert MIRA BarrierStatus to ROS BarrierStatus.
   *
   * @param status BarrierStatus from MIRA
   * @param timestamp Timestamp of the state
   * @return scitos2_msgs::msg::BarrierStatus BarrierStatus for ROS
   */
  scitos2_msgs::msg::BarrierStatus miraToRosBarrierStatus(
    const uint64 & status, const mira::Time & timestamp);

  // MIRA Authority
  std::shared_ptr<mira::Authority> authority_;

  // Plugin related
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("Drive")};

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // ROS Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::BumperStatus>>
  bumper_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  bumper_markers_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::DriveStatus>>
  drive_status_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::EmergencyStopStatus>>
  emergency_stop_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::BarrierStatus>>
  magnetic_barrier_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::Mileage>> mileage_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odometry_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::RfidTag>> rfid_pub_;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;

  // ROS Services
  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::ChangeForce>> change_force_service_;
  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::EmergencyStop>> emergency_stop_service_;
  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::EnableMotors>> enable_motors_service_;
  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::EnableRfid>> enable_rfid_service_;
  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::ResetBarrierStop>> reset_barrier_stop_service_;
  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::ResetMotorStop>> reset_motor_stop_service_;
  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::ResetOdometry>> reset_odometry_service_;
  std::shared_ptr<rclcpp::Service<scitos2_msgs::srv::SuspendBumper>> suspend_bumper_service_;

  std::string base_frame_;
  bool emergency_stop_activated_;
  scitos2_msgs::msg::BarrierStatus barrier_status_;
  bool is_active_;

  // Bumper
  bool bumper_activated_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_bumper_reset_;
  rclcpp::Duration reset_bumper_interval_{0, 0};

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__DRIVE_HPP_
