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

// TF2
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <string>
#include <chrono>
#include <thread>
#include <utility>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "scitos2_modules/drive.hpp"

namespace scitos2_modules
{

namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

// Flags of the drive status that are not part of a normal operation, with the level that they
// deserve. The rest of the flags (modes, laser fields...) are only reported as values
struct DriveStatusFlag
{
  DriveStatusBit bit;
  const char * name;
  uint8_t level;
};

const std::array<DriveStatusFlag, 20> kDriveStatusFlags = {{
  {DriveStatusBit::EMERGENCY_STOP_STATUS, "emergency_stop_status", DiagnosticStatus::ERROR},
  {DriveStatusBit::ERROR_SIFAS_COMMUNICATION, "error_sifas_communication", DiagnosticStatus::ERROR},
  {DriveStatusBit::ERROR_SIFAS_INTERNAL, "error_sifas_internal", DiagnosticStatus::ERROR},
  {DriveStatusBit::ERROR_MOTOR_LEFT, "error_motor_left", DiagnosticStatus::ERROR},
  {DriveStatusBit::ERROR_MOTOR_RIGHT, "error_motor_right", DiagnosticStatus::ERROR},
  {DriveStatusBit::ERROR_FIRMARE, "error_firmare", DiagnosticStatus::ERROR},
  {DriveStatusBit::RELAY_ACTIVE, "relay_active", DiagnosticStatus::ERROR},
  {DriveStatusBit::MODE_FORCED_STOPPED, "mode_forced_stopped", DiagnosticStatus::WARN},
  {DriveStatusBit::BUMPER_FRONT_STATUS, "bumper_front_status", DiagnosticStatus::WARN},
  {DriveStatusBit::BUMPER_REAR_STATUS, "bumper_rear_status", DiagnosticStatus::WARN},
  {DriveStatusBit::SOFTWARE_SAFE_STOP, "software_safe_stop", DiagnosticStatus::WARN},
  {DriveStatusBit::ERROR_BATTERY_LOW, "error_battery_low", DiagnosticStatus::WARN},
  {DriveStatusBit::ERROR_TEMPERATURE_HIGH, "error_temperature_high", DiagnosticStatus::WARN},
  {DriveStatusBit::TEMPERATURE_WARNING, "temperature_warning", DiagnosticStatus::WARN},
  {DriveStatusBit::ERROR_STALL_MODE, "error_stall_mode", DiagnosticStatus::WARN},
  {DriveStatusBit::TRACTION_LOSS, "traction_loss", DiagnosticStatus::WARN},
  {DriveStatusBit::ERROR_ROTATION_TOO_HIGH, "error_rotation_too_high", DiagnosticStatus::WARN},
  {DriveStatusBit::ERROR_MOTOR_ACCELERATION, "error_motor_acceleration", DiagnosticStatus::WARN},
  {DriveStatusBit::LIMITED_PWM_LEFT, "limited_pwm_left", DiagnosticStatus::WARN},
  {DriveStatusBit::LIMITED_PWM_RIGHT, "limited_pwm_right", DiagnosticStatus::WARN},
}};

}  // namespace

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;

void Drive::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
  auto node = configureMira(parent, name);
  is_active_ = false;
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mira_drive_module",
    rclcpp::ParameterValue("auto"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "MIRA module that holds the drive properties (e.g. MainControlUnit or MotorController), "
      "or 'auto' to detect it from the robot"));
  node->get_parameter(plugin_name_ + ".mira_drive_module", drive_module_);
  RCLCPP_INFO(
    logger_, "The parameter mira_drive_module is set to: [%s]", drive_module_.c_str());

  int detect_timeout = 3000;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mira_drive_detect_timeout",
    rclcpp::ParameterValue(detect_timeout), rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "Maximum time in ms to wait for the robot to expose the drive properties when "
      "mira_drive_module is 'auto'"));
  node->get_parameter(plugin_name_ + ".mira_drive_detect_timeout", detect_timeout);
  drive_detect_timeout_ = std::chrono::milliseconds(std::max(detect_timeout, 0));

  // Declare and read parameters
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_base_frame",
    rclcpp::ParameterValue("base_link"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the base frame of the robot"));
  node->get_parameter(plugin_name_ + ".robot_base_frame", robot_base_frame_);
  RCLCPP_INFO(logger_, "The parameter robot_base_frame is set to: [%s]", robot_base_frame_.c_str());

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".odom_frame",
    rclcpp::ParameterValue("odom"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the odometry frame"));
  node->get_parameter(plugin_name_ + ".odom_frame", odom_frame_);
  RCLCPP_INFO(logger_, "The parameter odom_frame is set to: [%s]", odom_frame_.c_str());

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".odom_topic",
    rclcpp::ParameterValue("odom"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the odometry topic"));
  node->get_parameter(plugin_name_ + ".odom_topic", odom_topic_);
  RCLCPP_INFO(logger_, "The parameter odom_topic is set to: [%s]", odom_topic_.c_str());

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".magnetic_barrier_enabled",
    rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable the magnetic strip detector to cut out the motors"));
  node->get_parameter(plugin_name_ + ".magnetic_barrier_enabled", magnetic_barrier_enabled_);
  RCLCPP_INFO(
    logger_, "The parameter magnetic_barrier_enabled is set to: [%s]",
    magnetic_barrier_enabled_ ? "true" : "false");

  double cmd_vel_max_age = cmd_vel_max_age_;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cmd_vel_max_age", rclcpp::ParameterValue(cmd_vel_max_age),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "Seconds after which a cmd_vel is discarded, using its header stamp. "
      "0 disables it, and commands without a stamp are never discarded"));
  node->get_parameter(plugin_name_ + ".cmd_vel_max_age", cmd_vel_max_age);
  cmd_vel_max_age_ = std::max(cmd_vel_max_age, 0.0);
  RCLCPP_INFO(logger_, "The parameter cmd_vel_max_age is set to: [%.2f]", cmd_vel_max_age_);

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_tf",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Publish the odometry as a tf2 transform"));
  node->get_parameter(plugin_name_ + ".publish_tf", publish_tf_);
  RCLCPP_INFO(
    logger_, "The parameter publish_tf_ is set to: [%s]", publish_tf_ ? "true" : "false");

  int rbi = 0;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reset_bumper_interval",
    rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "The interval in milliseconds to reset the bumper. A value <= 0 disables the "
      "automatic reset"));
  node->get_parameter(plugin_name_ + ".reset_bumper_interval", rbi);
  RCLCPP_INFO(logger_, "The parameter reset_bumper_interval is set to: [%i]", rbi);
  reset_bumper_interval_ = rclcpp::Duration::from_seconds(rbi / 1000.0);

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".footprint",
    rclcpp::ParameterValue(""), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The footprint of the robot"));
  node->get_parameter(plugin_name_ + ".footprint", footprint_);
  RCLCPP_INFO(logger_, "The parameter footprint is set to: [%s]", footprint_.c_str());

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_radius",
    rclcpp::ParameterValue(0.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The radius of the robot"));
  node->get_parameter(plugin_name_ + ".robot_radius", robot_radius_);
  RCLCPP_INFO(logger_, "The parameter robot_radius is set to: [%f]", robot_radius_);

  // If the footprint has been specified and is valid, use it; otherwise fall back to the radius
  if (footprint_ == "" || footprint_ == "[]" ||
    !nav2_costmap_2d::makeFootprintFromString(footprint_, unpadded_footprint_))
  {
    if (footprint_ != "" && footprint_ != "[]") {
      RCLCPP_ERROR(
        logger_, "The footprint parameter is invalid: \"%s\", using radius (%lf) instead",
        footprint_.c_str(), robot_radius_);
    }
    unpadded_footprint_ = nav2_costmap_2d::makeFootprintFromRadius(robot_radius_);
  }

  // Create ROS publishers
  auto latched_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  bumper_pub_ = node->create_publisher<scitos2_msgs::msg::BumperStatus>(
    "bumper", latched_profile);
  bumper_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "bumper/visualization", rclcpp::SystemDefaultsQoS());
  drive_status_pub_ = node->create_publisher<scitos2_msgs::msg::DriveStatus>(
    "drive_status", rclcpp::SystemDefaultsQoS());
  emergency_stop_pub_ = node->create_publisher<scitos2_msgs::msg::EmergencyStopStatus>(
    "emergency_stop_status", latched_profile);
  magnetic_barrier_pub_ = node->create_publisher<scitos2_msgs::msg::BarrierStatus>(
    "barrier_status", latched_profile);
  mileage_pub_ = node->create_publisher<scitos2_msgs::msg::Mileage>(
    "mileage", rclcpp::SystemDefaultsQoS());
  odometry_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS());
  rfid_pub_ = node->create_publisher<scitos2_msgs::msg::RfidTag>(
    "rfid", rclcpp::SystemDefaultsQoS());
  cmd_vel_applied_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_applied", rclcpp::SystemDefaultsQoS());

  // Create MIRA subscribers
  // The odometry is the fastest channel, so it gets its own thread: a slow callback of the
  // authority (status, RFID...) must not delay it
  authority_->subscribe<mira::robot::Odometry2>(
    "/robot/Odometry", std::bind(&Drive::odometryDataCallback, this, _1), true);
  authority_->subscribe<bool>(
    "/robot/Bumper", std::bind(&Drive::bumperDataCallback, this, _1));
  authority_->subscribe<float>(
    "/robot/Mileage", std::bind(&Drive::mileageDataCallback, this, _1));
  authority_->subscribe<uint32>(
    "/robot/DriveStatusPlain", std::bind(&Drive::driveStatusCallback, this, _1));
  authority_->subscribe<uint64>(
    "/robot/RFIDUserTag", std::bind(&Drive::rfidStatusCallback, this, _1));
  authority_->subscribe<mira::Velocity2>(
    "/robot/TargetVelocity", std::bind(&Drive::targetVelocityCallback, this, _1));

  // Create ROS subscribers
  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&Drive::velocityCommandCallback, this, _1));

  // Create ROS services
  change_force_service_ = node->create_service<scitos2_msgs::srv::ChangeForce>(
    "drive/change_force", std::bind(&Drive::changeForce, this, _1, _2));
  emergency_stop_service_ = node->create_service<scitos2_msgs::srv::EmergencyStop>(
    "drive/emergency_stop", std::bind(&Drive::emergencyStop, this, _1, _2));
  enable_motors_service_ = node->create_service<scitos2_msgs::srv::EnableMotors>(
    "drive/enable_motors", std::bind(&Drive::enableMotors, this, _1, _2));
  enable_rfid_service_ = node->create_service<scitos2_msgs::srv::EnableRfid>(
    "drive/enable_rfid", std::bind(&Drive::enableRfid, this, _1, _2));
  reset_barrier_stop_service_ = node->create_service<scitos2_msgs::srv::ResetBarrierStop>(
    "drive/reset_barrier_stop", std::bind(&Drive::resetBarrierStop, this, _1, _2));
  reset_motor_stop_service_ = node->create_service<scitos2_msgs::srv::ResetMotorStop>(
    "drive/reset_motor_stop", std::bind(&Drive::resetMotorStop, this, _1, _2));
  reset_odometry_service_ = node->create_service<scitos2_msgs::srv::ResetOdometry>(
    "drive/reset_odometry", std::bind(&Drive::resetOdometry, this, _1, _2));
  suspend_bumper_service_ = node->create_service<scitos2_msgs::srv::SuspendBumper>(
    "drive/suspend_bumper", std::bind(&Drive::suspendBumper, this, _1, _2));

  // Callback for monitor changes in parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Drive::dynamicParametersCallback, this, _1));

  // Initialize the transform broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }

  // Initialize the bumper and emergency stop status
  bumper_activated_ = false;
  emergency_stop_activated_ = false;
}

void Drive::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type scitos2_module::Drive", plugin_name_.c_str());
  cleanupMira();
  bumper_pub_.reset();
  bumper_markers_pub_.reset();
  drive_status_pub_.reset();
  emergency_stop_pub_.reset();
  magnetic_barrier_pub_.reset();
  mileage_pub_.reset();
  odometry_pub_.reset();
  rfid_pub_.reset();
  cmd_vel_applied_pub_.reset();
  change_force_service_.reset();
  emergency_stop_service_.reset();
  enable_motors_service_.reset();
  enable_rfid_service_.reset();
  reset_barrier_stop_service_.reset();
  reset_motor_stop_service_.reset();
  reset_odometry_service_.reset();
  suspend_bumper_service_.reset();
  tf_broadcaster_.reset();
}

bool Drive::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type scitos2_module::Drive", plugin_name_.c_str());
  bumper_pub_->on_activate();
  bumper_markers_pub_->on_activate();
  drive_status_pub_->on_activate();
  emergency_stop_pub_->on_activate();
  magnetic_barrier_pub_->on_activate();
  mileage_pub_->on_activate();
  odometry_pub_->on_activate();
  rfid_pub_->on_activate();
  cmd_vel_applied_pub_->on_activate();

  if (!startMira()) {
    return false;
  }
  is_active_ = true;

  // MIRA parameters can only be written once the authority has started. Not fatal to
  // activation: the drive still works, just without the magnetic barrier configured.
  // The MotorController boards (e.g. the Morphia) have no magnetic barrier, so there is nothing
  // to switch off there
  resolveDriveModule(drive_detect_timeout_);
  if (magnetic_barrier_enabled_ || drive_module_ != "MotorController") {
    if (!setDriveParam("RearLaser.Enabled", magnetic_barrier_enabled_ ? "true" : "false")) {
      RCLCPP_ERROR(logger_, "Failed to set the magnetic_barrier_enabled MIRA parameter");
    }
  }

  return true;
}

bool Drive::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type scitos2_module::Drive", plugin_name_.c_str());
  // The MIRA authority stays checked in (see cleanup()), so the module can be activated again
  is_active_ = false;
  bumper_pub_->on_deactivate();
  bumper_markers_pub_->on_deactivate();
  drive_status_pub_->on_deactivate();
  emergency_stop_pub_->on_deactivate();
  magnetic_barrier_pub_->on_deactivate();
  mileage_pub_->on_deactivate();
  odometry_pub_->on_deactivate();
  rfid_pub_->on_deactivate();
  cmd_vel_applied_pub_->on_deactivate();
  return true;
}

rcl_interfaces::msg::SetParametersResult Drive::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".magnetic_barrier_enabled") {
        magnetic_barrier_enabled_ = parameter.as_bool();
        setDriveParam("RearLaser.Enabled", magnetic_barrier_enabled_ ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter magnetic_barrier_enabled is set to: [%s]",
          magnetic_barrier_enabled_ ? "true" : "false");
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".robot_base_frame") {
        if (parameter.as_string().empty()) {
          result.successful = false;
          result.reason = "robot_base_frame must not be empty";
          return result;
        }
        robot_base_frame_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter base_frame is set to: [%s]", robot_base_frame_.c_str());
      } else if (name == plugin_name_ + ".odom_frame") {
        if (parameter.as_string().empty()) {
          result.successful = false;
          result.reason = "odom_frame must not be empty";
          return result;
        }
        odom_frame_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter odom_frame is set to: [%s]", odom_frame_.c_str());
      } else if (name == plugin_name_ + ".odom_topic") {
        if (parameter.as_string().empty()) {
          result.successful = false;
          result.reason = "odom_topic must not be empty";
          return result;
        }
        odom_topic_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter odom_topic is set to: [%s]", odom_topic_.c_str());
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".reset_bumper_interval") {
        int rbi = parameter.as_int();
        reset_bumper_interval_ = rclcpp::Duration::from_seconds(rbi / 1000.0);
        RCLCPP_INFO(logger_, "The parameter reset_bumper_interval is set to: [%i]", rbi);
      }
    }
  }

  result.successful = true;
  return result;
}

void Drive::odometryDataCallback(mira::ChannelRead<mira::robot::Odometry2> data)
{
  if (!is_active_) {
    return;
  }

  auto odom_msg = miraToRosOdometry(data->value(), data->timestamp);
  odometry_pub_->publish(odom_msg);

  // Publish the TF
  if (publish_tf_) {
    auto tf_msg = miraToRosTf(data->value(), data->timestamp);
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void Drive::bumperDataCallback(mira::ChannelRead<bool> data)
{
  if (!is_active_) {
    return;
  }

  rclcpp::Time stamp = rclcpp::Time(data->timestamp.toUnixNS());

  scitos2_msgs::msg::BumperStatus bumper_status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bumper_status.header.frame_id = robot_base_frame_;
  }
  bumper_status.header.stamp = stamp;
  bumper_status.bumper_activated = data->value();
  bumper_status.bumper_status = data->value();
  bumper_pub_->publish(bumper_status);
  bumper_markers_pub_->publish(createBumperMarkers(bumper_status.header));

  bumper_activated_ = bumper_status.bumper_activated;

  resetMotorStopAfterTimeout(stamp);
}

void Drive::mileageDataCallback(mira::ChannelRead<float> data)
{
  if (!is_active_) {
    return;
  }

  scitos2_msgs::msg::Mileage mileage_msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mileage_msg.header.frame_id = robot_base_frame_;
  }
  mileage_msg.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
  mileage_msg.distance = data->value();
  mileage_pub_->publish(mileage_msg);
}

void Drive::driveStatusCallback(mira::ChannelRead<uint32> data)
{
  if (!is_active_) {
    return;
  }

  auto drive_status_msg = miraToRosDriveStatus(data->value(), data->timestamp);
  auto emergency_stop_msg = miraToRosEmergencyStopStatus(data->value(), data->timestamp);

  RCLCPP_DEBUG_STREAM(
    logger_, "Drive controller status "
      << "(Nor: " << drive_status_msg.mode_normal
      << ", MStop: " << drive_status_msg.mode_forced_stopped
      << ", FreeRun: " << drive_status_msg.mode_freerun
      << ", EmBut: " << drive_status_msg.emergency_stop_activated
      << ", BumpPres: " << drive_status_msg.bumper_front_activated
      << ", BusErr: " << drive_status_msg.error_sifas_communication
      << ", Stall: " << drive_status_msg.error_stall_mode
      << ", InterErr: " << drive_status_msg.error_sifas_internal << ")");

  drive_status_pub_->publish(drive_status_msg);
  emergency_stop_pub_->publish(emergency_stop_msg);

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_status_ = data->value();
    last_status_time_ = std::chrono::steady_clock::now();
  }

  // Commands are not forwarded during the emergency stop, so the last velocity would stay
  // in the robot: tell it to stop once, when the emergency starts
  const bool was_activated = emergency_stop_activated_.exchange(
    emergency_stop_msg.emergency_stop_activated);
  if (!was_activated && emergency_stop_msg.emergency_stop_activated) {
    authority_->callServiceAsync("setVelocity", mira::Velocity2(0.0, 0.0, 0.0));
  }
}

void Drive::rfidStatusCallback(mira::ChannelRead<uint64> data)
{
  if (!is_active_) {
    return;
  }

  if (isBarrierCode(data->value())) {
    auto barrier_status = miraToRosBarrierStatus(data->value(), data->timestamp);
    std::lock_guard<std::mutex> lock(mutex_);
    barrier_status_ = barrier_status;
    magnetic_barrier_pub_->publish(barrier_status_);
  }

  scitos2_msgs::msg::RfidTag tag_msg;
  tag_msg.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
  tag_msg.header.frame_id = robot_base_frame_;
  tag_msg.tag = data->value();
  rfid_pub_->publish(tag_msg);
}

void Drive::targetVelocityCallback(mira::ChannelRead<mira::Velocity2> data)
{
  if (!is_active_) {
    return;
  }

  cmd_vel_applied_pub_->publish(miraToRosTargetVelocity(data->value(), data->timestamp));
}

void Drive::velocityCommandCallback(const geometry_msgs::msg::TwistStamped & msg)
{
  if (emergency_stop_activated_ || !is_active_) {
    return;
  }
  if (isCommandStale(msg.header.stamp)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Discarding a cmd_vel older than %.2f s", cmd_vel_max_age_);
    return;
  }

  // Never wait for the robot here: a slow answer would block the executor, and an old
  // velocity is worthless once there is a newer one
  mira::Velocity2 speed(msg.twist.linear.x, 0, msg.twist.angular.z);
  authority_->callServiceAsync("setVelocity", speed);
}

bool Drive::isCommandStale(const builtin_interfaces::msg::Time & stamp) const
{
  if (cmd_vel_max_age_ <= 0.0 || (stamp.sec == 0 && stamp.nanosec == 0)) {
    return false;
  }
  const auto age = clock_->now() - rclcpp::Time(stamp, clock_->get_clock_type());
  return age.seconds() > cmd_vel_max_age_;
}

bool Drive::resolveDriveModule(const std::chrono::milliseconds & timeout)
{
  if (drive_module_ != "auto") {
    return true;
  }

  // Each candidate is probed with a property that only its own family of boards exposes
  static const std::array<std::pair<std::string, std::string>, 2> candidates = {{
    std::make_pair("MotorController", "MotorController.Force"),
    std::make_pair("MainControlUnit", "MainControlUnit.RearLaser.Enabled")}};

  // The resource may be up before the robot has loaded its modules, so the properties
  // appear a moment later. Without a reachable resource there is nothing to wait for
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    for (const auto & [module, probe] : candidates) {
      if (!authority_->getParam(probe).empty()) {
        drive_module_ = module;
        RCLCPP_INFO(logger_, "Detected the MIRA drive module: [%s]", drive_module_.c_str());
        return true;
      }
    }
    if (!authority_->isReady() || std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  RCLCPP_WARN(logger_, "Unable to detect the MIRA drive module, the property was not written");
  return false;
}

bool Drive::setDriveParam(const std::string & property, const std::string & value)
{
  if (!resolveDriveModule()) {
    return false;
  }
  return authority_->setParam(drive_module_ + "." + property, value);
}

bool Drive::changeForce(
  const std::shared_ptr<scitos2_msgs::srv::ChangeForce::Request> request,
  std::shared_ptr<scitos2_msgs::srv::ChangeForce::Response> response)
{
  response->success = setDriveParam("Force", mira::toString(request->force));
  if (!response->success) {
    response->message = "MIRA service call failed";
  }
  return response->success;
}

bool Drive::emergencyStop(
  const std::shared_ptr<scitos2_msgs::srv::EmergencyStop::Request>/*request*/,
  std::shared_ptr<scitos2_msgs::srv::EmergencyStop::Response> response)
{
  response->success = authority_->callService("emergencyStop");
  if (!response->success) {
    response->message = "MIRA service call failed";
  }
  return response->success;
}

bool Drive::enableMotors(
  const std::shared_ptr<scitos2_msgs::srv::EnableMotors::Request> request,
  std::shared_ptr<scitos2_msgs::srv::EnableMotors::Response> response)
{
  response->success = authority_->callService(
    "enableMotors", std::optional<bool>(request->enable));
  if (!response->success) {
    response->message = "MIRA service call failed";
  }
  return response->success;
}

bool Drive::enableRfid(
  const std::shared_ptr<scitos2_msgs::srv::EnableRfid::Request> request,
  std::shared_ptr<scitos2_msgs::srv::EnableRfid::Response> response)
{
  response->success = setDriveParam("RearLaser.Enabled", request->enable ? "true" : "false");
  if (!response->success) {
    response->message = "MIRA service call failed";
  }
  return response->success;
}

bool Drive::resetBarrierStop(
  const std::shared_ptr<scitos2_msgs::srv::ResetBarrierStop::Request>/*request*/,
  std::shared_ptr<scitos2_msgs::srv::ResetBarrierStop::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    barrier_status_.header.frame_id = robot_base_frame_;
    barrier_status_.header.stamp = clock_->now();
    barrier_status_.barrier_stopped = false;
    magnetic_barrier_pub_->publish(barrier_status_);
  }
  response->success = true;
  return response->success;
}

bool Drive::resetMotorStop(
  const std::shared_ptr<scitos2_msgs::srv::ResetMotorStop::Request>/*request*/,
  std::shared_ptr<scitos2_msgs::srv::ResetMotorStop::Response> response)
{
  response->success = authority_->callService("resetMotorStop");
  if (!response->success) {
    response->message = "MIRA service call failed";
  }
  return response->success;
}

bool Drive::resetOdometry(
  const std::shared_ptr<scitos2_msgs::srv::ResetOdometry::Request>/*request*/,
  std::shared_ptr<scitos2_msgs::srv::ResetOdometry::Response> response)
{
  response->success = authority_->callService("resetOdometry");
  if (!response->success) {
    response->message = "MIRA service call failed";
  }
  return response->success;
}

bool Drive::suspendBumper(
  const std::shared_ptr<scitos2_msgs::srv::SuspendBumper::Request>/*request*/,
  std::shared_ptr<scitos2_msgs::srv::SuspendBumper::Response> response)
{
  response->success = authority_->callService("suspendBumper");
  if (!response->success) {
    response->message = "MIRA service call failed";
  }
  return response->success;
}

visualization_msgs::msg::MarkerArray Drive::createBumperMarkers(std_msgs::msg::Header header)
{
  // Create markers
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "bumper";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.45;
  marker.pose.position.z = 0.03;
  marker.pose.orientation.w = 1.0;

  // Change color depending on the state: red if activated, white otherwise
  if (bumper_activated_) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  } else {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
  }

  // Create the markers using the footprint
  int id = 0;
  for (const auto & point : unpadded_footprint_) {
    marker.points.push_back(point);
    marker.id = id++;
  }

  // Last point is the first point to close the polygon
  if (!unpadded_footprint_.empty()) {
    marker.points.push_back(unpadded_footprint_.front());
  }

  marker_array.markers.push_back(marker);

  return marker_array;
}

bool Drive::isEmergencyStopReleased(scitos2_msgs::msg::EmergencyStopStatus msg)
{
  return msg.emergency_stop_activated && !msg.emergency_stop_status;
}

void Drive::resetMotorStopAfterTimeout(rclcpp::Time current_time)
{
  // A non-positive interval means the automatic reset is disabled
  if (reset_bumper_interval_.seconds() <= 0.0) {
    return;
  }

  if (bumper_activated_ && (current_time - last_bumper_reset_) > reset_bumper_interval_) {
    if (authority_) {
      authority_->callService("resetMotorStop");
    }
    last_bumper_reset_ = current_time;
  }
}

nav_msgs::msg::Odometry Drive::miraToRosOdometry(
  const mira::robot::Odometry2 & odometry, const mira::Time & timestamp)
{
  nav_msgs::msg::Odometry odom_msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = robot_base_frame_;
  }
  odom_msg.header.stamp = rclcpp::Time(timestamp.toUnixNS());

  // Set the position
  odom_msg.pose.pose.position.x = odometry.pose.x();
  odom_msg.pose.pose.position.y = odometry.pose.y();
  odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, odometry.pose.phi()));

  // Set the velocity
  odom_msg.twist.twist.linear.x = odometry.velocity.x();
  odom_msg.twist.twist.angular.z = odometry.velocity.phi();

  return odom_msg;
}

geometry_msgs::msg::TransformStamped Drive::miraToRosTf(
  const mira::robot::Odometry2 & odometry, const mira::Time & timestamp)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = robot_base_frame_;
  }
  tf_msg.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  tf_msg.transform.translation.x = odometry.pose.x();
  tf_msg.transform.translation.y = odometry.pose.y();
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, odometry.pose.phi()));
  return tf_msg;
}

void Drive::diagnostics(diagnostic_msgs::msg::DiagnosticArray & array)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;
  stat.name = "Drive";
  stat.hardware_id = "scitos2";
  fillDiagnostics(stat);
  array.status.push_back(stat);
}

void Drive::fillDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  uint32_t status;
  std::chrono::steady_clock::time_point status_time;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status = last_status_;
    status_time = last_status_time_;
  }

  const bool stale = status_time == std::chrono::steady_clock::time_point{} ||
  std::chrono::steady_clock::now() - status_time > kStatusTimeout;
  if (!is_active_) {
    stat.summary(DiagnosticStatus::OK, "Module is not active");
  } else if (stale) {
    stat.summary(DiagnosticStatus::ERROR, "No drive status received from MIRA");
  } else {
    summarizeDriveStatus(status, stat);
  }
}

void Drive::summarizeDriveStatus(
  uint32_t status, diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  uint8_t level = DiagnosticStatus::OK;
  std::string raised;
  for (const auto & flag : kDriveStatusFlags) {
    if (!isDriveStatusBitSet(status, flag.bit)) {
      continue;
    }
    level = std::max(level, flag.level);
    raised += (raised.empty() ? "" : ", ") + std::string(flag.name);
  }
  stat.summary(level, raised.empty() ? "Drive OK" : raised);

  for (const auto & flag : kDriveStatusFlags) {
    stat.add(flag.name, isDriveStatusBitSet(status, flag.bit));
  }
  stat.add("mode_normal", isDriveStatusBitSet(status, DriveStatusBit::MODE_NORMAL));
  stat.add(
    "safety_field_front_laser",
    isDriveStatusBitSet(status, DriveStatusBit::SAFETY_FIELD_FRONT_LASER));
  stat.add(
    "safety_field_rear_laser",
    isDriveStatusBitSet(status, DriveStatusBit::SAFETY_FIELD_REAR_LASER));
}

geometry_msgs::msg::TwistStamped Drive::miraToRosTargetVelocity(
  const mira::Velocity2 & velocity, const mira::Time & timestamp)
{
  geometry_msgs::msg::TwistStamped twist;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    twist.header.frame_id = robot_base_frame_;
  }
  twist.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  twist.twist.linear.x = velocity.x();
  twist.twist.linear.y = velocity.y();
  twist.twist.angular.z = velocity.phi();
  return twist;
}

scitos2_msgs::msg::DriveStatus Drive::miraToRosDriveStatus(
  const uint32 & status, const mira::Time & timestamp)
{
  scitos2_msgs::msg::DriveStatus drive_status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    drive_status.header.frame_id = robot_base_frame_;
  }
  drive_status.header.stamp = rclcpp::Time(timestamp.toUnixNS());

  // The bit layout belongs to MIRA, see drive_status_bits.hpp
  auto is_set = [status](DriveStatusBit bit) {return isDriveStatusBitSet(status, bit);};
  drive_status.mode_normal = is_set(DriveStatusBit::MODE_NORMAL);
  drive_status.mode_forced_stopped = is_set(DriveStatusBit::MODE_FORCED_STOPPED);
  drive_status.mode_freerun = is_set(DriveStatusBit::MODE_FREERUN);
  drive_status.mode_safe_drive = is_set(DriveStatusBit::MODE_SAFE_DRIVE);
  drive_status.mode_bumper_escape = is_set(DriveStatusBit::MODE_BUMPER_ESCAPE);
  drive_status.error_sifas_communication = is_set(DriveStatusBit::ERROR_SIFAS_COMMUNICATION);
  drive_status.error_sifas_internal = is_set(DriveStatusBit::ERROR_SIFAS_INTERNAL);
  drive_status.emergency_stop_activated = is_set(DriveStatusBit::EMERGENCY_STOP_ACTIVATED);
  drive_status.emergency_stop_status = is_set(DriveStatusBit::EMERGENCY_STOP_STATUS);
  drive_status.bumper_front_activated = is_set(DriveStatusBit::BUMPER_FRONT_ACTIVATED);
  drive_status.bumper_front_status = is_set(DriveStatusBit::BUMPER_FRONT_STATUS);
  drive_status.bumper_rear_activated = is_set(DriveStatusBit::BUMPER_REAR_ACTIVATED);
  drive_status.bumper_rear_status = is_set(DriveStatusBit::BUMPER_REAR_STATUS);
  drive_status.magnetic_proximity_sensor_activated =
    is_set(DriveStatusBit::MAGNETIC_PROXIMITY_SENSOR_ACTIVATED);
  drive_status.magnetic_proximity_sensor_status =
    is_set(DriveStatusBit::MAGNETIC_PROXIMITY_SENSOR_STATUS);
  drive_status.external_safety_ctrl_status = is_set(DriveStatusBit::EXTERNAL_SAFETY_CTRL_STATUS);
  drive_status.error_battery_low = is_set(DriveStatusBit::ERROR_BATTERY_LOW);
  drive_status.error_temperature_high = is_set(DriveStatusBit::ERROR_TEMPERATURE_HIGH);
  drive_status.error_stall_mode = is_set(DriveStatusBit::ERROR_STALL_MODE);
  drive_status.limited_pwm_left = is_set(DriveStatusBit::LIMITED_PWM_LEFT);
  drive_status.limited_pwm_right = is_set(DriveStatusBit::LIMITED_PWM_RIGHT);
  drive_status.error_rotation_too_high = is_set(DriveStatusBit::ERROR_ROTATION_TOO_HIGH);
  drive_status.error_motor_acceleration = is_set(DriveStatusBit::ERROR_MOTOR_ACCELERATION);
  drive_status.error_motor_left = is_set(DriveStatusBit::ERROR_MOTOR_LEFT);
  drive_status.error_motor_right = is_set(DriveStatusBit::ERROR_MOTOR_RIGHT);
  drive_status.safety_field_rear_laser = is_set(DriveStatusBit::SAFETY_FIELD_REAR_LASER);
  drive_status.safety_field_front_laser = is_set(DriveStatusBit::SAFETY_FIELD_FRONT_LASER);
  drive_status.error_firmare = is_set(DriveStatusBit::ERROR_FIRMARE);
  drive_status.relay_active = is_set(DriveStatusBit::RELAY_ACTIVE);
  drive_status.temperature_warning = is_set(DriveStatusBit::TEMPERATURE_WARNING);
  drive_status.traction_loss = is_set(DriveStatusBit::TRACTION_LOSS);
  drive_status.software_safe_stop = is_set(DriveStatusBit::SOFTWARE_SAFE_STOP);

  return drive_status;
}

scitos2_msgs::msg::EmergencyStopStatus Drive::miraToRosEmergencyStopStatus(
  const uint32 & status, const mira::Time & timestamp)
{
  scitos2_msgs::msg::EmergencyStopStatus emergency_stop_status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_stop_status.header.frame_id = robot_base_frame_;
  }
  emergency_stop_status.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  emergency_stop_status.emergency_stop_activated =
    isDriveStatusBitSet(status, DriveStatusBit::EMERGENCY_STOP_ACTIVATED);
  emergency_stop_status.emergency_stop_status =
    isDriveStatusBitSet(status, DriveStatusBit::EMERGENCY_STOP_STATUS);
  return emergency_stop_status;
}

scitos2_msgs::msg::BarrierStatus Drive::miraToRosBarrierStatus(
  const uint64 & /*status*/, const mira::Time & timestamp)
{
  // The caller only invokes this after isBarrierCode() confirmed the RFID code marks a
  // barrier, so barrier_stopped is unconditionally true here; the code value itself isn't
  // needed to build the message.
  scitos2_msgs::msg::BarrierStatus barrier;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    barrier.header.frame_id = robot_base_frame_;
  }
  barrier.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  barrier.barrier_stopped = true;
  barrier.last_detection_stamp = rclcpp::Time(timestamp.toUnixNS());
  return barrier;
}

}  // namespace scitos2_modules

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(scitos2_modules::Drive, scitos2_core::Module)
