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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "scitos2_modules/drive.hpp"

namespace scitos2_modules
{

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;

void Drive::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
  // Declare and read parameters
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  is_active_ = false;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  authority_ = std::make_shared<mira::Authority>();
  authority_->checkin("/", plugin_name_);

  // Create ROS publishers
  auto latched_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  bumper_pub_ = node->create_publisher<scitos2_msgs::msg::BumperStatus>(
    "bumper", latched_profile);
  bumper_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "bumper/visualization", 20);
  drive_status_pub_ = node->create_publisher<scitos2_msgs::msg::DriveStatus>(
    "drive_status", 20);
  emergency_stop_pub_ = node->create_publisher<scitos2_msgs::msg::EmergencyStopStatus>(
    "emergency_stop_status", latched_profile);
  magnetic_barrier_pub_ = node->create_publisher<scitos2_msgs::msg::BarrierStatus>(
    "barrier_status", latched_profile);
  mileage_pub_ = node->create_publisher<scitos2_msgs::msg::Mileage>("mileage", 20);
  odometry_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  rfid_pub_ = node->create_publisher<scitos2_msgs::msg::RfidTag>("rfid", 20);

  // Create MIRA subscribers
  authority_->subscribe<mira::robot::Odometry2>(
    "/robot/Odometry", &Drive::odometryDataCallback, this);
  authority_->subscribe<bool>(
    "/robot/Bumper", &Drive::bumperDataCallback, this);
  authority_->subscribe<float>(
    "/robot/Mileage", &Drive::mileageDataCallback, this);
  authority_->subscribe<uint32>(
    "/robot/DriveStatusPlain", &Drive::driveStatusCallback, this);
  authority_->subscribe<uint64>(
    "/robot/RFIDUserTag", &Drive::rfidStatusCallback, this);

  // Create ROS subscribers
  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1,
    std::bind(&Drive::velocityCommandCallback, this, _1));

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

  // Declare and read parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".base_frame",
    rclcpp::ParameterValue("base_link"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the base frame of the robot"));
  node->get_parameter(plugin_name_ + ".base_frame", base_frame_);
  RCLCPP_INFO(logger_, "The parameter base_frame is set to: [%s]", base_frame_.c_str());

  bool magnetic_barrier_enabled;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".magnetic_barrier_enabled",
    rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable the magnetic strip detector to cut out the motors"));
  node->get_parameter(plugin_name_ + ".magnetic_barrier_enabled", magnetic_barrier_enabled);
  RCLCPP_INFO(
    logger_, "The parameter magnetic_barrier_enabled is set to: [%s]",
    magnetic_barrier_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_tf",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Publish the odometry as a tf2 transform"));
  node->get_parameter(plugin_name_ + ".publish_tf", publish_tf_);
  RCLCPP_INFO(
    logger_, "The parameter publish_tf_ is set to: [%s]", publish_tf_ ? "true" : "false");

  int rbi = 0;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".reset_bumper_interval",
    rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The interval in milliseconds to reset the bumper"));
  node->get_parameter(plugin_name_ + ".reset_bumper_interval", rbi);
  RCLCPP_INFO(logger_, "The parameter reset_bumper_interval is set to: [%i]", rbi);
  reset_bumper_interval_ = rclcpp::Duration::from_seconds(rbi / 1000.0);

  set_mira_param(
    authority_, "MainControlUnit.RearLaser.Enabled", magnetic_barrier_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".footprint",
    rclcpp::ParameterValue(""), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The footprint of the robot"));
  node->get_parameter(plugin_name_ + ".footprint", footprint_);
  RCLCPP_INFO(logger_, "The parameter footprint is set to: [%s]", footprint_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_radius",
    rclcpp::ParameterValue(0.5), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The radius of the robot"));
  node->get_parameter(plugin_name_ + ".robot_radius", robot_radius_);
  RCLCPP_INFO(logger_, "The parameter robot_radius is set to: [%f]", robot_radius_);

  // If the footprint has been specified, it must be in the correct format
  use_radius_ = true;
  if (footprint_ != "" && footprint_ != "[]") {
    // Footprint parameter has been specified, try to convert it
    if (nav2_costmap_2d::makeFootprintFromString(footprint_, unpadded_footprint_)) {
      // The specified footprint is valid, so we'll use that instead of the radius
      use_radius_ = false;
    } else {
      // Footprint provided but invalid, so stay with the radius
      unpadded_footprint_ = nav2_costmap_2d::makeFootprintFromRadius(robot_radius_);
      RCLCPP_ERROR(
        logger_, "The footprint parameter is invalid: \"%s\", using radius (%lf) instead",
        footprint_.c_str(), robot_radius_);
    }
  }

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
    logger_,
    "Cleaning up module : %s of type scitos2_module::Drive",
    plugin_name_.c_str());
  authority_.reset();
  bumper_pub_.reset();
  bumper_markers_pub_.reset();
  drive_status_pub_.reset();
  emergency_stop_pub_.reset();
  magnetic_barrier_pub_.reset();
  mileage_pub_.reset();
  odometry_pub_.reset();
  rfid_pub_.reset();
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

void Drive::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating module : %s of type scitos2_module::Drive",
    plugin_name_.c_str());
  bumper_pub_->on_activate();
  bumper_markers_pub_->on_activate();
  drive_status_pub_->on_activate();
  emergency_stop_pub_->on_activate();
  magnetic_barrier_pub_->on_activate();
  mileage_pub_->on_activate();
  odometry_pub_->on_activate();
  rfid_pub_->on_activate();
  authority_->start();
  is_active_ = true;
}

void Drive::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating module : %s of type scitos2_module::Drive",
    plugin_name_.c_str());
  authority_->checkout();
  bumper_pub_->on_deactivate();
  bumper_markers_pub_->on_deactivate();
  drive_status_pub_->on_deactivate();
  emergency_stop_pub_->on_deactivate();
  magnetic_barrier_pub_->on_deactivate();
  mileage_pub_->on_deactivate();
  odometry_pub_->on_deactivate();
  rfid_pub_->on_deactivate();
  is_active_ = false;
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
        bool magnetic_barrier_enabled = parameter.as_bool();
        set_mira_param(
          authority_, "MainControlUnit.RearLaser.Enabled",
          magnetic_barrier_enabled ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter magnetic_barrier_enabled is set to: [%s]",
          magnetic_barrier_enabled ? "true" : "false");
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".base_frame") {
        base_frame_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter base_frame is set to: [%s]", base_frame_.c_str());
      }
    }
  }

  result.successful = true;
  return result;
}

void Drive::odometryDataCallback(mira::ChannelRead<mira::robot::Odometry2> data)
{
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
  rclcpp::Time stamp = rclcpp::Time(data->timestamp.toUnixNS());

  scitos2_msgs::msg::BumperStatus bumper_status;
  bumper_status.header.frame_id = base_frame_;
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
  scitos2_msgs::msg::Mileage mileage_msg;
  mileage_msg.header.frame_id = base_frame_;
  mileage_msg.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
  mileage_msg.distance = data->value();
  mileage_pub_->publish(mileage_msg);
}

void Drive::driveStatusCallback(mira::ChannelRead<uint32> data)
{
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
}

void Drive::rfidStatusCallback(mira::ChannelRead<uint64> data)
{
  if (isBarrierCode(data->value())) {
    auto barrier_status = miraToRosBarrierStatus(data->value(), data->timestamp);
    magnetic_barrier_pub_->publish(barrier_status_);
  }

  scitos2_msgs::msg::RfidTag tag_msg;
  tag_msg.tag = data->value();
  rfid_pub_->publish(tag_msg);
}

void Drive::velocityCommandCallback(const geometry_msgs::msg::Twist & msg)
{
  if (!emergency_stop_activated_ && is_active_) {
    mira::Velocity2 speed(msg.linear.x, 0, msg.angular.z);
    call_mira_service(authority_, "setVelocity", std::optional<mira::Velocity2>(speed));
  }
}

bool Drive::changeForce(
  const std::shared_ptr<scitos2_msgs::srv::ChangeForce::Request> request,
  std::shared_ptr<scitos2_msgs::srv::ChangeForce::Response> response)
{
  return set_mira_param(authority_, "MotorController.Force", mira::toString(request->force));
}

bool Drive::emergencyStop(
  const std::shared_ptr<scitos2_msgs::srv::EmergencyStop::Request> request,
  std::shared_ptr<scitos2_msgs::srv::EmergencyStop::Response> response)
{
  return call_mira_service(authority_, "emergencyStop");
}

bool Drive::enableMotors(
  const std::shared_ptr<scitos2_msgs::srv::EnableMotors::Request> request,
  std::shared_ptr<scitos2_msgs::srv::EnableMotors::Response> response)
{
  return call_mira_service(authority_, "enableMotors", std::optional<bool>(request->enable));
}

bool Drive::enableRfid(
  const std::shared_ptr<scitos2_msgs::srv::EnableRfid::Request> request,
  std::shared_ptr<scitos2_msgs::srv::EnableRfid::Response> response)
{
  return set_mira_param(
    authority_, "MainControlUnit.RearLaser.Enabled",
    request->enable ? "true" : "false");
}

bool Drive::resetBarrierStop(
  const std::shared_ptr<scitos2_msgs::srv::ResetBarrierStop::Request> request,
  std::shared_ptr<scitos2_msgs::srv::ResetBarrierStop::Response> response)
{
  barrier_status_.header.frame_id = base_frame_;
  barrier_status_.header.stamp = clock_->now();
  barrier_status_.barrier_stopped = false;
  magnetic_barrier_pub_->publish(barrier_status_);
  return true;
}

bool Drive::resetMotorStop(
  const std::shared_ptr<scitos2_msgs::srv::ResetMotorStop::Request> request,
  std::shared_ptr<scitos2_msgs::srv::ResetMotorStop::Response> response)
{
  return call_mira_service(authority_, "resetMotorStop");
}

bool Drive::resetOdometry(
  const std::shared_ptr<scitos2_msgs::srv::ResetOdometry::Request> request,
  std::shared_ptr<scitos2_msgs::srv::ResetOdometry::Response> response)
{
  return call_mira_service(authority_, "resetOdometry");
}

bool Drive::suspendBumper(
  const std::shared_ptr<scitos2_msgs::srv::SuspendBumper::Request> request,
  std::shared_ptr<scitos2_msgs::srv::SuspendBumper::Response> response)
{
  return call_mira_service(authority_, "suspendBumper");
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
  if (bumper_activated_ && (current_time - last_bumper_reset_) > reset_bumper_interval_) {
    call_mira_service(authority_, "resetMotorStop");
    last_bumper_reset_ = current_time;
  }
}

nav_msgs::msg::Odometry Drive::miraToRosOdometry(
  const mira::robot::Odometry2 & odometry, const mira::Time & timestamp)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  odom_msg.child_frame_id = base_frame_;

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
  tf_msg.header.frame_id = "odom";
  tf_msg.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  tf_msg.child_frame_id = base_frame_;
  tf_msg.transform.translation.x = odometry.pose.x();
  tf_msg.transform.translation.y = odometry.pose.y();
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, odometry.pose.phi()));
  return tf_msg;
}

scitos2_msgs::msg::DriveStatus Drive::miraToRosDriveStatus(
  const uint32 & status, const mira::Time & timestamp)
{
  scitos2_msgs::msg::DriveStatus drive_status;
  drive_status.header.frame_id = base_frame_;
  drive_status.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  drive_status.mode_normal = static_cast<bool>(status & 1);
  drive_status.mode_forced_stopped = static_cast<bool>(status & (1 << 1));
  drive_status.mode_freerun = static_cast<bool>(status & (1 << 2));

  drive_status.emergency_stop_activated = static_cast<bool>(status & (1 << 7));
  drive_status.emergency_stop_status = static_cast<bool>(status & (1 << 8));
  drive_status.bumper_front_activated = static_cast<bool>(status & (1 << 9));
  drive_status.bumper_front_status = static_cast<bool>(status & (1 << 10));
  drive_status.bumper_rear_activated = static_cast<bool>(status & (1 << 11));
  drive_status.bumper_rear_status = static_cast<bool>(status & (1 << 12));

  drive_status.safety_field_rear_laser = static_cast<bool>(status & (1 << 26));
  drive_status.safety_field_front_laser = static_cast<bool>(status & (1 << 27));

  return drive_status;
}

scitos2_msgs::msg::EmergencyStopStatus Drive::miraToRosEmergencyStopStatus(
  const uint32 & status, const mira::Time & timestamp)
{
  scitos2_msgs::msg::EmergencyStopStatus emergency_stop_status;
  emergency_stop_status.header.frame_id = base_frame_;
  emergency_stop_status.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  emergency_stop_status.emergency_stop_activated = static_cast<bool>(status & (1 << 7));
  emergency_stop_status.emergency_stop_status = static_cast<bool>(status & (1 << 8));
  return emergency_stop_status;
}

scitos2_msgs::msg::BarrierStatus Drive::miraToRosBarrierStatus(
  const uint64 & status, const mira::Time & timestamp)
{
  scitos2_msgs::msg::BarrierStatus barrier;
  barrier.header.frame_id = base_frame_;
  barrier.header.stamp = rclcpp::Time(timestamp.toUnixNS());
  barrier.barrier_stopped = true;
  barrier.last_detection_stamp = rclcpp::Time(timestamp.toUnixNS());
  return barrier;
}

}  // namespace scitos2_modules

PLUGINLIB_EXPORT_CLASS(scitos2_modules::Drive, scitos2_core::Module)
