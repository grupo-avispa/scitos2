// Copyright (c) 2024 Alberto J. Tudela Roldán
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

  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  authority_ = std::make_shared<mira::Authority>("/", name);

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

  timer_ = node->create_wall_timer(
    std::chrono::duration<double>(0.5),
    std::bind(&Drive::publishBumperMarkers, this));

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
    "change_force", std::bind(&Drive::changeForce, this, _1, _2));
  emergency_stop_service_ = node->create_service<scitos2_msgs::srv::EmergencyStop>(
    "emergency_stop", std::bind(&Drive::emergencyStop, this, _1, _2));
  enable_motors_service_ = node->create_service<scitos2_msgs::srv::EnableMotors>(
    "enable_motors", std::bind(&Drive::enableMotors, this, _1, _2));
  enable_rfid_service_ = node->create_service<scitos2_msgs::srv::EnableRfid>(
    "enable_rfid", std::bind(&Drive::enableRfid, this, _1, _2));
  reset_barrier_stop_service_ = node->create_service<scitos2_msgs::srv::ResetBarrierStop>(
    "reset_barrier_stop", std::bind(&Drive::resetBarrierStop, this, _1, _2));
  reset_motor_stop_service_ = node->create_service<scitos2_msgs::srv::ResetMotorStop>(
    "reset_motorstop", std::bind(&Drive::resetMotorStop, this, _1, _2));
  reset_odometry_service_ = node->create_service<scitos2_msgs::srv::ResetOdometry>(
    "reset_odometry", std::bind(&Drive::resetOdometry, this, _1, _2));
  suspend_bumper_service_ = node->create_service<scitos2_msgs::srv::SuspendBumper>(
    "suspend_bumper", std::bind(&Drive::suspendBumper, this, _1, _2));

  // Declare and read parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + "base_frame",
    rclcpp::ParameterValue("base_footprint"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the base frame of the robot"));
  node->get_parameter(plugin_name_ + "base_frame", base_frame_);
  RCLCPP_INFO(logger_, "The parameter base_frame is set to: [%s]", base_frame_.c_str());

  bool magnetic_barrier_enabled;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + "magnetic_barrier_enabled",
    rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable the magnetic strip detector to cut out the motors"));
  node->get_parameter(plugin_name_ + "magnetic_barrier_enabled", magnetic_barrier_enabled);
  RCLCPP_INFO(
    logger_, "The parameter magnetic_barrier_enabled is set to: [%s]",
    magnetic_barrier_enabled ? "true" : "false");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + "publish_tf",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Publish the odometry as a tf2 transform"));
  node->get_parameter(plugin_name_ + "publish_tf", publish_tf_);
  RCLCPP_INFO(
    logger_, "The parameter publish_tf_ is set to: [%s]",
    publish_tf_ ? "true" : "false");

  int rbi = 0;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + "reset_bumper_interval",
    rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The interval in milliseconds to reset the bumper"));
  node->get_parameter(plugin_name_ + "reset_bumper_interval", rbi);
  RCLCPP_INFO(logger_, "The parameter reset_bumper_interval is set to: [%i]", rbi);
  reset_bumper_interval_ = rclcpp::Duration::from_seconds(rbi / 1000.0);

  set_mira_param(
    authority_, "MainControlUnit.RearLaser.Enabled", magnetic_barrier_enabled ? "true" : "false");

  // Callback for monitor changes in parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Drive::dynamicParametersCallback, this, _1));

  // Initialize the transform broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }
}

void Drive::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up module : %s of type scitos2_module::Drive",
    plugin_name_.c_str());
  authority_->checkout();
  authority_.reset();
  bumper_pub_.reset();
  bumper_markers_pub_.reset();
  drive_status_pub_.reset();
  emergency_stop_pub_.reset();
  magnetic_barrier_pub_.reset();
  mileage_pub_.reset();
  odometry_pub_.reset();
  rfid_pub_.reset();
  timer_.reset();
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
}

void Drive::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating module : %s of type scitos2_module::Drive",
    plugin_name_.c_str());
  bumper_pub_->on_deactivate();
  bumper_markers_pub_->on_deactivate();
  drive_status_pub_->on_deactivate();
  emergency_stop_pub_->on_deactivate();
  magnetic_barrier_pub_->on_deactivate();
  mileage_pub_->on_deactivate();
  odometry_pub_->on_deactivate();
  rfid_pub_->on_deactivate();
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
      if (name == plugin_name_ + "magnetic_barrier_enabled") {
        bool magnetic_barrier_enabled = parameter.as_bool();
        set_mira_param(
          authority_, "MainControlUnit.RearLaser.Enabled",
          magnetic_barrier_enabled ? "true" : "false");
        RCLCPP_INFO(
          logger_, "The parameter magnetic_barrier_enabled is set to: [%s]",
          magnetic_barrier_enabled ? "true" : "false");
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + "base_frame") {
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
  // Get odometry data from mira
  rclcpp::Time odom_time = rclcpp::Time(data->timestamp.toUnixNS());

  // Create quaternion from yaw
  geometry_msgs::msg::Quaternion orientation = tf2::toMsg(
    tf2::Quaternion(
      {0, 0, 1},
      data->value().pose.phi()));

  // Publish as a nav_msgs::Odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = odom_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = base_frame_;

  // Set the position
  odom_msg.pose.pose.position.x = data->value().pose.x();
  odom_msg.pose.pose.position.y = data->value().pose.y();
  odom_msg.pose.pose.orientation = orientation;

  // Set the velocity
  odom_msg.twist.twist.linear.x = data->value().velocity.x();
  odom_msg.twist.twist.angular.z = data->value().velocity.phi();

  odometry_pub_->publish(odom_msg);

  // Publish a TF
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = data->value().pose.x();
    tf_msg.transform.translation.y = data->value().pose.y();
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void Drive::bumperDataCallback(mira::ChannelRead<bool> data)
{
  rclcpp::Time stamp = rclcpp::Time(data->timestamp.toUnixNS());
  bumper_status_.header.frame_id = base_frame_;
  bumper_status_.header.stamp = stamp;
  bumper_status_.bumper_activated = data->value();
  bumper_pub_->publish(bumper_status_);

  // Reset motorstop if bumper is activated after a certain time
  if (!bumper_status_.bumper_activated ||
    reset_bumper_interval_ == rclcpp::Duration::from_seconds(0) ||
    (stamp - last_bumper_reset_) < reset_bumper_interval_)
  {
    return;
  }

  last_bumper_reset_ = stamp;
  call_mira_service(authority_, "resetMotorStop");
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
  scitos2_msgs::msg::DriveStatus status_msg;
  status_msg.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
  status_msg.header.frame_id = base_frame_;
  status_msg.mode_normal = static_cast<bool>((*data) & 1);
  status_msg.mode_forced_stopped = static_cast<bool>((*data) & (1 << 1));
  status_msg.mode_freerun = static_cast<bool>((*data) & (1 << 2));

  status_msg.emergency_stop_activated = static_cast<bool>((*data) & (1 << 7));
  status_msg.emergency_stop_status = static_cast<bool>((*data) & (1 << 8));
  status_msg.bumper_front_activated = static_cast<bool>((*data) & (1 << 9));
  status_msg.bumper_front_status = static_cast<bool>((*data) & (1 << 10));
  status_msg.bumper_rear_activated = static_cast<bool>((*data) & (1 << 11));
  status_msg.bumper_rear_status = static_cast<bool>((*data) & (1 << 12));

  status_msg.safety_field_rear_laser = static_cast<bool>((*data) & (1 << 26));
  status_msg.safety_field_front_laser = static_cast<bool>((*data) & (1 << 27));

  // Reset motorstop when emergency button is released
  if (status_msg.emergency_stop_activated && !status_msg.emergency_stop_status) {
    call_mira_service(authority_, "resetMotorStop");
  }

  RCLCPP_DEBUG_STREAM(
    logger_, "Drive controller status "
      << "(Nor: " << status_msg.mode_normal
      << ", MStop: " << status_msg.mode_forced_stopped
      << ", FreeRun: " << status_msg.mode_freerun
      << ", EmBut: " << status_msg.emergency_stop_activated
      << ", BumpPres: " << status_msg.bumper_front_activated
      << ", BusErr: " << status_msg.error_sifas_communication
      << ", Stall: " << status_msg.error_stall_mode
      << ", InterErr: " << status_msg.error_sifas_internal << ")");

  drive_status_pub_->publish(status_msg);
}

void Drive::rfidStatusCallback(mira::ChannelRead<uint64> data)
{
  scitos2_msgs::msg::RfidTag tag_msg;
  if (data->value() == MAGNETIC_BARRIER_RFID_CODE) {
    barrier_status_.header.frame_id = base_frame_;
    barrier_status_.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
    barrier_status_.barrier_stopped = true;
    barrier_status_.last_detection_stamp = rclcpp::Time(data->timestamp.toUnixNS());
    magnetic_barrier_pub_->publish(barrier_status_);
  }
  tag_msg.tag = data->value();
  rfid_pub_->publish(tag_msg);
}

void Drive::velocityCommandCallback(const geometry_msgs::msg::Twist & msg)
{
  if (!emergency_stop_.emergency_stop_activated) {
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
  // Publish emergency stop
  emergency_stop_.header.frame_id = base_frame_;
  emergency_stop_.header.stamp = clock_->now();
  emergency_stop_.emergency_stop_activated = true;
  emergency_stop_pub_->publish(emergency_stop_);

  // Call mira service
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
  // Publish emergency stop
  emergency_stop_.header.frame_id = base_frame_;
  emergency_stop_.header.stamp = clock_->now();
  emergency_stop_.emergency_stop_activated = false;
  emergency_stop_pub_->publish(emergency_stop_);

  // Call mira service
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

void Drive::publishBumperMarkers()
{
  // Note: Only perform visualization if there's any subscriber
  if (bumper_markers_pub_->get_subscription_count() == 0) {return;}

  // Publish markers
  visualization_msgs::msg::MarkerArray markers_msg;
  visualization_msgs::msg::Marker cylinder;
  cylinder.header = bumper_status_.header;
  cylinder.lifetime = rclcpp::Duration(0, 10);
  cylinder.ns = "bumpers";
  cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
  cylinder.action = visualization_msgs::msg::Marker::ADD;
  cylinder.scale.x = 0.05;
  cylinder.scale.y = 0.05;
  cylinder.scale.z = 0.45;
  cylinder.pose.position.z = 0.03;

  // Change color depending on the state: red if activated, white otherwise
  if (bumper_status_.bumper_activated) {
    cylinder.color.r = 1.0;
    cylinder.color.g = 0.0;
    cylinder.color.b = 0.0;
    cylinder.color.a = 1.0;
  } else {
    cylinder.color.r = 1.0;
    cylinder.color.g = 1.0;
    cylinder.color.b = 1.0;
    cylinder.color.a = 1.0;
  }

  // Left bumper
  cylinder.id = 0;
  cylinder.pose.position.x = -0.10;
  cylinder.pose.position.y = 0.20;
  cylinder.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 1, 0}, M_PI / 2.0));
  markers_msg.markers.push_back(cylinder);

  // Right bumper
  cylinder.id = 1;
  cylinder.pose.position.x = -0.10;
  cylinder.pose.position.y = -0.20;
  cylinder.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 1, 0}, M_PI / 2.0));
  markers_msg.markers.push_back(cylinder);

  // Front bumper
  cylinder.id = 2;
  cylinder.pose.position.y = 0.0;
  cylinder.pose.position.x = 0.15;
  cylinder.pose.orientation = tf2::toMsg(tf2::Quaternion({1, 0, 0}, M_PI / 2.0));
  markers_msg.markers.push_back(cylinder);

  // Rear bumper
  cylinder.id = 3;
  cylinder.pose.position.y = 0.0;
  cylinder.pose.position.x = -0.35;
  cylinder.pose.orientation = tf2::toMsg(tf2::Quaternion({1, 0, 0}, M_PI / 2.0));
  markers_msg.markers.push_back(cylinder);

  bumper_markers_pub_->publish(markers_msg);
}

}  // namespace scitos2_modules

PLUGINLIB_EXPORT_CLASS(scitos2_modules::Drive, scitos2_core::Module)
