/*
 * SCITOS DRIVE
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/node_utils.hpp"

#include "scitos_mira/modules/ScitosDrive.hpp"

uint64 MAGNETIC_BARRIER_RFID_CODE = 0xabababab;

using namespace std::placeholders;

ScitosDrive::ScitosDrive() : ScitosModule("scitos_drive"){
}

void ScitosDrive::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & ros_node){
	node_ = ros_node;

	// Create ROS publishers
	auto node = node_.lock();
	bumper_pub_ 		= node->create_publisher<scitos_msgs::msg::BumperStatus>("bumper", 20);
	bumper_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
							"bumper_viz", 20);
	drive_status_pub_ 	= node->create_publisher<scitos_msgs::msg::DriveStatus>(
							"drive_status", 20);
	emergency_stop_pub_ = node->create_publisher<scitos_msgs::msg::EmergencyStopStatus>(
							"emergency_stop_status", rclcpp::QoS(1).transient_local());
	magnetic_barrier_pub_	= node->create_publisher<scitos_msgs::msg::BarrierStatus>(
							"barrier_status", rclcpp::QoS(1).transient_local());
	mileage_pub_ 		= node->create_publisher<scitos_msgs::msg::Mileage>("mileage", 20);
	odometry_pub_ 		= node->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
	rfid_pub_ 			= node->create_publisher<scitos_msgs::msg::RfidTag>("rfid", 20);

	// Create MIRA subscribers
	authority_.subscribe<mira::robot::Odometry2>("/robot/Odometry", 
		&ScitosDrive::odometry_data_callback, this);
	authority_.subscribe<bool>("/robot/Bumper", &ScitosDrive::bumper_data_callback, this);
	authority_.subscribe<float>("/robot/Mileage", &ScitosDrive::mileage_data_callback, this);
	authority_.subscribe<uint32>("/robot/DriveStatusPlain", 
		&ScitosDrive::drive_status_callback, this);
	authority_.subscribe<uint64>("/robot/RFIDUserTag", &ScitosDrive::rfid_status_callback, this);

	// Create ROS subscribers
	cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1000, 
									std::bind(&ScitosDrive::velocity_command_callback, this, _1));

	// Create ROS services
	change_force_service_ 		= node->create_service<scitos_msgs::srv::ChangeForce>(
		"change_force", std::bind(&ScitosDrive::change_force, this, _1, _2));
	emergency_stop_service_ 	= node->create_service<scitos_msgs::srv::EmergencyStop>(
		"emergency_stop", std::bind(&ScitosDrive::emergency_stop, this, _1, _2));
	enable_motors_service_ 		= node->create_service<scitos_msgs::srv::EnableMotors>(
		"enable_motors", std::bind(&ScitosDrive::enable_motors, this, _1, _2));
	enable_rfid_service_ 		= node->create_service<scitos_msgs::srv::EnableRfid>(
		"enable_rfid", std::bind(&ScitosDrive::enable_rfid, this, _1, _2));
	reset_barrier_stop_service_ = node->create_service<scitos_msgs::srv::ResetBarrierStop>(
		"reset_barrier_stop", std::bind(&ScitosDrive::reset_barrier_stop, this, _1, _2));
	reset_motor_stop_service_ 	= node->create_service<scitos_msgs::srv::ResetMotorStop>(
		"reset_motorstop", std::bind(&ScitosDrive::reset_motor_stop, this, _1, _2));
	reset_odometry_service_ 	= node->create_service<scitos_msgs::srv::ResetOdometry>(
		"reset_odometry", std::bind(&ScitosDrive::reset_odometry, this, _1, _2));
	suspend_bumper_service_ 	= node->create_service<scitos_msgs::srv::SuspendBumper>(
		"suspend_bumper", std::bind(&ScitosDrive::suspend_bumper, this, _1, _2));

	// Declare and read parameters
	nav2_util::declare_parameter_if_not_declared(node, "drive.base_frame", 
		rclcpp::ParameterValue("base_footprint"), rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The name of the base frame of the robot"));
	node->get_parameter("drive.base_frame", base_frame_);
	RCLCPP_INFO(logger_, "The parameter drive.base_frame is set to: %s", base_frame_.c_str());

	bool magnetic_barrier_enabled = true;
	nav2_util::declare_parameter_if_not_declared(node, "drive.magnetic_barrier_enabled", 
		rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Enable the magnetic strip detector to cut out the motors"));
	node->get_parameter("drive.magnetic_barrier_enabled", magnetic_barrier_enabled);
	RCLCPP_INFO(logger_, "The parameter drive.magnetic_barrier_enabled is set to: %s", 
		magnetic_barrier_enabled ? "true" : "false");

	try{
		set_mira_param("MainControlUnit.RearLaser.Enabled", 
			magnetic_barrier_enabled ? "true" : "false");
	}catch(mira::Exception& ex){}

	// Callback for monitor changes in parameters
	dyn_params_handler_ = node->add_on_set_parameters_callback(
						std::bind(&ScitosDrive::parameters_callback, this, _1));

	// Publish initial values
	emergency_stop_.header.frame_id = base_frame_;
	emergency_stop_.header.stamp = node->now();
	emergency_stop_.emergency_stop_activated = false;
	emergency_stop_pub_->publish(emergency_stop_);

	// Publish initial values
	barrier_status_.header.frame_id = base_frame_;
	barrier_status_.header.stamp = node->now();
	barrier_status_.barrier_stopped = false;
	barrier_status_.last_detection_stamp = rclcpp::Time(0);
	magnetic_barrier_pub_->publish(barrier_status_);

	// Initialize the transform broadcaster
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
}

void ScitosDrive::reset_publishers(){
	bumper_pub_.reset();
	bumper_markers_pub_.reset();
	drive_status_pub_.reset();
	emergency_stop_pub_.reset();
	magnetic_barrier_pub_.reset();
	mileage_pub_.reset();
	odometry_pub_.reset();
	rfid_pub_.reset();
}

rcl_interfaces::msg::SetParametersResult ScitosDrive::parameters_callback(
												const std::vector<rclcpp::Parameter> &parameters){
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	
	for (const auto &param: parameters){
		if (param.get_name() == "drive.magnetic_barrier_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("MainControlUnit.RearLaser.Enabled", 
					param.as_bool() ? "true" : "false");
				RCLCPP_INFO(logger_, "The parameter drive.magnetic_barrier_enabled is set to: %s", 
					param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "drive.base_frame" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			base_frame_ = param.as_string();
			RCLCPP_INFO(logger_, "The parameter drive.base_frame is set to: %s", base_frame_.c_str());
		}
	}

	return result;
}

void ScitosDrive::bumper_data_callback(mira::ChannelRead<bool> data){
	scitos_msgs::msg::BumperStatus status_msg;
	status_msg.header.frame_id = base_frame_;
	status_msg.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
	status_msg.bumper_activated = data->value();
	bumper_pub_->publish(status_msg);

	// Reset motorstop if bumper is activated at startup
	if (!setup_ && status_msg.bumper_activated){
		call_mira_service("resetMotorStop");
		setup_ = true;
	}

	// Note: Only perform visualization if there's any subscriber
	if (bumper_markers_pub_->get_subscription_count() == 0) return;

	// Publish markers
	visualization_msgs::msg::MarkerArray bumper_markers_;
	visualization_msgs::msg::Marker cylinder;
	cylinder.header = status_msg.header;
	cylinder.lifetime = rclcpp::Duration(0, 10);
	cylinder.ns = "bumpers";
	cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
	cylinder.action = visualization_msgs::msg::Marker::ADD;
	cylinder.scale.x = 0.05;
	cylinder.scale.y = 0.05;
	cylinder.scale.z = 0.45;
	cylinder.pose.position.z = 0.03;

	// Change color depending on the state: red if activated, white otherwise
	if (data->value()){
		cylinder.color.r = 1.0;
		cylinder.color.g = 0.0;
		cylinder.color.b = 0.0;
		cylinder.color.a = 1.0;
	}else{
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
	bumper_markers_.markers.push_back(cylinder);

	// Right bumper
	cylinder.id = 1;
	cylinder.pose.position.x = -0.10;
	cylinder.pose.position.y = -0.20;
	cylinder.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 1, 0}, M_PI / 2.0));
	bumper_markers_.markers.push_back(cylinder);

	// Front bumper
	cylinder.id = 2;
	cylinder.pose.position.y = 0.0;
	cylinder.pose.position.x = 0.15;
	cylinder.pose.orientation = tf2::toMsg(tf2::Quaternion({1, 0, 0}, M_PI / 2.0));
	bumper_markers_.markers.push_back(cylinder);

	// Rear bumper
	cylinder.id = 3;
	cylinder.pose.position.y = 0.0;
	cylinder.pose.position.x = -0.35;
	cylinder.pose.orientation = tf2::toMsg(tf2::Quaternion({1, 0, 0}, M_PI / 2.0));
	bumper_markers_.markers.push_back(cylinder);

	bumper_markers_pub_->publish(bumper_markers_);
}

void ScitosDrive::drive_status_callback(mira::ChannelRead<uint32> data){
	scitos_msgs::msg::DriveStatus status_msg;
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
	if (status_msg.emergency_stop_activated && !status_msg.emergency_stop_status){
		call_mira_service("resetMotorStop");
	}

	RCLCPP_DEBUG_STREAM(logger_, "Drive controller status "
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

void ScitosDrive::mileage_data_callback(mira::ChannelRead<float> data){
	scitos_msgs::msg::Mileage mileage_msg;
	mileage_msg.header.frame_id = base_frame_;
	mileage_msg.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
	mileage_msg.distance = data->value();
	mileage_pub_->publish(mileage_msg);
}

void ScitosDrive::odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data){
	// Get odometry data from mira
	rclcpp::Time odom_time = rclcpp::Time(data->timestamp.toUnixNS());

	// Create quaternion from yaw
	geometry_msgs::msg::Quaternion orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 
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

void ScitosDrive::velocity_command_callback(const geometry_msgs::msg::Twist& msg){
	if (!emergency_stop_.emergency_stop_activated) {
		mira::Velocity2 speed(msg.linear.x, 0, msg.angular.z);
		mira::RPCFuture<void> rpc = authority_.callService<void>(
			"/robot/Robot", "setVelocity", speed);
		rpc.wait();
	}
}

void ScitosDrive::rfid_status_callback(mira::ChannelRead<uint64> data){
	scitos_msgs::msg::RfidTag tag_msg;
	if (data->value() == MAGNETIC_BARRIER_RFID_CODE){
		barrier_status_.header.frame_id = base_frame_;
		barrier_status_.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
		barrier_status_.barrier_stopped = true;
		barrier_status_.last_detection_stamp = rclcpp::Time(data->timestamp.toUnixNS());
		magnetic_barrier_pub_->publish(barrier_status_);
	}
	tag_msg.tag = data->value();
	rfid_pub_->publish(tag_msg);
}

bool ScitosDrive::change_force(
	const std::shared_ptr<scitos_msgs::srv::ChangeForce::Request> request,
	std::shared_ptr<scitos_msgs::srv::ChangeForce::Response> response){

	return set_mira_param("MotorController.Force", mira::toString(request->force));
}

bool ScitosDrive::emergency_stop(
	const std::shared_ptr<scitos_msgs::srv::EmergencyStop::Request> request,
	std::shared_ptr<scitos_msgs::srv::EmergencyStop::Response> response){

	auto node = node_.lock();

	// Publish emergency stop
	emergency_stop_.header.frame_id = base_frame_;
	emergency_stop_.header.stamp = node->now();
	emergency_stop_.emergency_stop_activated = true;
	emergency_stop_pub_->publish(emergency_stop_);

	// Call mira service
	return call_mira_service("emergencyStop");
}

bool ScitosDrive::enable_motors(
	const std::shared_ptr<scitos_msgs::srv::EnableMotors::Request> request,
	std::shared_ptr<scitos_msgs::srv::EnableMotors::Response> response){

	// Call mira service
	mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot", 
		std::string("enableMotors"), static_cast<bool>(request->enable));
	rpc.timedWait(mira::Duration::seconds(1));
	rpc.get();
	RCLCPP_DEBUG(logger_, "Enable motor: %i", true);

	return true;
}

bool ScitosDrive::reset_motor_stop(
	const std::shared_ptr<scitos_msgs::srv::ResetMotorStop::Request> request,
	std::shared_ptr<scitos_msgs::srv::ResetMotorStop::Response> response){

	// Publish emergency stop
	auto node = node_.lock();
	emergency_stop_.header.frame_id = base_frame_;
	emergency_stop_.header.stamp = node->now();
	emergency_stop_.emergency_stop_activated = false;
	emergency_stop_pub_->publish(emergency_stop_);

	// Call mira service
	return call_mira_service("resetMotorStop");
}

bool ScitosDrive::reset_odometry(
	const std::shared_ptr<scitos_msgs::srv::ResetOdometry::Request> request,
	std::shared_ptr<scitos_msgs::srv::ResetOdometry::Response> response){

	// Call mira service
	return call_mira_service("resetOdometry");
}

bool ScitosDrive::suspend_bumper(
	const std::shared_ptr<scitos_msgs::srv::SuspendBumper::Request> request,
	std::shared_ptr<scitos_msgs::srv::SuspendBumper::Response> response){

	// Call mira service
	return call_mira_service("suspendBumper");
}

bool ScitosDrive::enable_rfid(const std::shared_ptr<scitos_msgs::srv::EnableRfid::Request> request,
								std::shared_ptr<scitos_msgs::srv::EnableRfid::Response> response){
	if (request->enable == true){
		return set_mira_param("MainControlUnit.RearLaser.Enabled", "true");
	}else{
		return set_mira_param("MainControlUnit.RearLaser.Enabled", "false");
	}
	return false;
}

bool ScitosDrive::reset_barrier_stop(
	const std::shared_ptr<scitos_msgs::srv::ResetBarrierStop::Request> request,
	std::shared_ptr<scitos_msgs::srv::ResetBarrierStop::Response> response){

	auto node = node_.lock();

	barrier_status_.header.frame_id = base_frame_;
	barrier_status_.header.stamp = node->now();
	barrier_status_.barrier_stopped = false;
	magnetic_barrier_pub_->publish(barrier_status_);
	return true;
}