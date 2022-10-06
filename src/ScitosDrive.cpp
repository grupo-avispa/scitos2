/*
 * SCITOS DRIVE
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "scitos_mira/ScitosDrive.hpp"

ScitosDrive::ScitosDrive() : ScitosModule("scitos_drive"){
}

void ScitosDrive::initialize(){
	// Create ROS publishers
	bumper_pub_ 		= this->create_publisher<scitos_msgs::msg::BumperStatus>("bumper", 20);
	drive_status_pub_ 	= this->create_publisher<scitos_msgs::msg::DriveStatus>("drive_status", 20);
	emergency_stop_pub_ = this->create_publisher<scitos_msgs::msg::EmergencyStopStatus>("emergency_stop_status", rclcpp::QoS(20).transient_local());
	mileage_pub_ 		= this->create_publisher<scitos_msgs::msg::Mileage>("mileage", 20);
	odometry_pub_ 		= this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);

	// Create MIRA subscribers
	authority_.subscribe<mira::robot::Odometry2>("/robot/Odometry", &ScitosDrive::odometry_data_callback, this);
	authority_.subscribe<bool>("/robot/Bumper", &ScitosDrive::bumper_data_callback, this);
	authority_.subscribe<float>("/robot/Mileage", &ScitosDrive::mileage_data_callback, this);
	authority_.subscribe<uint32>("/robot/DriveStatusPlain", &ScitosDrive::drive_status_callback, this);

	// Create ROS subscribers
	cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1000, 
									std::bind(&ScitosDrive::velocity_command_callback, this, std::placeholders::_1));

	// Create ROS services
	change_force_service_ 		= this->create_service<scitos_msgs::srv::ChangeForce>("change_force", 
									std::bind(&ScitosDrive::change_force, this, std::placeholders::_1, std::placeholders::_2));
	emergency_stop_service_ 	= this->create_service<scitos_msgs::srv::EmergencyStop>("emergency_stop", 
									std::bind(&ScitosDrive::emergency_stop, this, std::placeholders::_1, std::placeholders::_2));
	enable_motors_service_ 		= this->create_service<scitos_msgs::srv::EnableMotors>("enable_motors", 
									std::bind(&ScitosDrive::enable_motors, this, std::placeholders::_1, std::placeholders::_2));
	reset_motor_stop_service_ 	= this->create_service<scitos_msgs::srv::ResetMotorStop>("reset_motorstop", 
									std::bind(&ScitosDrive::reset_motor_stop, this, std::placeholders::_1, std::placeholders::_2));
	reset_odometry_service_ 	= this->create_service<scitos_msgs::srv::ResetOdometry>("reset_odometry", 
									std::bind(&ScitosDrive::reset_odometry, this, std::placeholders::_1, std::placeholders::_2));
	suspend_bumper_service_ 	= this->create_service<scitos_msgs::srv::SuspendBumper>("suspend_bumper", 
									std::bind(&ScitosDrive::suspend_bumper, this, std::placeholders::_1, std::placeholders::_2));

	emergency_stop_.emergency_stop_activated = false;
}

void ScitosDrive::bumper_data_callback(mira::ChannelRead<bool> data){
	scitos_msgs::msg::BumperStatus status_msg;
	status_msg.header.stamp = this->now();
	status_msg.bumper_activated = data->value();
	bumper_pub_->publish(status_msg);
}

void ScitosDrive::drive_status_callback(mira::ChannelRead<uint32> data){
	scitos_msgs::msg::DriveStatus status_msg;
	status_msg.header.stamp = rclcpp::Time(data->timestamp.toUnixNS());
	status_msg.mode_normal = (bool)((*data) & 1);
	status_msg.mode_forced_stopped = (bool)((*data) & (1 << 1));
	status_msg.mode_freerun = (bool)((*data) & (1 << 2));

	status_msg.emergency_stop_activated = (bool)((*data) & (1 << 7));
	status_msg.emergency_stop_status = (bool)((*data) & (1 << 8));
	status_msg.bumper_front_activated = (bool)((*data) & (1 << 9));
	status_msg.bumper_front_status = (bool)((*data) & (1 << 10));
	status_msg.bumper_rear_activated = (bool)((*data) & (1 << 11));
	status_msg.bumper_rear_status = (bool)((*data) & (1 << 12));

	status_msg.safety_field_rear_laser = (bool)((*data) & (1 << 26));
	status_msg.safety_field_front_laser = (bool)((*data) & (1 << 27));

	RCLCPP_DEBUG(this->get_logger(), "Drive controller status (Nor: %i, MStop: %i, FreeRun: %i, EmBut: %i, BumpPres: %i, BusErr: %i, Stall: %i, InterErr: %i)",
			status_msg.mode_normal, status_msg.mode_forced_stopped, status_msg.mode_freerun, status_msg.emergency_stop_activated, status_msg.bumper_front_activated, status_msg.error_sifas_communication, status_msg.error_stall_mode, status_msg.error_sifas_internal);

	drive_status_pub_->publish(status_msg); 
}

void ScitosDrive::mileage_data_callback(mira::ChannelRead<float> data){
	scitos_msgs::msg::Mileage mileage_msg;
	mileage_msg.header.stamp = this->now();
	mileage_msg.distance = data->value();
	mileage_pub_->publish(mileage_msg);
}

void ScitosDrive::odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data){
	// Get odometry data from mira
	rclcpp::Time odom_time = rclcpp::Time(data->timestamp.toUnixNS());

	// Create quaternion from yaw
	geometry_msgs::msg::Quaternion orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, data->value().pose.phi()));

	// Publish as a nav_msgs::Odometry
	nav_msgs::msg::Odometry odom_msg;
	odom_msg.header.stamp = odom_time;
	odom_msg.header.frame_id = "/odom";
	odom_msg.child_frame_id = "base_footprint";

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
	tf_msg.header.frame_id = "/odom";
	tf_msg.child_frame_id = "base_footprint";

	tf_msg.transform.translation.x = data->value().pose.x();
	tf_msg.transform.translation.y = data->value().pose.y();
	tf_msg.transform.translation.z = 0.0;
	tf_msg.transform.rotation = orientation;

	// TODO: Check transform
	//tf_broadcaster_->sendTransform(tf_msg);
}

void ScitosDrive::velocity_command_callback(const geometry_msgs::msg::Twist& msg){
	if (!emergency_stop_.emergency_stop_activated) {
		mira::Velocity2 speed(msg.linear.x, 0, msg.angular.z);
		mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot", "setVelocity", speed);
		rpc.wait();
	}
}

bool ScitosDrive::change_force(const std::shared_ptr<scitos_msgs::srv::ChangeForce::Request> request,
								std::shared_ptr<scitos_msgs::srv::ChangeForce::Response> response){
	return set_mira_param("MotorController.Force",mira::toString(request->force));
}

bool ScitosDrive::emergency_stop(const std::shared_ptr<scitos_msgs::srv::EmergencyStop::Request> request,
								std::shared_ptr<scitos_msgs::srv::EmergencyStop::Response> response){
	// Publish emergency stop
	emergency_stop_.header.stamp = this->now();
	emergency_stop_.emergency_stop_activated = true;
	emergency_stop_pub_->publish(emergency_stop_);

	// Call mira service
	return call_mira_service("emergencyStop");
}

bool ScitosDrive::enable_motors(const std::shared_ptr<scitos_msgs::srv::EnableMotors::Request> request,
								std::shared_ptr<scitos_msgs::srv::EnableMotors::Response> response){
	// Call mira service
	mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot", std::string("enableMotors"),(bool)request->enable);
	rpc.timedWait(mira::Duration::seconds(1));
	rpc.get();
	RCLCPP_DEBUG(this->get_logger(), "Enable motor: %i", true);

	return true;
}

bool ScitosDrive::reset_motor_stop(const std::shared_ptr<scitos_msgs::srv::ResetMotorStop::Request> request,
								std::shared_ptr<scitos_msgs::srv::ResetMotorStop::Response> response){
	// Publish emergency stop
	emergency_stop_.header.stamp = this->now();
	emergency_stop_.emergency_stop_activated = false;
	emergency_stop_pub_->publish(emergency_stop_);

	// Call mira service
	return call_mira_service("resetMotorStop");
}

bool ScitosDrive::reset_odometry(const std::shared_ptr<scitos_msgs::srv::ResetOdometry::Request> request,
								std::shared_ptr<scitos_msgs::srv::ResetOdometry::Response> response){
	// Call mira service
	return call_mira_service("resetOdometry");
}

bool ScitosDrive::suspend_bumper(const std::shared_ptr<scitos_msgs::srv::SuspendBumper::Request> request,
								std::shared_ptr<scitos_msgs::srv::SuspendBumper::Response> response){
	// Call mira service
	return call_mira_service("suspendBumper");
}