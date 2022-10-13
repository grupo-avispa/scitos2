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

uint64 MAGNETIC_BARRIER_RFID_CODE = 0xabababab;

ScitosDrive::ScitosDrive() : ScitosModule("scitos_drive"){
}

void ScitosDrive::initialize(){
	// Create ROS publishers
	bumper_pub_ 		= this->create_publisher<scitos_msgs::msg::BumperStatus>("bumper", 20);
	drive_status_pub_ 	= this->create_publisher<scitos_msgs::msg::DriveStatus>("drive_status", 20);
	emergency_stop_pub_ = this->create_publisher<scitos_msgs::msg::EmergencyStopStatus>("emergency_stop_status", rclcpp::QoS(20).transient_local());
	magnetic_barrier_pub_	= this->create_publisher<scitos_msgs::msg::BarrierStatus>("barrier_status", rclcpp::QoS(20).transient_local());
	mileage_pub_ 		= this->create_publisher<scitos_msgs::msg::Mileage>("mileage", 20);
	odometry_pub_ 		= this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
	rfid_pub_ 			= this->create_publisher<scitos_msgs::msg::RfidTag>("rfid", 20);

	// Create MIRA subscribers
	authority_.subscribe<mira::robot::Odometry2>("/robot/Odometry", &ScitosDrive::odometry_data_callback, this);
	authority_.subscribe<bool>("/robot/Bumper", &ScitosDrive::bumper_data_callback, this);
	authority_.subscribe<float>("/robot/Mileage", &ScitosDrive::mileage_data_callback, this);
	authority_.subscribe<uint32>("/robot/DriveStatusPlain", &ScitosDrive::drive_status_callback, this);
	authority_.subscribe<uint64>("/robot/RFIDUserTag", &ScitosDrive::rfid_status_callback, this);

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
	enable_rfid_service_ 		= this->create_service<scitos_msgs::srv::EnableRfid>("enable_rfid", 
									std::bind(&ScitosDrive::enable_rfid, this, std::placeholders::_1, std::placeholders::_2));
	reset_barrier_stop_service_ = this->create_service<scitos_msgs::srv::ResetBarrierStop>("reset_barrier_stop", 
									std::bind(&ScitosDrive::reset_barrier_stop, this, std::placeholders::_1, std::placeholders::_2));
	reset_motor_stop_service_ 	= this->create_service<scitos_msgs::srv::ResetMotorStop>("reset_motorstop", 
									std::bind(&ScitosDrive::reset_motor_stop, this, std::placeholders::_1, std::placeholders::_2));
	reset_odometry_service_ 	= this->create_service<scitos_msgs::srv::ResetOdometry>("reset_odometry", 
									std::bind(&ScitosDrive::reset_odometry, this, std::placeholders::_1, std::placeholders::_2));
	suspend_bumper_service_ 	= this->create_service<scitos_msgs::srv::SuspendBumper>("suspend_bumper", 
									std::bind(&ScitosDrive::suspend_bumper, this, std::placeholders::_1, std::placeholders::_2));

	// Declare and read parameters
	declare_parameter_if_not_declared("base_frame", rclcpp::ParameterValue("base_footprint"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The name of the base frame of the robot"));
	this->get_parameter("base_frame", base_frame_);
	RCLCPP_INFO(this->get_logger(), "The parameter base_frame is set to: %s", base_frame_.c_str());

	bool magnetic_barrier_enabled = true;
	declare_parameter_if_not_declared("magnetic_barrier_enabled", rclcpp::ParameterValue(false), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Enable the magnetic strip detector to cut out the motors"));
	this->get_parameter("magnetic_barrier_enabled", magnetic_barrier_enabled);
	RCLCPP_INFO(this->get_logger(), "The parameter magnetic_barrier_enabled is set to: %s", magnetic_barrier_enabled ? "true" : "false");

	try{
		set_mira_param("MainControlUnit.RearLaser.Enabled", magnetic_barrier_enabled ? "true" : "false");
	}catch(mira::Exception& ex){}

	// Callback for monitor changes in parameters
	callback_handle_ = this->add_on_set_parameters_callback(
						std::bind(&ScitosDrive::parameters_callback, this, std::placeholders::_1));

	emergency_stop_.emergency_stop_activated = false;
	barrier_status_.barrier_stopped = false;
	barrier_status_.last_detection_stamp = rclcpp::Time(0);
	magnetic_barrier_pub_->publish(barrier_status_);

	// Initialize the transform broadcaster
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

rcl_interfaces::msg::SetParametersResult ScitosDrive::parameters_callback(
												const std::vector<rclcpp::Parameter> &parameters){
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	
	for (const auto &param: parameters){
		if (param.get_name() == "magnetic_barrier_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("MainControlUnit.RearLaser.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter magnetic_barrier_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "base_frame" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			base_frame_ = param.as_string();
			RCLCPP_INFO(this->get_logger(), "The parameter base_frame is set to: %s", base_frame_.c_str());
		}
	}

	return result;
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

	RCLCPP_DEBUG(this->get_logger(), "Drive controller status (Nor: %i, MStop: %i, FreeRun: %i, EmBut: %i, BumpPres: %i, BusErr: %i, Stall: %i, InterErr: %i)",
			status_msg.mode_normal, status_msg.mode_forced_stopped, 
			status_msg.mode_freerun, status_msg.emergency_stop_activated, 
			status_msg.bumper_front_activated, status_msg.error_sifas_communication, 
			status_msg.error_stall_mode, status_msg.error_sifas_internal);

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
	tf_msg.header.frame_id = "/odom";
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
		mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot", "setVelocity", speed);
		rpc.wait();
	}
}

void ScitosDrive::rfid_status_callback(mira::ChannelRead<uint64> data){
	scitos_msgs::msg::RfidTag tag_msg;
	if (data->value() == MAGNETIC_BARRIER_RFID_CODE) {
		barrier_status_.barrier_stopped = true;
		barrier_status_.last_detection_stamp = rclcpp::Time(data->timestamp.toUnixNS());
		magnetic_barrier_pub_->publish(barrier_status_);
	}
	tag_msg.tag = data->value();
	rfid_pub_->publish(tag_msg);
}

bool ScitosDrive::change_force(const std::shared_ptr<scitos_msgs::srv::ChangeForce::Request> request,
								std::shared_ptr<scitos_msgs::srv::ChangeForce::Response> response){
	return set_mira_param("MotorController.Force", mira::toString(request->force));
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
	mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot", std::string("enableMotors"), 
																static_cast<bool>(request->enable));
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

bool ScitosDrive::enable_rfid(const std::shared_ptr<scitos_msgs::srv::EnableRfid::Request> request,
								std::shared_ptr<scitos_msgs::srv::EnableRfid::Response> response){
	if (request->enable == true){
		return set_mira_param("MainControlUnit.RearLaser.Enabled", "true");
	}else{
		return set_mira_param("MainControlUnit.RearLaser.Enabled", "false");
	}
	return false;
}

bool ScitosDrive::reset_barrier_stop(const std::shared_ptr<scitos_msgs::srv::ResetBarrierStop::Request> request,
								std::shared_ptr<scitos_msgs::srv::ResetBarrierStop::Response> response){
	barrier_status_.barrier_stopped = false;
	magnetic_barrier_pub_->publish(barrier_status_);
	return true;
}