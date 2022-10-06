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

#ifndef SCITOS_MIRA__SCITOS_DRIVE_HPP_
#define SCITOS_MIRA__SCITOS_DRIVE_HPP_

// MIRA
#include <robot/Odometry.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

// SCITOS MSGS
#include "scitos_msgs/msg/bumper_status.hpp"
#include "scitos_msgs/msg/drive_status.hpp"
#include "scitos_msgs/msg/emergency_stop_status.hpp"
#include "scitos_msgs/msg/mileage.hpp"
#include "scitos_msgs/srv/change_force.hpp"
#include "scitos_msgs/srv/emergency_stop.hpp"
#include "scitos_msgs/srv/enable_motors.hpp"
#include "scitos_msgs/srv/reset_motor_stop.hpp"
#include "scitos_msgs/srv/reset_odometry.hpp"
#include "scitos_msgs/srv/suspend_bumper.hpp"

#include "scitos_mira/ScitosModule.hpp"

/**
 * @brief Module for interfacing all related to drive: odometry, motor controller, state etc.
 * 
 */
class ScitosDrive : public ScitosModule{
	public:
		static std::shared_ptr<ScitosModule> Create() {
			return std::shared_ptr<ScitosModule>(new ScitosDrive());
		}

		void initialize();

	private:
		rclcpp::Publisher<scitos_msgs::msg::BumperStatus>::SharedPtr bumper_pub_;
		rclcpp::Publisher<scitos_msgs::msg::DriveStatus>::SharedPtr drive_status_pub_;
		rclcpp::Publisher<scitos_msgs::msg::EmergencyStopStatus>::SharedPtr emergency_stop_pub_;
		rclcpp::Publisher<scitos_msgs::msg::Mileage>::SharedPtr mileage_pub_;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
			
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

		rclcpp::Service<scitos_msgs::srv::ChangeForce>::SharedPtr change_force_service_;
		rclcpp::Service<scitos_msgs::srv::EmergencyStop>::SharedPtr emergency_stop_service_;
		rclcpp::Service<scitos_msgs::srv::EnableMotors>::SharedPtr enable_motors_service_;
		rclcpp::Service<scitos_msgs::srv::ResetMotorStop>::SharedPtr reset_motor_stop_service_;
		rclcpp::Service<scitos_msgs::srv::ResetOdometry>::SharedPtr reset_odometry_service_;
		rclcpp::Service<scitos_msgs::srv::SuspendBumper>::SharedPtr suspend_bumper_service_;

		scitos_msgs::msg::EmergencyStopStatus emergency_stop_;
		std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

		ScitosDrive();

		void bumper_data_callback(mira::ChannelRead<bool> data);
		void drive_status_callback(mira::ChannelRead<uint32> data);
		void mileage_data_callback(mira::ChannelRead<float> data);
		void odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data);

		void velocity_command_callback(const geometry_msgs::msg::Twist& msg);

		bool change_force(const std::shared_ptr<scitos_msgs::srv::ChangeForce::Request> request,
								std::shared_ptr<scitos_msgs::srv::ChangeForce::Response> response);
		bool emergency_stop(const std::shared_ptr<scitos_msgs::srv::EmergencyStop::Request> request,
								std::shared_ptr<scitos_msgs::srv::EmergencyStop::Response> response);
		bool enable_motors(const std::shared_ptr<scitos_msgs::srv::EnableMotors::Request> request,
								std::shared_ptr<scitos_msgs::srv::EnableMotors::Response> response);
		bool reset_motor_stop(const std::shared_ptr<scitos_msgs::srv::ResetMotorStop::Request> request,
								std::shared_ptr<scitos_msgs::srv::ResetMotorStop::Response> response);
		bool reset_odometry(const std::shared_ptr<scitos_msgs::srv::ResetOdometry::Request> request,
								std::shared_ptr<scitos_msgs::srv::ResetOdometry::Response> response);
		bool suspend_bumper(const std::shared_ptr<scitos_msgs::srv::SuspendBumper::Request> request,
								std::shared_ptr<scitos_msgs::srv::SuspendBumper::Response> response);
};

#endif // SCITOS_MIRA__SCITOS_DRIVE_HPP_

