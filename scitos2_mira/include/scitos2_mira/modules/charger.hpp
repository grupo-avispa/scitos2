/*
 * SCITOS CHARGER
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#ifndef SCITOS_MIRA__SCITOS_CHARGER_HPP_
#define SCITOS_MIRA__SCITOS_CHARGER_HPP_

// MIRA
#include <robot/BatteryState.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

// SCITOS MSGS
#include "scitos2_msgs/msg/charger_status.hpp"
#include "scitos2_msgs/srv/save_persistent_errors.hpp"

#include "scitos2_mira/module.hpp"

/**
 * @brief Module for interfacing all the charger topics, service and params.
 * 
 */
class ScitosCharger : public ScitosModule{
	public:
		static std::shared_ptr<ScitosModule> Create() {
			return std::shared_ptr<ScitosModule>(new ScitosCharger());
		}

		void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & ros_node);
		void reset_publishers();

	private:
		rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
		rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::ChargerStatus>::SharedPtr charger_pub_;
		
		rclcpp::Service<scitos2_msgs::srv::SavePersistentErrors>::SharedPtr 
			save_persistent_errors_service_;
		
		ScitosCharger();

		void battery_data_callback(mira::ChannelRead<mira::robot::BatteryState> data);
		void charger_status_callback(mira::ChannelRead<uint8> data);

		bool save_persistent_errors(
			const std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Request> request,
			std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Response> response);
};

#endif // SCITOS_MIRA__SCITOS_CHARGER_HPP_
