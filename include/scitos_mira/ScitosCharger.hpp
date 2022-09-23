/*
 * SCITOS CHARGER
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#ifndef SCITOS_MIRA__SCITOS_CHARGER_HPP_
#define SCITOS_MIRA__SCITOS_CHARGER_HPP_

// MIRA
#include <fw/Framework.h>
#include <robot/BatteryState.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

// SCITOS MSGS
#include "scitos_msgs/msg/charger_status.hpp"
#include "scitos_msgs/srv/save_persistent_errors.hpp"

#include "scitos_mira/ScitosModule.hpp"

/**
 * @brief Module for interfacing all the charger topics, service and params.
 * 
 */
class ScitosCharger : public ScitosModule{
	public:
		ScitosCharger();

		void battery_data_callback(mira::ChannelRead<mira::robot::BatteryState> data);
		void charger_status_callback(mira::ChannelRead<uint8> data);

		bool save_persistent_errors(const std::shared_ptr<scitos_msgs::srv::SavePersistentErrors::Request> request,
								std::shared_ptr<scitos_msgs::srv::SavePersistentErrors::Response> response);

	private:
		rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
		rclcpp::Publisher<scitos_msgs::msg::ChargerStatus>::SharedPtr charger_pub_;

		rclcpp::Service<scitos_msgs::srv::SavePersistentErrors>::SharedPtr save_persistent_errors_service_;
};

#endif // SCITOS_MIRA__SCITOS_CHARGER_HPP_

