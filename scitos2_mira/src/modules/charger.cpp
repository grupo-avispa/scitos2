/*
 * SCITOS CHARGER
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos2_mira project.
 * 
 * All rights reserved.
 *
 */

// C++
#include <limits>

#include "scitos2_mira/modules/charger.hpp"

ScitosCharger::ScitosCharger() : ScitosModule("scitos_charger"){
}

void ScitosCharger::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & ros_node){
	node_ = ros_node;

	// Create ROS publishers
	auto node = node_.lock();
	battery_pub_ 		= node->create_publisher<sensor_msgs::msg::BatteryState>("battery", 20);
	charger_pub_ 		= node->create_publisher<scitos2_msgs::msg::ChargerStatus>(
							"charger_status", 20);

	// Create MIRA subscribers
	authority_.subscribe<mira::robot::BatteryState>("/robot/charger/Battery", 
		&ScitosCharger::battery_data_callback, this);
	authority_.subscribe<uint8>("/robot/charger/ChargerStatus", 
		&ScitosCharger::charger_status_callback, this);

	// Create ROS services
	save_persistent_errors_service_  = node->create_service<scitos2_msgs::srv::SavePersistentErrors>(
							"charger/save_persistent_errors", 
							std::bind(&ScitosCharger::save_persistent_errors, this, 
								std::placeholders::_1, std::placeholders::_2));
}

void ScitosCharger::reset_publishers(){
	battery_pub_.reset();
	charger_pub_.reset();
}

void ScitosCharger::battery_data_callback(mira::ChannelRead<mira::robot::BatteryState> data){
	// Get time data from mira
	rclcpp::Time time = rclcpp::Time(data->timestamp.toUnixNS());

	sensor_msgs::msg::BatteryState battery;

	battery.header.stamp = time;
	battery.header.frame_id = "base_link";
	battery.voltage = data->voltage;
	battery.temperature = std::numeric_limits<float>::quiet_NaN();
	battery.current = -data->current;
	battery.charge = (data->lifeTime == -1) ? std::numeric_limits<float>::quiet_NaN() : 
									(static_cast<float>(data->lifeTime) / 60.0 * battery.current);
	battery.capacity = std::numeric_limits<float>::quiet_NaN();
	battery.design_capacity = 40.0;
	battery.percentage = (data->lifePercent == 255) ? std::numeric_limits<float>::quiet_NaN() : 
													static_cast<float>(data->lifePercent) / 100.0;

	if (data->charging){
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
	}else if (battery.percentage == 1.0){
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
	}else if (data->powerSupplyPresent){
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
	}else if (battery.current < 0){
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
	}else{
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
	}

	battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
	battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
	battery.present = (data->cellVoltage).empty() ? false : true;
	battery.cell_voltage = data->cellVoltage;
	battery.cell_temperature.resize(battery.cell_voltage.size(), 
		std::numeric_limits<float>::quiet_NaN());
	battery.location = "Slot 1";
	battery.serial_number = "Unknown";

	battery_pub_->publish(battery);
}

void ScitosCharger::charger_status_callback(mira::ChannelRead<uint8> data){
	// Get time data from mira
	rclcpp::Time time = rclcpp::Time(data->timestamp.toUnixNS());

	scitos2_msgs::msg::ChargerStatus charger;

	charger.header.stamp = time;
	charger.header.frame_id = "base_link";
	charger.charging = (*data) & 1;
	charger.empty = (*data) & (1 << 1);
	charger.full = (*data) & (1 << 2);
	charger.derating = (*data) & (1 << 3);
	charger.charging_disabled = (*data) & (1 << 4);
	charger.const_volt_mode = (*data) & (1 << 5);
	charger.const_current_mode = (*data) & (1 << 6);
	charger.internal_error_flag = (*data) & (1 << 7);

	charger_pub_->publish(charger);
}

bool ScitosCharger::save_persistent_errors(
	const std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Request> request,
		std::shared_ptr<scitos2_msgs::srv::SavePersistentErrors::Response> response){
	RCLCPP_INFO_STREAM(logger_, "Saving persistent error log to '" << request->filename << "'");
	
	// Call mira service
	try{
		mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot", 
			std::string("savePersistentErrors"), request->filename);
	}catch (mira::XRPC& e){
		RCLCPP_INFO_STREAM(logger_, "Problem with RPC savePersistentErrors: " << e.message());
		return false;
	}

	return true;
}