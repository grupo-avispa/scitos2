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

// C++
#include <limits>

#include "scitos_mira/ScitosCharger.hpp"

ScitosCharger::ScitosCharger() : ScitosModule("scitos_charger"){
}

ScitosCallReturn ScitosCharger::on_configure(const rclcpp_lifecycle::State &){
	// Create ROS publishers
	battery_pub_ 		= this->create_publisher<sensor_msgs::msg::BatteryState>("battery", 20);
	charger_pub_ 		= this->create_publisher<scitos_msgs::msg::ChargerStatus>("charger_status", 20);

	// Create MIRA subscribers
	authority_.subscribe<mira::robot::BatteryState>("/robot/charger/Battery", &ScitosCharger::battery_data_callback, this);
	authority_.subscribe<uint8>("/robot/charger/ChargerStatus", &ScitosCharger::charger_status_callback, this);

	// Create ROS services
	save_persistent_errors_service_ 	= this->create_service<scitos_msgs::srv::SavePersistentErrors>("charger/save_persistent_errors", 
										std::bind(&ScitosCharger::save_persistent_errors, this, std::placeholders::_1, std::placeholders::_2));

	return ScitosCallReturn::SUCCESS;
}

ScitosCallReturn ScitosCharger::on_activate(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Activating the node...");

	// Explicitly activate the lifecycle publishers
	battery_pub_->on_activate();
	charger_pub_->on_activate();

	return ScitosCallReturn::SUCCESS;
}

ScitosCallReturn ScitosCharger::on_deactivate(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Deactivating the node...");

	// Explicitly deactivate the lifecycle publishers
	battery_pub_->on_deactivate();
	charger_pub_->on_deactivate();

	// Stops the main dispatcher thread
	authority_.stop();

	return ScitosCallReturn::SUCCESS;
}

ScitosCallReturn ScitosCharger::on_cleanup(const rclcpp_lifecycle::State &){
	RCLCPP_INFO(this->get_logger(), "Cleaning the node...");

	// Release the shared pointers
	battery_pub_.reset();
	charger_pub_.reset();

	return ScitosCallReturn::SUCCESS;
}

ScitosCallReturn ScitosCharger::on_shutdown(const rclcpp_lifecycle::State & state){
	RCLCPP_INFO(this->get_logger(), "Shutdown the node from state %s.", state.label().c_str());

	// Release the shared pointers
	battery_pub_.reset();
	charger_pub_.reset();

	// Checks out and invalidate the authority
	authority_.checkout();

	return ScitosCallReturn::SUCCESS;
}

void ScitosCharger::battery_data_callback(mira::ChannelRead<mira::robot::BatteryState> data){
	// Get time data from mira
	rclcpp::Time time = rclcpp::Time(data->timestamp.toUnixNS());

	sensor_msgs::msg::BatteryState battery;

	battery.header.stamp = time;
	battery.voltage = data->voltage;
	battery.current = data->current;
	battery.percentage = (data->lifePercent == 255) ? std::numeric_limits<float>::quiet_NaN() : 
													static_cast<float>(data->lifePercent) / 100.0;
	battery.cell_voltage = data->cellVoltage;
	battery.present = data->powerSupplyPresent;
	battery.capacity = static_cast<float>(data->lifeTime) / 60.0 * battery.current;
	
	if (data->charging){
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
	}else if (battery.percentage == 1.0){
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
	}else if (battery.current < 0){
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
	}else{
		battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
	}

	battery_pub_->publish(battery);
}

void ScitosCharger::charger_status_callback(mira::ChannelRead<uint8> data){
	// Get time data from mira
	rclcpp::Time time = rclcpp::Time(data->timestamp.toUnixNS());

	scitos_msgs::msg::ChargerStatus charger;

	charger.header.stamp = time;
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

bool ScitosCharger::save_persistent_errors(const std::shared_ptr<scitos_msgs::srv::SavePersistentErrors::Request> request,
								std::shared_ptr<scitos_msgs::srv::SavePersistentErrors::Response> response){
	RCLCPP_INFO_STREAM(this->get_logger(), "Saving persistent error log to '" << request->filename << "'");
	
	// Call mira service
	try{
		mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot", std::string("savePersistentErrors"), request->filename);
	}catch (mira::XRPC& e){
		RCLCPP_INFO_STREAM(this->get_logger(), "Problem with RPC savePersistentErrors: " << e.message());
		return false;
	}

	return true;
}