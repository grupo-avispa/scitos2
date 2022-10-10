/*
 * SCITOS DISPLAY
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#include "scitos_mira/ScitosDisplay.hpp"

ScitosDisplay::ScitosDisplay() : ScitosModule("scitos_display"){
}

void ScitosDisplay::initialize(){
	// Create publisher
	display_data_pub_ = this->create_publisher<scitos_msgs::msg::MenuEntry>("user_menu_selected", 1);

	// Callback for monitor changes in parameters
	callback_handle_ = this->add_on_set_parameters_callback(
						std::bind(&ScitosDisplay::parameters_callback, this, std::placeholders::_1));

	// Create MIRA subscriber
	authority_.subscribe<uint8>("/robot/StatusDisplayUserMenuEvent", &ScitosDisplay::menu_data_callback, this);

	// Declare and read parameters
	this->declare_parameter("user_menu_enabled", rclcpp::ParameterValue(false));
	this->get_parameter("user_menu_enabled", user_menu_enabled_);
	this->declare_parameter("menu_name", rclcpp::ParameterValue("User Menu"));
	this->declare_parameter("menu_entry_name_1", rclcpp::ParameterValue("Entry 1"));
	this->declare_parameter("menu_entry_name_2", rclcpp::ParameterValue("Entry 2"));
	this->declare_parameter("menu_entry_name_3", rclcpp::ParameterValue("Entry 3"));

	if (user_menu_enabled_){
		set_mira_param("StatusDisplay.EnableUserMenu", "true");

		// Read menu entries
		std::string menu_entry;
		this->get_parameter("menu_name", menu_entry);
		set_mira_param("StatusDisplay.UserMenuName", menu_entry);
		RCLCPP_INFO(this->get_logger(), "The parameter menu_name is set to: %s", menu_entry.c_str());
		
		this->get_parameter("menu_entry_name_1", menu_entry);
		set_mira_param("StatusDisplay.UserMenuEntryName1", menu_entry);
		RCLCPP_INFO(this->get_logger(), "The parameter menu_entry_name_1 is set to: %s", menu_entry.c_str());
		
		this->get_parameter("menu_entry_name_2", menu_entry);
		set_mira_param("StatusDisplay.UserMenuEntryName2", menu_entry);
		RCLCPP_INFO(this->get_logger(), "The parameter menu_entry_name_2 is set to: %s", menu_entry.c_str());
		
		this->get_parameter("menu_entry_name_3", menu_entry);
		set_mira_param("StatusDisplay.UserMenuEntryName3", menu_entry);
		RCLCPP_INFO(this->get_logger(), "The parameter menu_entry_name_3 is set to: %s", menu_entry.c_str());
	}else{
		set_mira_param("StatusDisplay.EnableUserMenu", "false");
		RCLCPP_INFO(this->get_logger(), "The parameter user_menu_enabled is set to: false");
	}
}

rcl_interfaces::msg::SetParametersResult ScitosDisplay::parameters_callback(
												const std::vector<rclcpp::Parameter> &parameters){
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";

	for (const auto &param: parameters){
		if (param.get_name() == "user_menu_enabled" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			user_menu_enabled_ = param.as_bool();
			try{
				set_mira_param("StatusDisplay.EnableUserMenu", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter user_menu_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "menu_name" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			if (user_menu_enabled_){
				try{
					set_mira_param("StatusDisplay.UserMenuName", param.as_string());
					RCLCPP_INFO(this->get_logger(), "The parameter menu_name is set to: %s", param.as_string().c_str());
				}catch(mira::Exception& ex){
					result.successful = false;
					result.reason = "MIRA exception";
				}
			}
		}

		if (param.get_name() == "menu_entry_name_1" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			if (user_menu_enabled_){
				try{
					set_mira_param("StatusDisplay.UserMenuEntryName1", param.as_string());
					RCLCPP_INFO(this->get_logger(), "The parameter menu_entry_name_1 is set to: %s", param.as_string().c_str());
				}catch(mira::Exception& ex){
					result.successful = false;
					result.reason = "MIRA exception";
				}
			}
		}

		if (param.get_name() == "menu_entry_name_2" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			if (user_menu_enabled_){
				try{
					set_mira_param("StatusDisplay.UserMenuEntryName2", param.as_string());
					RCLCPP_INFO(this->get_logger(), "The parameter menu_entry_name_2 is set to: %s", param.as_string().c_str());
				}catch(mira::Exception& ex){
					result.successful = false;
					result.reason = "MIRA exception";
				}
			}
		}

		if (param.get_name() == "menu_entry_name_3" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
			if (user_menu_enabled_){
				try{
					set_mira_param("StatusDisplay.UserMenuEntryName3", param.as_string());
					RCLCPP_INFO(this->get_logger(), "The parameter menu_entry_name_3 is set to: %s", param.as_string().c_str());
				}catch(mira::Exception& ex){
					result.successful = false;
					result.reason = "MIRA exception";
				}
			}
		}
	}
}

void ScitosDisplay::menu_data_callback(mira::ChannelRead<uint8> data){
	scitos_msgs::msg::MenuEntry msg;
	msg.header.stamp = this->now();
	msg.entry = data->value();
	display_data_pub_->publish(msg);
}