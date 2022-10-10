/*
 * SCITOS EBC
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#include "scitos_mira/ScitosEBC.hpp"

ScitosEBC::ScitosEBC() : ScitosModule("scitos_ebc"){
}

void ScitosEBC::initialize(){
	// Declare and read parameters
	bool port_enabled;
	this->declare_parameter("mcu_5v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("mcu_5v_enabled", port_enabled);
	set_mira_param("MainControlUnit.EBC_5V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter mcu_5v_enabled is set to: %s", port_enabled ? "true" : "false");

	this->declare_parameter("mcu_12v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("mcu_12v_enabled", port_enabled);
	set_mira_param("MainControlUnit.EBC_12V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter mcu_12v_enabled is set to: %s", port_enabled ? "true" : "false");

	this->declare_parameter("mcu_24v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("mcu_24v_enabled", port_enabled);
	set_mira_param("MainControlUnit.EBC_24V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter mcu_24v_enabled is set to: %s", port_enabled ? "true" : "false");

	this->declare_parameter("port0_5v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("port0_5v_enabled", port_enabled);
	set_mira_param("EBC7.Port0_5V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter port0_5v_enabled is set to: %s", port_enabled ? "true" : "false");

	this->declare_parameter("port0_12v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("port0_12v_enabled", port_enabled);
	set_mira_param("EBC7.Port0_12V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter port0_12v_enabled is set to: %s", port_enabled ? "true" : "false");

	this->declare_parameter("port0_24v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("port0_24v_enabled", port_enabled);
	set_mira_param("EBC7.Port0_24V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter port0_24v_enabled is set to: %s", port_enabled ? "true" : "false");

	this->declare_parameter("port1_5v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("port1_5v_enabled", port_enabled);
	set_mira_param("EBC7.Port1_5V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter port1_5v_enabled is set to: %s", port_enabled ? "true" : "false");

	this->declare_parameter("port1_12v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("port1_12v_enabled", port_enabled);
	set_mira_param("EBC7.Port1_12V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter port1_12v_enabled is set to: %s", port_enabled ? "true" : "false");

	this->declare_parameter("port1_24v_enabled", rclcpp::ParameterValue(true));
	this->get_parameter("port1_24v_enabled", port_enabled);
	set_mira_param("EBC7.Port1_24V.Enabled", port_enabled ? "true" : "false");
	RCLCPP_INFO(this->get_logger(), "The parameter port1_24v_enabled is set to: %s", port_enabled ? "true" : "false");

	float port_max_current;
	this->declare_parameter("mcu_5v_max_current", rclcpp::ParameterValue(2.5));
	this->get_parameter("mcu_5v_max_current", port_max_current);
	set_mira_param("MainControlUnit.EBC_5V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter mcu_5v_max_current is set to: %f", port_max_current);

	this->declare_parameter("mcu_12v_max_current", rclcpp::ParameterValue(2.5));
	this->get_parameter("mcu_12v_max_current", port_max_current);
	set_mira_param("MainControlUnit.EBC_12V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter mcu_12v_max_current is set to: %f", port_max_current);

	this->declare_parameter("mcu_24v_max_current", rclcpp::ParameterValue(2.5));
	this->get_parameter("mcu_24v_max_current", port_max_current);
	set_mira_param("MainControlUnit.EBC_24V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter mcu_24v_max_current is set to: %f", port_max_current);

	this->declare_parameter("port0_5v_max_current", rclcpp::ParameterValue(2.5));
	this->get_parameter("port0_5v_max_current", port_max_current);
	set_mira_param("EBC7.Port0_5V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter port0_5v_max_current is set to: %f", port_max_current);

	this->declare_parameter("port0_12v_max_current", rclcpp::ParameterValue(2.5));
	this->get_parameter("port0_12v_max_current", port_max_current);
	set_mira_param("EBC7.Port0_12V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter port0_12v_max_current is set to: %f", port_max_current);

	this->declare_parameter("port0_24v_max_current", rclcpp::ParameterValue(2.5));
	this->get_parameter("port0_24v_max_current", port_max_current);
	set_mira_param("EBC7.Port0_24V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter port0_24v_max_current is set to: %f", port_max_current);

	this->declare_parameter("port1_5v_max_current", rclcpp::ParameterValue(2.5));
	this->get_parameter("port1_5v_max_current", port_max_current);
	set_mira_param("EBC7.Port1_5V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter port1_5v_max_current is set to: %f", port_max_current);

	this->declare_parameter("port1_12v_max_current", rclcpp::ParameterValue(2.5));
	this->get_parameter("port1_12v_max_current", port_max_current);
	set_mira_param("EBC7.Port1_12V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter port1_12v_max_current is set to: %f", port_max_current);

	this->declare_parameter("port1_24v_max_current", rclcpp::ParameterValue(4.0));
	this->get_parameter("port1_24v_max_current", port_max_current);
	set_mira_param("EBC7.Port1_24V.MaxCurrent", std::to_string(port_max_current));
	RCLCPP_INFO(this->get_logger(), "The parameter port1_24v_max_current is set to: %f", port_max_current);

	// Callback for monitor changes in parameters
	callback_handle_ = this->add_on_set_parameters_callback(
						std::bind(&ScitosEBC::parameters_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ScitosEBC::parameters_callback(
												const std::vector<rclcpp::Parameter> &parameters){
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	
	for (const auto &param: parameters){
		if (param.get_name() == "mcu_5v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("MainControlUnit.EBC_5V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter mcu_5v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "mcu_12v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("MainControlUnit.EBC_12V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter mcu_12v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "mcu_24v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("MainControlUnit.EBC_24V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter mcu_24v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port0_5v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("EBC7.Port0_5V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter port0_5v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port0_12v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("EBC7.Port0_12V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter port0_12v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port0_24v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("EBC7.Port0_24V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter port0_24v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port1_5v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("EBC7.Port1_5V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter port1_5v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port1_12v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("EBC7.Port1_12V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter port1_12v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port1_24v_enabled" && 
			param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
			try{
				set_mira_param("EBC7.Port1_24V.Enabled", param.as_bool() ? "true" : "false");
				RCLCPP_INFO(this->get_logger(), "The parameter port1_24v_enabled is set to: %s", param.as_bool() ? "true" : "false");
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "mcu_5v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 2.5){
			try{
				set_mira_param("MainControlUnit.EBC_5V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter mcu_5v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "mcu_12v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 2.5){
			try{
				set_mira_param("MainControlUnit.EBC_12V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter mcu_12v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "mcu_24v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 2.5){
			try{
				set_mira_param("MainControlUnit.EBC_24V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter mcu_24v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port0_5v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 2.5){
			try{
				set_mira_param("EBC7.EBC_5V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter port0_5v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port0_12v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 2.5){
			try{
				set_mira_param("EBC7.EBC_12V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter port0_12v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port0_24v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 2.5){
			try{
				set_mira_param("EBC7.EBC_24V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter port0_24v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port1_5v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 2.5){
			try{
				set_mira_param("EBC7.Port1_5V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter port1_5v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port1_12v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 4.0){
			try{
				set_mira_param("EBC7.Port1_12V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter port1_12v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}

		if (param.get_name() == "port1_24v_max_current" &&
			param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
			param.as_double() >= 0.0 && param.as_double() <= 4.0){
			try{
				set_mira_param("EBC7.Port1_24V.MaxCurrent", std::to_string(param.as_double()));
				RCLCPP_INFO(this->get_logger(), "The parameter port1_24v_max_current is set to: %f", param.as_double());
			}catch(mira::Exception& ex){
				result.successful = false;
				result.reason = "MIRA exception";
			}
		}
	}

	return result;
}
