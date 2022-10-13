/*
 * SCITOS MIRA
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

// C++
#include <chrono>
#include <thread>

// BOOST
#include <boost/algorithm/string/join.hpp>

#include "scitos_mira/ScitosMira.hpp"
#include "scitos_mira/ModuleFactory.hpp"

ScitosMira::ScitosMira(const std::string& name) : Node(name), framework_(args_){
	RCLCPP_INFO(this->get_logger(), "Configuring the node...");

	// Redirect Mira logger
	MIRA_LOGGER.registerSink(RosLogSink(this->get_logger()));

	// Declare and read parameters
	declare_parameter_if_not_declared("modules", rclcpp::ParameterType::PARAMETER_STRING_ARRAY, 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("List of the modules exposed by the node"));
	this->get_parameter("modules", modules_names_);
	std::string joined = boost::algorithm::join(modules_names_, ", ");
	if (!joined.empty()){
		RCLCPP_INFO(this->get_logger(), "Loaded modules: [%s]", joined.c_str());
	}else{
		RCLCPP_ERROR(this->get_logger(), "Can't read parameter 'modules'. This MUST be supplied as a space separated list of SCITOS hardware modules to interface into ROS");
		exit(1);
	}

	std::string config;
	declare_parameter_if_not_declared("scitos_config", rclcpp::ParameterType::PARAMETER_STRING, 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("Configuration of the robot in XML format"));
	this->get_parameter("scitos_config", config);
	if (!config.empty()){
		RCLCPP_INFO(this->get_logger(), "Loaded scitos config: %s", config.c_str());
		framework_.load(config);
	}else{
		RCLCPP_ERROR(this->get_logger(), "Can't read parameter 'scitos_config'");
		exit(1);
	}

	// Sleep for 2 seconds and start Mira framework
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	framework_.start();

	// Create MIRA modules
	ModuleFactory& factory = ModuleFactory::get_instance();
	for (const auto& name : modules_names_){
		if (!factory.is_registered(name)){
			RCLCPP_ERROR_STREAM(this->get_logger(), "A non existent module named '" << name << "' was trying to be created.");
		}else{
			RCLCPP_INFO(this->get_logger(), "Loading module: [%s]", name.c_str());
			modules_.push_back(factory.create_module(name));
		}
	}

	initialize();
}

ScitosMira::~ScitosMira(){
}

void ScitosMira::initialize(){
	// Initialize all of the modules
	for (auto& module: modules_){
		module->initialize();
	}
}

void ScitosMira::declare_parameter_if_not_declared(const std::string & param_name, 
	const rclcpp::ParameterType & param_type,
	const rcl_interfaces::msg::ParameterDescriptor & param_descriptor){

	if (!this->has_parameter(param_name)){
		this->declare_parameter(param_name, param_type, param_descriptor);
	}
}