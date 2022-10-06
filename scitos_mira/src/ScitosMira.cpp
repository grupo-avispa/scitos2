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

ScitosMira::ScitosMira(const std::string& name) : framework_(args_), Node(name, 
					rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)){
	RCLCPP_INFO(this->get_logger(), "Configuring the node...");

	// Redirect Mira logger
	MIRA_LOGGER.registerSink(RosLogSink(this->get_logger()));

	// Initialize node
	if (!this->has_parameter(("modules"))){
		RCLCPP_ERROR(this->get_logger(), "Can't read parameter 'modules'. This MUST be supplied as a space separated list of SCITOS hardware modules to interface into ROS");
		exit(1);
	}else{
		this->get_parameter("modules", modules_names_);
		std::string joined = boost::algorithm::join(modules_names_, ", ");
		RCLCPP_INFO(this->get_logger(), "Loaded modules: [%s]", joined.c_str());
	}
	if (!this->has_parameter(("config_file"))){
		RCLCPP_ERROR(this->get_logger(), "Can't read parameter 'config_file'");
		exit(1);
	}else{
		std::string config;
		this->get_parameter("config_file", config);
		RCLCPP_INFO(this->get_logger(), "Loaded config file: %s", config.c_str());
		framework_.load(config);
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