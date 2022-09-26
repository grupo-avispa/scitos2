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

// BOOST
#include <boost/algorithm/string/join.hpp>

#include "scitos_mira/ScitosMira.hpp"

ScitosMira::ScitosMira(const std::string& name) : framework_(args_), Node(name, 
					rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)){
	RCLCPP_INFO(this->get_logger(), "Configuring the node...");

	// Redirect Mira logger
	MIRA_LOGGER.registerSink(RosLogSink());

	// Initialize node
	if (!this->has_parameter(("modules"))){
		RCLCPP_ERROR(this->get_logger(), "Can't read parameter 'modules'. This MUST be supplied as a space separated list of SCITOS hardware modules to interface into ROS");
	}else{
		this->get_parameter("modules", modules_names_);
		std::string joined = boost::algorithm::join(modules_names_, ", ");
		RCLCPP_INFO(this->get_logger(), "Loaded modules: [%s]", joined.c_str());
	}

	if (!this->has_parameter(("config_file"))){
		RCLCPP_ERROR(this->get_logger(), "Can't read parameter 'config_file'");
	}else{
		std::string config;
		this->get_parameter("config_file", config);
		RCLCPP_INFO(this->get_logger(), "Loaded config file: %s", config.c_str());
		framework_.load(config);
	}

	// TODO: Replace by STL
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
	framework_.start();
}

ScitosMira::~ScitosMira(){
}