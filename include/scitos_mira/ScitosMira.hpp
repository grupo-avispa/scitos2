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

#ifndef SCITOS_MIRA__SCITOS_MIRA_HPP_
#define SCITOS_MIRA__SCITOS_MIRA_HPP_

// C++
#include <string>

// MIRA
#include <fw/Framework.h>

// ROS
#include "rclcpp/rclcpp.hpp"

// Scitos Mira
#include "scitos_mira/ScitosModule.hpp"
#include "scitos_mira/RosLogSink.hpp"

/**
 * @brief Main class for the robot. Manages MIRA handle and ROS node.
 * 
 */
class ScitosMira: public rclcpp::Node{
	public:
		ScitosMira(const std::string& name);
		~ScitosMira();
	private:
		mira::Framework framework_;
		std::vector<std::string> args_;
		std::vector<std::string> modules_names_;
		std::vector<std::unique_ptr<ScitosModule> > modules_;

		void initialize();
};

#endif // SCITOS_MIRA__MIRA_CHARGER_HPP_

