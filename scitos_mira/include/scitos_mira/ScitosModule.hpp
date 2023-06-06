/*
 * SCITOS MODULE
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#ifndef SCITOS_MIRA__SCITOS_MODULE_HPP_
#define SCITOS_MIRA__SCITOS_MODULE_HPP_

// C++
#include <string>

// MIRA
#include <fw/Framework.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/**
 * @brief Base class for all Scitos modules (Drive, Charger, etc).
 * 
 */
class ScitosModule{
	public:
		ScitosModule(const std::string & name);
		virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & ros_node) = 0;
		virtual void reset_publishers() = 0;
		virtual ~ScitosModule(){};

	protected:
		bool call_mira_service(std::string service_name);
		bool set_mira_param(std::string param_name, std::string value);
		std::string get_mira_param(std::string param_name);

		mira::Authority authority_;
		rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
		rclcpp::Logger logger_;
};

#endif // SCITOS_MIRA__SCITOS_MODULE_HPP_
