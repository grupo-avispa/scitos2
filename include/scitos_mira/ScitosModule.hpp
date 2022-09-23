/*
 * SCITOS MODULE
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
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

// 
//#include <scitos_mira/ScitosMira.hpp>

/**
 * @brief Base class for all Scitos modules (Drive, Charger, etc).
 * 
 */
class ScitosModule : public rclcpp::Node{
	public:
		ScitosModule(const std::string& name);
		virtual ~ScitosModule();

	protected:
		bool call_mira_service(std::string service_name);
		bool set_mira_param(std::string param_name, std::string value);
		std::string get_mira_param(std::string param_name);

		mira::Authority authority_;
	//	std::string name_;
	//	ScitosMira mira_;
};

#endif // SCITOS_MIRA__SCITOS_MODULE_HPP_

