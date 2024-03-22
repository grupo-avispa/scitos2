/*
 * SCITOS MIRA
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

// Scitos Mira
#include "scitos2_mira/ScitosModule.hpp"
#include "scitos2_mira/RosLogSink.hpp"

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn rclcpp_CallReturn;

/**
 * @brief Main class for the robot. Manages MIRA handle and ROS node.
 * 
 */
class ScitosMira: public rclcpp_lifecycle::LifecycleNode{
	public:
		ScitosMira(const std::string& name, bool intra_process_comms = false);
		~ScitosMira();
		rclcpp_CallReturn on_configure(const rclcpp_lifecycle::State &);
		rclcpp_CallReturn on_activate(const rclcpp_lifecycle::State & state);
		rclcpp_CallReturn on_deactivate(const rclcpp_lifecycle::State & state);
		rclcpp_CallReturn on_cleanup(const rclcpp_lifecycle::State &);
		rclcpp_CallReturn on_shutdown(const rclcpp_lifecycle::State & state);

	private:
		mira::Framework framework_;
		std::vector<std::string> modules_names_;
		std::vector<std::shared_ptr<ScitosModule> > modules_;

		void configure_modules();
		void reset_publishers_modules();
};

#endif // SCITOS_MIRA__SCITOS_MIRA_HPP_
