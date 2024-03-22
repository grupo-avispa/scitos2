/*
 * SCITOS DISPLAY
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#ifndef SCITOS_MIRA__SCITOS_DISPLAY_HPP_
#define SCITOS_MIRA__SCITOS_DISPLAY_HPP_

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// SCITOS MSGS
#include "scitos2_msgs/msg/menu_entry.hpp"

#include "scitos2_mira/ScitosModule.hpp"

/**
 * @brief Module for interfacing the status on the mini embedded display.
 * 
 */
class ScitosDisplay : public ScitosModule{
	public:
		static std::shared_ptr<ScitosModule> Create() {
			return std::shared_ptr<ScitosModule>(new ScitosDisplay());
		}

		void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & ros_node);
		void reset_publishers();

	private:
		rclcpp_lifecycle::LifecyclePublisher<scitos2_msgs::msg::MenuEntry>::SharedPtr display_data_pub_;

		rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
		bool user_menu_enabled_;

		ScitosDisplay();

		rcl_interfaces::msg::SetParametersResult parameters_callback(
			const std::vector<rclcpp::Parameter> &parameters);
		
		void menu_data_callback(mira::ChannelRead<uint8> data);
};

#endif // SCITOS_MIRA__SCITOS_DISPLAY_HPP_
