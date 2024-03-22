/*
 * SCITOS EBC
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#ifndef SCITOS_MIRA__SCITOS_EBC_HPP_
#define SCITOS_MIRA__SCITOS_EBC_HPP_

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "scitos2_mira/module.hpp"

/**
 * @brief Module for the EBC power board control.
 * 
 */
class ScitosEBC : public ScitosModule{
	public:
		static std::shared_ptr<ScitosModule> Create() {
			return std::shared_ptr<ScitosModule>(new ScitosEBC());
		}

		void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & ros_node);
		void reset_publishers();

	private:
		ScitosEBC();

		rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

		rcl_interfaces::msg::SetParametersResult parameters_callback(
			const std::vector<rclcpp::Parameter> &parameters);

};

#endif // SCITOS_MIRA__SCITOS_EBC_HPP_
