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

#ifndef SCITOS_MIRA__SCITOS_EBC_HPP_
#define SCITOS_MIRA__SCITOS_EBC_HPP_

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "scitos_mira/ScitosModule.hpp"

/**
 * @brief Module for the EBC power board control.
 * 
 */
class ScitosEBC : public ScitosModule{
	public:
		static std::shared_ptr<ScitosModule> Create() {
			return std::shared_ptr<ScitosModule>(new ScitosEBC());
		}

		rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
		ScitosCallReturn on_configure(const rclcpp_lifecycle::State &);
		ScitosCallReturn on_activate(const rclcpp_lifecycle::State &);
		ScitosCallReturn on_deactivate(const rclcpp_lifecycle::State &);
		ScitosCallReturn on_cleanup(const rclcpp_lifecycle::State &);
		ScitosCallReturn on_shutdown(const rclcpp_lifecycle::State & state);

	private:
		OnSetParametersCallbackHandle::SharedPtr callback_handle_;

		ScitosEBC();
};

#endif // SCITOS_MIRA__SCITOS_EBC_HPP_

