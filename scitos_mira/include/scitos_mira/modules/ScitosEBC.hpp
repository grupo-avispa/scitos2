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

		void initialize();

	private:
		ScitosEBC();

		OnSetParametersCallbackHandle::SharedPtr callback_handle_;

		rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

};

#endif // SCITOS_MIRA__SCITOS_EBC_HPP_

