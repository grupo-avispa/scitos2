/*
 * ROS LOG SINK
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#ifndef SCITOS_MIRA__ROS_LOG_SINK_HPP_
#define SCITOS_MIRA__ROS_LOG_SINK_HPP_


// MIRA
#include <error/LoggingCore.h>

// ROS
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Class for MIRA log sinks. Redirect Mira logging to RCLCPP logging.
 * 
 */
class RosLogSink : public mira::LogSink{
	void consume(const mira::LogRecord &record) {
		switch (record.level) {
			case mira::SeverityLevel::CRITICAL:
			case mira::SeverityLevel::ERROR:
				RCLCPP_ERROR_STREAM(rclcpp::get_logger("mira_logger"), record.message);
				break;
			case mira::SeverityLevel::WARNING:
				RCLCPP_WARN_STREAM(rclcpp::get_logger("mira_logger"), record.message);
				break;
			case mira::SeverityLevel::NOTICE:
				RCLCPP_INFO_STREAM(rclcpp::get_logger("mira_logger"), record.message);
				break;
			case mira::SeverityLevel::DEBUG:
				RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mira_logger"), record.message);
				break;
			default:
				break;
			}
		}
};

#endif // SCITOS_MIRA__ROS_LOG_SINK_HPP_

