/*
 * SCITOS NODE
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

// C++
#include <string>
#include <vector>

// Boost
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

// MIRA
#include <error/LoggingCore.h>
#include <fw/Framework.h>

/// ROS
#include "rclcpp/rclcpp.hpp"
#include "scitos_mira/ScitosDrive.hpp"

/*class RosLogSink : public mira::LogSink {
	void consume(const mira::LogRecord &record) {
		switch (record.level) {
		case mira::SeverityLevel::CRITICAL:
		case mira::SeverityLevel::ERROR:
			ROS_ERROR_STREAM("(MIRA) " << record.message);
			break;
		case mira::SeverityLevel::WARNING:
			ROS_WARN_STREAM("(MIRA) " << record.message);
			break;
		case mira::SeverityLevel::NOTICE:
			ROS_INFO_STREAM("(MIRA) " << record.message);
			break;
		case mira::SeverityLevel::DEBUG:
			ROS_DEBUG_STREAM("(MIRA) " << record.message);
			break;
		default:
			break;
		}
	}
};*/

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	//MIRA_LOGGER.registerSink(RosLogSink());

	//std::string  config_file, port_number, scitos_modules;
	std::vector<std::string> args;

/*
	if (argc < 2) { // no arguments, so use ROS parameters.

		if (ros::param::get("~config_file", config_file))  {
			args.push_back(std::string("-c"));
			args.push_back(config_file);
		} else {
			ROS_ERROR("Can't read parameter 'config_file'");
			return 1;
		}
		if (ros::param::get("~server_port", port_number))  {
			args.push_back(std::string("-p"));
			args.push_back(port_number);
			ROS_INFO_STREAM("Loading with MIRA multiprocess communication support on port " << port_number);
		} else {
			ROS_INFO("Not loading with MIRA multiprocess support.");
		}
	}else {
		for (int i=1; i<argc;i++)
			args.push_back(std::string(argv[i]));
	}*/

	mira::Framework framework(args, true);
	// TODO: Replace by STL
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

	// Find out what SCITOS hardware modules are to be interfaced
	// this could come from reading the MIRA config file but then different config files would be required.
	/*if (!(ros::param::get("~scitos_modules", scitos_modules)) ) {
		ROS_ERROR("Can't read parameter 'scitos_modules'. This MUST be supplied as a space seperated "
		    "list of SCITOS hardware modules to interface into ROS"); 
	  return 1;
	}*/

	/*ROS_INFO_STREAM("Creating G5 with modules [" << scitos_modules << "]");
	std::vector<std::string> modules;
	boost::split(modules, scitos_modules, boost::is_any_of("\t "));
	ScitosG5 s(modules);
	ROS_INFO("Going into main loop.");
	s.spin();
	return 0;*/

	//auto node = std::make_shared<ScitosDrive>();
	//rclcpp::spin_some(node);
	//rclcpp::shutdown();
  	return 0;
}
