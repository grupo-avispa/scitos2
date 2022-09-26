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
#include <fw/Framework.h>

/// ROS
#include "rclcpp/rclcpp.hpp"
#include "scitos_mira/ScitosCharger.hpp"
#include "scitos_mira/ScitosDrive.hpp"
#include "scitos_mira/ScitosMira.hpp"

int main(int argc, char **argv){
	rclcpp::init(argc, argv);

	/*
	mira::Framework framework(argc, argv);
	framework.load("/home/alberto/instaladores-itera-rosi/mira/SCITOS/CLARC/SCITOSDriver.xml");
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
	framework.start();
	*/

	/*ROS_INFO_STREAM("Creating G5 with modules [" << scitos_modules << "]");
	std::vector<std::string> modules;
	boost::split(modules, scitos_modules, boost::is_any_of("\t "));
	ScitosG5 s(modules);
	ROS_INFO("Going into main loop.");
	s.spin();
	return 0;*/

	auto node0 = std::make_shared<ScitosMira>("scitos_mira");
	rclcpp::spin_some(node0);
	std::cout << "sdsds" << std::endl;
	
	/*
	auto node1 = std::make_shared<ScitosDrive>();
	rclcpp::spin_some(node1);
	auto node2 = std::make_shared<ScitosCharger>();
	rclcpp::spin_some(node2);
	std::cout << "asasa" << std::endl;
	rclcpp::shutdown();
	*/
	return 0;
}
