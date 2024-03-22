/*
 * SCITOS NODE
 *
 * Copyright (c) 2022-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

/// ROS
#include "rclcpp/rclcpp.hpp"

// Scitos Mira
#include "scitos2_mira/ScitosMira.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor exe;
	auto node = std::make_shared<ScitosMira>("scitos_mira");
	exe.add_node(node->get_node_base_interface());
	exe.spin();
	rclcpp::shutdown();
	return 0;
}