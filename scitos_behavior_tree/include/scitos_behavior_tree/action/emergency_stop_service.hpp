/*
 * EMERGENCY STOP SERVICE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_behavior_tree project.
 * 
 * All rights reserved.
 *
 */

#ifndef SCITOS_BEHAVIOR_TREE__ACTION__EMERGENCY_STOP_SERVICE_HPP_
#define SCITOS_BEHAVIOR_TREE__ACTION__EMERGENCY_STOP_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "scitos_msgs/srv/emergency_stop.hpp"

namespace scitos_behavior_tree{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps scitos_msgs::srv::EmergencyStop
 */
class EmergencyStopService : public nav2_behavior_tree::BtServiceNode<scitos_msgs::srv::EmergencyStop>{
	public:
		/**
		 * @brief A constructor for scitos_behavior_tree::EmergencyStop Service
		 * @param service_node_name Service name this node creates a client for
		 * @param conf BT node configuration
		 */
		EmergencyStopService(
			const std::string & service_node_name,
			const BT::NodeConfiguration & conf);

		/**
		 * @brief Function to perform some user-defined operation on tick
		 */
		void on_tick() override;
};

}  // namespace scitos_behavior_tree

#endif  // SCITOS_BEHAVIOR_TREE__ACTION__EMERGENCY_STOP_SERVICE_HPP_
