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

#include <string>
#include <memory>

#include "scitos_behavior_tree/action/emergency_stop_service.hpp"

namespace scitos_behavior_tree{

EmergencyStopService::EmergencyStopService(
	const std::string & service_node_name,
	const BT::NodeConfiguration & conf)
: BtServiceNode<scitos_msgs::srv::EmergencyStop>(service_node_name, conf)
{
}

void EmergencyStopService::on_tick(){
}

}  // namespace scitos_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
	factory.registerNodeType<scitos_behavior_tree::EmergencyStopService>("EmergencyStop");
}
