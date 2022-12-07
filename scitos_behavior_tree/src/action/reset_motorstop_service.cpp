/*
 * RESET MOTORSTOP SERVICE
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_behavior_tree project.
 * 
 * All rights reserved.
 *
 */
#include <string>
#include <memory>

#include "scitos_behavior_tree/action/reset_motorstop_service.hpp"

namespace scitos_behavior_tree{

ResetMotorStopService::ResetMotorStopService(
	const std::string & service_node_name,
	const BT::NodeConfiguration & conf)
: BtServiceNode<scitos_msgs::srv::ResetMotorStop>(service_node_name, conf)
{
}

void ResetMotorStopService::on_tick(){
	increment_recovery_count();
}

}  // namespace scitos_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
	factory.registerNodeType<scitos_behavior_tree::ResetMotorStopService>("ResetMotorStop");
}
