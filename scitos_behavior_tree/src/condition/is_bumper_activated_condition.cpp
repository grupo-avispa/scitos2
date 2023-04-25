/*
 * IS BUMPER ACTIVATED CONDITION
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_behavior_tree project.
 * 
 * All rights reserved.
 *
 */

#include <string>

#include "scitos_behavior_tree/condition/is_bumper_activated_condition.hpp"

namespace scitos_behavior_tree{

IsBumperActivatedCondition::IsBumperActivatedCondition(
	const std::string & condition_name,
	const BT::NodeConfiguration & conf)
	: BT::ConditionNode(condition_name, conf),
	bumper_topic_("/bumper"),
	is_bumper_activated_(false)
{
	getInput("bumper_topic", bumper_topic_);
	node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
	callback_group_ = node_->create_callback_group(
	rclcpp::CallbackGroupType::MutuallyExclusive,
	false);
	callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

	rclcpp::SubscriptionOptions sub_option;
	sub_option.callback_group = callback_group_;
	bumper_sub_ = node_->create_subscription<scitos_msgs::msg::BumperStatus>(
	bumper_topic_,
	rclcpp::SystemDefaultsQoS(),
	std::bind(&IsBumperActivatedCondition::bumperCallback, this, std::placeholders::_1),
	sub_option);
}

BT::NodeStatus IsBumperActivatedCondition::tick(){
	callback_group_executor_.spin_some();
	if (is_bumper_activated_) {
		return BT::NodeStatus::SUCCESS;
	}
	return BT::NodeStatus::FAILURE;
}

void IsBumperActivatedCondition::bumperCallback(scitos_msgs::msg::BumperStatus::SharedPtr msg){
	is_bumper_activated_ = msg->bumper_activated;
}

}  // namespace scitos_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
	factory.registerNodeType<scitos_behavior_tree::IsBumperActivatedCondition>("IsBumperActivated");
}