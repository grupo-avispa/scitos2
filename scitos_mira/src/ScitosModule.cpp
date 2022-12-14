/*
 * SCITOS MODULE
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

// MIRA
#include <rpc/RPCError.h>

#include "scitos_mira/ScitosModule.hpp"

ScitosModule::ScitosModule(const std::string& name) : Node(name), authority_("/", "scitos_ros", mira::Authority::ANONYMOUS){

}

bool ScitosModule::call_mira_service(std::string service_name){
	mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot", service_name); 
	rpc.timedWait(mira::Duration::seconds(1));
	try{
		rpc.get();
		RCLCPP_DEBUG(this->get_logger(), "service_name: %i", true);
	}catch (mira::XRPC& e){
		RCLCPP_WARN(this->get_logger(), "Mira RPC error caught when calling the service: %s", e.what() );
		return false;
	}
	return true;
}

bool ScitosModule::set_mira_param(std::string param_name, std::string value){
	mira::RPCFuture<void> rpc = authority_.callService<void>("/robot/Robot#builtin", std::string("setProperty"), param_name, value); 
	rpc.timedWait(mira::Duration::seconds(1));
	try{
		rpc.get();
	}catch (mira::XRPC& e){
		RCLCPP_WARN(this->get_logger(), "Mira RPC error caught when setting parameter: %s", e.what() );
		return false;
	}
	return true;
}

std::string ScitosModule::get_mira_param(std::string param_name){
	mira::RPCFuture<std::string> rpc = authority_.callService<std::string>("/robot/Robot#builtin", std::string("getProperty"), param_name);
	rpc.timedWait(mira::Duration::seconds(1));
	return rpc.get();
}