/*
 * MODULE FACTORY
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos2_mira project.
 * 
 * All rights reserved.
 *
 */

#include "scitos2_mira/ModuleFactory.hpp"
#include "scitos2_mira/ScitosModule.hpp"
#include "scitos2_mira/modules/ScitosCharger.hpp"
#include "scitos2_mira/modules/ScitosDisplay.hpp"
#include "scitos2_mira/modules/ScitosDrive.hpp"
#include "scitos2_mira/modules/ScitosEBC.hpp"

// Constructor
ModuleFactory::ModuleFactory(){
	register_module("Charger", ScitosCharger::Create());
	register_module("Display", ScitosDisplay::Create());
	register_module("Drive", ScitosDrive::Create());
	register_module("EBC", ScitosEBC::Create());
}

// Registers the module with the given name
void ModuleFactory::register_module(const std::string& name, std::shared_ptr<ScitosModule> create){
	registered_modules_[name] = create;
}

// Creates a new instance of the module with the given name
std::shared_ptr<ScitosModule> ModuleFactory::create_module(const std::string& name){
	if (registered_modules_.find(name) != registered_modules_.end()){
		return registered_modules_[name];
	}else{
		RCLCPP_ERROR(rclcpp::get_logger("ModuleFactory"), "Trying to create unknown Scitos module.");
		return nullptr;
	}
}
