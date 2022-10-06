/*
 * MODULE FACTORY
 *
 * Copyright (c) 2022 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of scitos_mira project.
 * 
 * All rights reserved.
 *
 */

#ifndef SCITOS_MIRA__MODULE_FACTORY_HPP_
#define SCITOS_MIRA__MODULE_FACTORY_HPP_

// C++
#include <map>
#include <memory>

// SCITOS MIRA
#include "scitos_mira/ScitosModule.hpp"

/**
 * @brief Factory pattern design class for the creation of Scitos modules.
 *        This class is a singleton, so only one instance of it is created.
 *        The class is responsible for the creation of the modules and
 *        for the registration of the modules to the Mira framework.
 */
class ModuleFactory{
	public:
		/**
		 * @brief Destroy the Module Factory object
		 * 
		 */
		~ModuleFactory(){ registered_modules_.clear(); }

		/**
		 * @brief Get the single instance of the ModuleFactory
		 * 
		 * @return ModuleFactory& 
		 */
		static ModuleFactory& get_instance(){
			static ModuleFactory instance;
			return instance;
		}

		/**
		 * @brief Construct a new Module Factory object using the copy constructor
		 * 
		 * @param other Another ModuleFactory object
		 */
		ModuleFactory(const ModuleFactory& other) = delete;

		/**
		 * @brief Construct a new Module Factory object using the assignment operator
		 * 
		 * @param other Another ModuleFactory object
		 * @return ModuleFactory& 
		 */
		ModuleFactory& operator=(const ModuleFactory& other) = delete;

		/**
		 * @brief Registers the module with the given name
		 * 
		 * @param name The name of the module
		 * @param create The function to create the module
		 *
		 */
		void register_module(const std::string& name, std::shared_ptr<ScitosModule> create);

		/**
		 * @brief Create a a new instance of the module with the given name
		 * 
		 * @param name The name of the module
		 * @return std::shared_ptr<ScitosModule> 
		 */
		std::shared_ptr<ScitosModule> create_module(const std::string& name);

		/**
		 * @brief Checks if the module with the given name is registered
		 * 
		 * @param name The name of the module
		 * @return true If the module is registered 
		 * @return false If the module is not registered
		 */
		bool is_registered(const std::string& name){
			return (registered_modules_.find(name) != registered_modules_.end());
		}

	private:
		/**
		 * @brief Construct a new Module Factory object
		 * 
		 */
		ModuleFactory();

		/**
		 * @brief Map of the registered modules
		 * 
		 */
		std::map<std::string, std::shared_ptr<ScitosModule>> registered_modules_;
};

#endif // SCITOS_MIRA__MODULE_FACTORY_HPP_