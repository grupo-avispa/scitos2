// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// C++
#include <chrono>
#include <thread>

// ROS
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/node_utils.hpp"

#include "scitos2_mira/mira_framework.hpp"

namespace scitos2_mira
{

MiraFramework::MiraFramework(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("scitos_mira", "", options),
  module_loader_("scitos2_core", "scitos2_core::Module")
{
  RCLCPP_INFO(get_logger(), "Creating MIRA framework");

  // Redirect MIRA logger
  MIRA_LOGGER.registerSink(scitos2_core::SinkLogger(this->get_logger()));

  framework_ = std::make_unique<mira::Framework>(0, nullptr);
}

MiraFramework::~MiraFramework()
{
  modules_.clear();

  if (framework_->isTerminationRequested()) {
    RCLCPP_INFO(get_logger(), "Stopping MIRA framework...");
  }
}

nav2_util::CallbackReturn MiraFramework::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring MIRA framework interface");

  // Load the configuration of the robot
  std::string config;
  nav2_util::declare_parameter_if_not_declared(
    this, "scitos_config", rclcpp::ParameterValue("/opt/SCITOS/SCITOSDriver.xml"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Configuration of the robot in XML format"));
  this->get_parameter("scitos_config", config);
  if (!config.empty()) {
    RCLCPP_INFO(get_logger(), "Loaded scitos config: %s", config.c_str());
    framework_->load(config);
  } else {
    RCLCPP_ERROR(get_logger(), "Can't read parameter 'scitos_config'");
    return nav2_util::CallbackReturn::FAILURE;
  }

  nav2_util::declare_parameter_if_not_declared(
    this, "module_plugins",
    rclcpp::ParameterValue(std::vector<std::string>{}),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("List of the modules exposed by the node"));
  this->get_parameter("module_plugins", module_ids_);
  if (module_ids_.empty()) {
    RCLCPP_ERROR(get_logger(), "No modules specified in the parameter 'module_plugins'");
    return nav2_util::CallbackReturn::FAILURE;
  }
  module_types_.resize(module_ids_.size());

  // Create MIRA modules
  for (size_t i = 0; i != module_ids_.size(); i++) {
    try {
      module_types_[i] = nav2_util::get_plugin_type_param(node, module_ids_[i]);
      scitos2_core::Module::Ptr module =
        module_loader_.createUniqueInstance(module_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created module : %s of type %s",
        module_ids_[i].c_str(), module_types_[i].c_str());
      module->configure(node, module_ids_[i]);
      modules_.insert({module_ids_[i], module});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create module. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != module_ids_.size(); i++) {
    module_ids_concat_ += module_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "MIRA framework has %s modules available.", module_ids_concat_.c_str());

  framework_->start();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MiraFramework::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activate the modules
  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    it->second->activate();
  }

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MiraFramework::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    it->second->deactivate();
  }

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MiraFramework::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    it->second->cleanup();
  }
  modules_.clear();

  framework_->requestTermination();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MiraFramework::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace scitos2_mira

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(scitos2_mira::MiraFramework)
