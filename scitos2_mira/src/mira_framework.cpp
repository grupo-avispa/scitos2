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
#include "nav2_ros_common/node_utils.hpp"

#include "scitos2_mira/mira_framework.hpp"

namespace scitos2_mira
{

MiraFramework::MiraFramework(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("scitos_mira", "", options),
  loaded_(false),
  module_loader_("scitos2_core", "scitos2_core::Module"),
  default_ids_{"drive"},
  default_types_{"scitos2_modules::Drive"}
{
  RCLCPP_INFO(get_logger(), "Creating MIRA framework");

  // Redirect MIRA logger
  MIRA_LOGGER.registerSink(scitos2_core::SinkLogger(this->get_logger()));
  MIRA_LOGGER.setSeverityLevel(mira::SeverityLevel::DEBUG);

  framework_ = std::make_unique<mira::Framework>(0, nullptr);
}

MiraFramework::~MiraFramework()
{
  modules_.clear();

  if (framework_->isTerminationRequested()) {
    RCLCPP_INFO(get_logger(), "Stopping MIRA framework...");
  }
  framework_.reset();
  timer_.reset();
}

nav2::CallbackReturn MiraFramework::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring MIRA framework interface");

  // Load the configuration of the robot
  std::string config;
  nav2::declare_parameter_if_not_declared(
    this, "scitos_config", rclcpp::ParameterValue("/opt/SCITOS/SCITOSDriver.xml"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Configuration of the robot in XML format"));
  this->get_parameter("scitos_config", config);
  if (!loaded_) {
    if (!config.empty()) {
      RCLCPP_INFO(get_logger(), "Loaded scitos config: %s", config.c_str());
      framework_->load(config);
      loaded_ = true;
    } else {
      RCLCPP_ERROR(get_logger(), "Can't read parameter 'scitos_config'");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  } else {
    RCLCPP_WARN(get_logger(), "Already loaded scitos config");
  }

  nav2::declare_parameter_if_not_declared(
    this, "module_plugins",
    rclcpp::ParameterValue(default_ids_),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("List of the modules exposed by the node"));
  this->get_parameter("module_plugins", module_ids_);

  // Print the list of modules
  for (size_t i = 0; i != module_ids_.size(); ++i) {
    RCLCPP_INFO(get_logger(), "Module: %s", module_ids_[i].c_str());
  }

  for (size_t i = 0; i != default_ids_.size(); ++i) {
    RCLCPP_INFO(get_logger(), "Default module: %s", default_ids_[i].c_str());
  }

  if (module_ids_ == default_ids_) {
    for (size_t i = 0; i != default_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        this, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]),
        rcl_interfaces::msg::ParameterDescriptor()
        .set__description("Type of the module"));
    }
  }

  module_types_.resize(module_ids_.size());

  // Create MIRA modules
  for (size_t i = 0; i != module_ids_.size(); i++) {
    try {
      module_types_[i] = nav2::get_plugin_type_param(node, module_ids_[i]);
      scitos2_core::Module::Ptr module =
        module_loader_.createUniqueInstance(module_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created module : %s of type %s",
        module_ids_[i].c_str(), module_types_[i].c_str());
      module->configure(node, module_ids_[i]);
      modules_.insert({module_ids_[i], module});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create module. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != module_ids_.size(); i++) {
    module_ids_concat_ += module_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(), "MIRA framework has %s modules available.", module_ids_concat_.c_str());

  // Create a publisher for diagnostics
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn MiraFramework::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activate the modules
  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    try {
      it->second->activate();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to activate module. Exception: %s", ex.what());
      on_deactivate(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  // Start the MIRA framework
  try {
    framework_->start();
  } catch (const mira::Exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to start MIRA framework. Exception: %s", ex.what());
    on_deactivate(state);
    return nav2::CallbackReturn::FAILURE;
  }

  // Create a timer to publish diagnostics
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), [this]() {
      diag_pub_->publish(createDiagnostics());
    });

  // Create bond connection
  createBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn MiraFramework::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    it->second->deactivate();
  }

  // Destroy bond connection
  destroyBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn MiraFramework::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    it->second->cleanup();
  }
  modules_.clear();

  diag_pub_.reset();
  timer_.reset();

  try {
    framework_->requestTermination();
  } catch (const mira::Exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to request termination. Exception: %s", ex.what());
    return nav2::CallbackReturn::FAILURE;
  }

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn MiraFramework::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2::CallbackReturn::SUCCESS;
}

diagnostic_msgs::msg::DiagnosticArray MiraFramework::createDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "MIRA framework";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "MIRA framework is running";
  msg.status.push_back(status);
  return msg;
}

}  // namespace scitos2_mira

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(scitos2_mira::MiraFramework)
