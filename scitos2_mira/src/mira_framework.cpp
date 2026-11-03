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
#include <string>
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

  // The parameters that shape the MIRA framework are read here because the framework is
  // created in the constructor: there is only one per process and it has to exist before the
  // node is configured
  int port = 0;
  std::vector<std::string> known_frameworks, variables;
  std::string save_processed_config;
  int log_level = -1;
  nav2::declare_parameter_if_not_declared(
    this, "mira_port", rclcpp::ParameterValue(0),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "Port where the MIRA framework listens, so that miracenter or mirainspect can connect to "
      "the robot while ROS is running. 0 disables it"));
  this->get_parameter("mira_port", port);
  nav2::declare_parameter_if_not_declared(
    this, "mira_known_frameworks", rclcpp::ParameterValue(std::vector<std::string>()),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("MIRA frameworks to connect to, as host:port"));
  this->get_parameter("mira_known_frameworks", known_frameworks);
  nav2::declare_parameter_if_not_declared(
    this, "mira_variables", rclcpp::ParameterValue(std::vector<std::string>()),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Variables of the MIRA configuration, as name=value"));
  this->get_parameter("mira_variables", variables);
  nav2::declare_parameter_if_not_declared(
    this, "save_processed_config", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("File where MIRA saves the configuration once its variables are resolved"));
  this->get_parameter("save_processed_config", save_processed_config);
  nav2::declare_parameter_if_not_declared(
    this, "mira_log_level", rclcpp::ParameterValue(-1),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "Severity of the messages of MIRA (0 critical ... 5 trace). -1 follows the log level of "
      "this node"));
  this->get_parameter("mira_log_level", log_level);

  // Redirect MIRA logger
  MIRA_LOGGER.registerSink(scitos2_mira_utils::SinkLogger(this->get_logger()));
  MIRA_LOGGER.setSeverityLevel(miraSeverity(log_level));

  framework_ = std::make_unique<mira::Framework>(
    buildMiraArguments(port, known_frameworks, variables, save_processed_config));
}

std::vector<std::string> MiraFramework::buildMiraArguments(
  int port, const std::vector<std::string> & known_frameworks,
  const std::vector<std::string> & variables, const std::string & save_processed_config)
{
  // The first argument is the name of the program, like in argv
  std::vector<std::string> args{"scitos2_mira"};
  if (port > 0) {
    args.push_back("-p");
    args.push_back(std::to_string(port));
  }
  for (const auto & known : known_frameworks) {
    args.push_back("-k");
    args.push_back(known);
  }
  for (const auto & variable : variables) {
    args.push_back("-v");
    args.push_back(variable);
  }
  if (!save_processed_config.empty()) {
    args.push_back("--save-processed-config");
    args.push_back(save_processed_config);
  }
  return args;
}

mira::SeverityLevel MiraFramework::miraSeverity(int log_level) const
{
  if (log_level >= mira::SeverityLevel::CRITICAL && log_level <= mira::SeverityLevel::TRACE) {
    return static_cast<mira::SeverityLevel>(log_level);
  }

  // Follow the level of the ROS logger, so that MIRA does not format messages that ROS drops
  switch (rcutils_logging_get_logger_effective_level(get_logger().get_name())) {
    case RCUTILS_LOG_SEVERITY_DEBUG:
      return mira::SeverityLevel::DEBUG;
    case RCUTILS_LOG_SEVERITY_INFO:
      return mira::SeverityLevel::NOTICE;
    case RCUTILS_LOG_SEVERITY_WARN:
      return mira::SeverityLevel::WARNING;
    case RCUTILS_LOG_SEVERITY_ERROR:
      return mira::SeverityLevel::ERROR;
    default:
      return mira::SeverityLevel::CRITICAL;
  }
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
      try {
        framework_->load(config);
        loaded_ = true;
        RCLCPP_INFO(get_logger(), "Loaded scitos config: %s", config.c_str());
      } catch (const mira::Exception & ex) {
        RCLCPP_ERROR(
          get_logger(), "Failed to load scitos config '%s'. Exception: %s",
          config.c_str(), ex.what());
        on_cleanup(state);
        return nav2::CallbackReturn::FAILURE;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Can't read parameter 'scitos_config'");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  } else {
    // The framework outlives a cleanup (it is only terminated on shutdown), so it is reused
    RCLCPP_WARN(get_logger(), "Already loaded scitos config");
  }

  nav2::declare_parameter_if_not_declared(
    this, "mira_robot_resource", rclcpp::ParameterValue("/robot/Robot"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The MIRA resource to wait for before activating the modules"));
  this->get_parameter("mira_robot_resource", robot_resource_);

  nav2::declare_parameter_if_not_declared(
    this, "mira_resource_timeout", rclcpp::ParameterValue(5000),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Milliseconds to wait for the MIRA robot resource when activating"));
  this->get_parameter("mira_resource_timeout", resource_timeout_);

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
      scitos2_core::Module::Ptr module = loadModule(module_types_[i]);
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

  // Start the MIRA framework, only once: it keeps running while the node is deactivated
  if (!started_) {
    try {
      framework_->start();
      started_ = true;
    } catch (const mira::Exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to start MIRA framework. Exception: %s", ex.what());
      on_deactivate(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  // The modules write MIRA properties when they are activated, which only works once the
  // robot unit exists. Not fatal: the modules report the writes that failed
  waitForRobotResource();

  // Activate the modules
  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    try {
      if (!it->second->activate()) {
        RCLCPP_ERROR(get_logger(), "Failed to activate module '%s'", it->first.c_str());
        on_deactivate(state);
        return nav2::CallbackReturn::FAILURE;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to activate module. Exception: %s", ex.what());
      on_deactivate(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  // Create a timer to publish diagnostics
  diag_pub_->on_activate();
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), [this]() {
      diag_pub_->publish(createDiagnostics());
    });

  // Create bond connection
  createBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn MiraFramework::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivate every module regardless of earlier failures: this is best-effort teardown,
  // not a place to leave some modules still holding their MIRA authority
  bool success = true;
  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    if (!it->second->deactivate()) {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate module '%s'", it->first.c_str());
      success = false;
    }
  }

  // Stop publishing diagnostics
  timer_.reset();
  if (diag_pub_) {
    diag_pub_->on_deactivate();
  }

  // Destroy bond connection
  destroyBond();

  return success ? nav2::CallbackReturn::SUCCESS : nav2::CallbackReturn::FAILURE;
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

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn MiraFramework::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  try {
    framework_->requestTermination();
  } catch (const mira::Exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to request termination. Exception: %s", ex.what());
    return nav2::CallbackReturn::FAILURE;
  }

  return nav2::CallbackReturn::SUCCESS;
}

scitos2_core::Module::Ptr MiraFramework::loadModule(const std::string & type)
{
  return module_loader_.createUniqueInstance(type);
}

void MiraFramework::waitForRobotResource()
{
  scitos2_mira_utils::MiraAuthority probe(get_logger());
  probe.setResource(robot_resource_);
  try {
    probe.checkin("scitos_mira_probe");
    probe.start();
    if (!probe.waitForResource(std::chrono::milliseconds(resource_timeout_))) {
      RCLCPP_WARN(
        get_logger(), "MIRA resource '%s' not available after %d ms, activating the modules anyway",
        robot_resource_.c_str(), resource_timeout_);
    }
    probe.checkout();
  } catch (const mira::Exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to wait for the MIRA resource. Exception: %s", ex.what());
  }
}

diagnostic_msgs::msg::DiagnosticArray MiraFramework::createDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "MIRA framework";

  if (!loaded_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "MIRA framework configuration is not loaded";
  } else if (modules_.empty()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "MIRA framework is loaded but has no modules";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "MIRA framework is running with " + std::to_string(modules_.size()) +
      " module(s): " + module_ids_concat_;
  }

  msg.status.push_back(status);

  for (const auto & module : modules_) {
    module.second->diagnostics(msg);
  }
  return msg;
}

}  // namespace scitos2_mira

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(scitos2_mira::MiraFramework)
