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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "scitos2_mira/mira_framework.hpp"
#include "scitos2_core/module.hpp"

class MiraFrameworkFixture : public scitos2_mira::MiraFramework
{
public:
  MiraFrameworkFixture()
  : scitos2_mira::MiraFramework(rclcpp::NodeOptions())
  {}

  diagnostic_msgs::msg::DiagnosticArray createDiagnostics()
  {
    return scitos2_mira::MiraFramework::createDiagnostics();
  }

  scitos2_core::Module::Ptr loadModule(const std::string & type) override;
};

class DummyModule : public scitos2_core::Module
{
public:
  DummyModule() {}

  ~DummyModule() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &/*parent*/, std::string/*name*/) {}

  virtual void cleanup() {}

  virtual bool activate() {return true;}

  virtual bool deactivate() {return true;}
};

scitos2_core::Module::Ptr MiraFrameworkFixture::loadModule(const std::string & type)
{
  if (type == "drive") {
    return scitos2_core::Module::Ptr(new DummyModule());
  }
  return scitos2_mira::MiraFramework::loadModule(type);
}

TEST(ScitosMiraFrameworkTest, configure) {
  // Create the node
  auto node = std::make_shared<MiraFrameworkFixture>();

  // Set an empty scitos config parameter
  nav2::declare_parameter_if_not_declared(node, "scitos_config", rclcpp::ParameterValue(""));

  // Configure the node
  node->configure();
  node->activate();

  // Check results: the node should be in the unconfigured state as scitos_config plugins is empty
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Now, set the scitos config parameter. In the the robot this should be a XML file
  std::string pkg = ament_index_cpp::get_package_share_directory("scitos2_mira");
  node->set_parameter(
    rclcpp::Parameter(
      "scitos_config", rclcpp::ParameterValue(std::string(pkg + "/test/scitos_config.xml"))));
  nav2::declare_parameter_if_not_declared(
    node, "module_plugins",
    rclcpp::ParameterValue(std::vector<std::string>(1, "drive")));
  nav2::declare_parameter_if_not_declared(
    node, "drive.plugin", rclcpp::ParameterValue("drive"));

  // Configure the node
  node->configure();
  node->activate();

  // Just call the function
  node->createDiagnostics();

  // Check results: the node should be in the active state
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Cleaning up
  node->deactivate();
  node->cleanup();

  // Configure the node again to warn that the scitos config is already loaded
  node->configure();
  node->activate();

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
