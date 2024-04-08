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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/node_utils.hpp"
#include "scitos2_mira/mira_framework.hpp"
#include "scitos2_core/module.hpp"

class MiraFrameworkFixture : public scitos2_mira::MiraFramework
{
public:
  MiraFrameworkFixture()
  : scitos2_mira::MiraFramework(rclcpp::NodeOptions())
  {}
};

class DummyModule : public scitos2_core::Module
{
public:
  DummyModule() {}

  ~DummyModule() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) {}

  virtual void cleanup() {}

  virtual void activate() {}

  virtual void deactivate() {}
};

// Mocked class loader
void onPluginDeletion(scitos2_core::Module * obj)
{
  if (nullptr != obj) {
    delete (obj);
  }
}

template<>
pluginlib::UniquePtr<scitos2_core::Module> pluginlib::ClassLoader<scitos2_core::Module>::
createUniqueInstance(const std::string & lookup_name)
{
  if (lookup_name != "drive") {
    // original method body
    if (!isClassLoaded(lookup_name)) {
      loadLibraryForClass(lookup_name);
    }
    try {
      std::string class_type = getClassType(lookup_name);
      pluginlib::UniquePtr<scitos2_core::Module> obj =
        lowlevel_class_loader_.createUniqueInstance<scitos2_core::Module>(class_type);
      return obj;
    } catch (const class_loader::CreateClassException & ex) {
      throw pluginlib::CreateClassException(ex.what());
    }
  }

  // mocked plugin creation
  return std::unique_ptr<scitos2_core::Module,
           class_loader::ClassLoader::DeleterType<scitos2_core::Module>>(
    new DummyModule(), onPluginDeletion);
}

TEST(ScitosMiraFrameworkTest, configure) {
  // Create the node
  auto node = std::make_shared<MiraFrameworkFixture>();

  // Set an empty scitos config parameter
  nav2_util::declare_parameter_if_not_declared(node, "scitos_config", rclcpp::ParameterValue(""));

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
  nav2_util::declare_parameter_if_not_declared(
    node, "module_plugins",
    rclcpp::ParameterValue(std::vector<std::string>(1, "drive")));
  nav2_util::declare_parameter_if_not_declared(
    node, "drive.plugin", rclcpp::ParameterValue("drive"));

  // Configure the node
  node->configure();
  node->activate();

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
