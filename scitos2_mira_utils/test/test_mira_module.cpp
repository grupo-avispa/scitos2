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

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "scitos2_mira_utils/mira_module.hpp"

class MiraModuleFixture : public scitos2_mira_utils::MiraModule
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) override
  {
    configureMira(parent, name);
  }

  void cleanup() override
  {
    cleanupMira();
  }

  bool activate() override
  {
    return startMira();
  }

  bool deactivate() override
  {
    return true;
  }

  bool hasAuthority() const
  {
    return authority_ != nullptr;
  }

  std::string pluginName() const
  {
    return plugin_name_;
  }
};

TEST(MiraModuleTest, configureWithExpiredNodeThrows) {
  MiraModuleFixture module;
  EXPECT_THROW(
    module.configure(rclcpp_lifecycle::LifecycleNode::WeakPtr(), "test"), std::runtime_error);
}

TEST(MiraModuleTest, lifecycle) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testMiraModule");

  MiraModuleFixture module;
  module.configure(node, "test_mira_module");
  EXPECT_TRUE(module.hasAuthority());
  EXPECT_EQ(module.pluginName(), "test_mira_module");

  // The parameters that every MIRA module shares are declared with their defaults
  EXPECT_EQ(
    node->get_parameter("test_mira_module.mira_robot_resource").as_string(), "/robot/Robot");
  EXPECT_EQ(node->get_parameter("test_mira_module.mira_rpc_timeout").as_int(), 1000);

  // Starting twice is harmless and the authority is released on cleanup
  EXPECT_TRUE(module.activate());
  EXPECT_TRUE(module.activate());
  module.cleanup();
  EXPECT_FALSE(module.hasAuthority());
  EXPECT_NO_THROW(module.cleanup());
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
