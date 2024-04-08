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

class MiraFrameworkFixture : public scitos2_mira::MiraFramework
{
public:
  MiraFrameworkFixture()
  : scitos2_mira::MiraFramework(rclcpp::NodeOptions())
  {}
};

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

  // Cleaning up
  node->deactivate();
  node->cleanup();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
