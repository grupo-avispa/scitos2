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
#include "scitos2_modules/ebc.hpp"

class EBCFixture : public scitos2_modules::EBC
{
public:
  EBCFixture()
  : scitos2_modules::EBC()
  {}
};

TEST(ScitosEBCTest, configure) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testEBC");

  // Create the module
  auto module = std::make_shared<EBCFixture>();
  module->configure(node, "test");
  module->activate();

  // Cleaning up
  module->deactivate();
  module->cleanup();
}

TEST(ScitosEBCTest, dynamicParameters) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testEBC");

  // Create the module
  auto module = std::make_shared<EBCFixture>();
  module->configure(node, "test");
  module->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set the parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.mcu_5v_enabled", false),
      rclcpp::Parameter("test.mcu_12v_enabled", false),
      rclcpp::Parameter("test.mcu_24v_enabled", false),
      rclcpp::Parameter("test.port0_5v_enabled", false),
      rclcpp::Parameter("test.port0_12v_enabled", false),
      rclcpp::Parameter("test.port0_24v_enabled", false),
      rclcpp::Parameter("test.port1_5v_enabled", false),
      rclcpp::Parameter("test.port1_12v_enabled", false),
      rclcpp::Parameter("test.port1_24v_enabled", false),
      rclcpp::Parameter("test.mcu_5v_max_current", 1.0),
      rclcpp::Parameter("test.mcu_12v_max_current", 1.0),
      rclcpp::Parameter("test.mcu_24v_max_current", 1.0),
      rclcpp::Parameter("test.port0_5v_max_current", 1.0),
      rclcpp::Parameter("test.port0_12v_max_current", 1.0),
      rclcpp::Parameter("test.port0_24v_max_current", 1.0),
      rclcpp::Parameter("test.port1_5v_max_current", 1.0),
      rclcpp::Parameter("test.port1_12v_max_current", 1.0),
      rclcpp::Parameter("test.port1_24v_max_current", 1.0)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.mcu_5v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.mcu_12v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.mcu_24v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.port0_5v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.port0_12v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.port0_24v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.port1_5v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.port1_12v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.port1_24v_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.mcu_5v_max_current").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.mcu_12v_max_current").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.mcu_24v_max_current").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.port0_5v_max_current").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.port0_12v_max_current").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.port0_24v_max_current").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.port1_5v_max_current").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.port1_12v_max_current").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.port1_24v_max_current").as_double(), 1.0);

  // Cleaning up
  module->deactivate();
  module->cleanup();
}


TEST(ScitosEBCTest, dynamicParametersOutRange) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testEBC");

  // Create the module
  auto module = std::make_shared<EBCFixture>();
  module->configure(node, "test");
  module->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set the parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.mcu_5v_enabled", true),
      rclcpp::Parameter("test.mcu_12v_enabled", true),
      rclcpp::Parameter("test.mcu_24v_enabled", true),
      rclcpp::Parameter("test.port0_5v_enabled", true),
      rclcpp::Parameter("test.port0_12v_enabled", true),
      rclcpp::Parameter("test.port0_24v_enabled", true),
      rclcpp::Parameter("test.port1_5v_enabled", true),
      rclcpp::Parameter("test.port1_12v_enabled", true),
      rclcpp::Parameter("test.port1_24v_enabled", true),
      rclcpp::Parameter("test.mcu_5v_max_current", 5.0),
      rclcpp::Parameter("test.mcu_12v_max_current", 5.0),
      rclcpp::Parameter("test.mcu_24v_max_current", 5.0),
      rclcpp::Parameter("test.port0_5v_max_current", 5.0),
      rclcpp::Parameter("test.port0_12v_max_current", 5.0),
      rclcpp::Parameter("test.port0_24v_max_current", 5.0),
      rclcpp::Parameter("test.port1_5v_max_current", 5.0),
      rclcpp::Parameter("test.port1_12v_max_current", 5.0),
      rclcpp::Parameter("test.port1_24v_max_current", 5.0)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters: it should warn about the max currents and set to max
  EXPECT_EQ(node->get_parameter("test.mcu_5v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.mcu_12v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.mcu_24v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.port0_5v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.port0_12v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.port0_24v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.port1_5v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.port1_12v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.port1_24v_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.mcu_5v_max_current").as_double(), 2.5);
  EXPECT_EQ(node->get_parameter("test.mcu_12v_max_current").as_double(), 2.5);
  EXPECT_EQ(node->get_parameter("test.mcu_24v_max_current").as_double(), 2.5);
  EXPECT_EQ(node->get_parameter("test.port0_5v_max_current").as_double(), 2.5);
  EXPECT_EQ(node->get_parameter("test.port0_12v_max_current").as_double(), 2.5);
  EXPECT_EQ(node->get_parameter("test.port0_24v_max_current").as_double(), 2.5);
  EXPECT_EQ(node->get_parameter("test.port1_5v_max_current").as_double(), 2.5);
  EXPECT_EQ(node->get_parameter("test.port1_12v_max_current").as_double(), 2.5);
  EXPECT_EQ(node->get_parameter("test.port1_24v_max_current").as_double(), 4.0);

  // Cleaning up
  module->deactivate();
  module->cleanup();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  mira::Framework framework(argc, argv);
  framework.start();
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
