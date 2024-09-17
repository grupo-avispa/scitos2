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
#include "scitos2_modules/display.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

class DisplayFixture : public scitos2_modules::Display
{
public:
  DisplayFixture()
  : scitos2_modules::Display()
  {}

  void menuDataCallback(mira::ChannelRead<uint8> data)
  {
    menuDataCallback(data);
  }
};

TEST(ScitosDisplayTest, configure) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDisplay");

  // Create the module
  auto module = std::make_shared<DisplayFixture>();
  module->configure(node, "test");
  module->activate();

  // Cleaning up
  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosDisplayTest, dynamicParameters) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDisplay");

  // Create the module
  auto module = std::make_shared<DisplayFixture>();
  module->configure(node, "test");
  module->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set the parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.user_menu_enabled", false),
      rclcpp::Parameter("test.menu_name", "New user Menu"),
      rclcpp::Parameter("test.menu_entry_name_1", "New entry 1"),
      rclcpp::Parameter("test.menu_entry_name_2", "New entry 2"),
      rclcpp::Parameter("test.menu_entry_name_3", "New entry 3")});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.user_menu_enabled").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.menu_name").as_string(), "New user Menu");
  EXPECT_EQ(node->get_parameter("test.menu_entry_name_1").as_string(), "New entry 1");
  EXPECT_EQ(node->get_parameter("test.menu_entry_name_2").as_string(), "New entry 2");
  EXPECT_EQ(node->get_parameter("test.menu_entry_name_3").as_string(), "New entry 3");

  // Now, set the user menu enabled
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.user_menu_enabled", true)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.user_menu_enabled").as_bool(), true);

  // Cleaning up
  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosDisplayTest, menuEntryPublisher) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_display");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<uint8>("/robot/StatusDisplayUserMenuEvent");

  // Create and configure the module
  auto display_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDisplay");
  auto module = std::make_shared<DisplayFixture>();
  module->configure(display_node, "test");
  module->activate();
  auto display_thread = std::thread([&]() {rclcpp::spin(display_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDisplaySubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the message
  uint8_t status = 1;

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<scitos2_msgs::msg::MenuEntry>(
    "user_menu_selected", 1,
    [&](const scitos2_msgs::msg::MenuEntry & msg) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = status;
  writer.finish();

  // Check the received message
  EXPECT_EQ(sub->get_publisher_count(), 1);

  // Cleaning up
  module->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  display_thread.join();
  sub_thread.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
