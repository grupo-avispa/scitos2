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

#include <fw/Framework.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "scitos2_modules/channel_bridge.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

TEST(ScitosChannelBridgeTest, bridgesTheConfiguredChannels) {
  rclcpp::init(0, nullptr);
  mira::Authority authority("/", "test_channel_bridge");
  authority.start();
  auto flag_pub = authority.publish<bool>("/test/Flag");
  auto gain_pub = authority.publish<float>("/test/Gain");
  auto velocity_pub = authority.publish<mira::Velocity2>("/test/Velocity");

  // Three valid channels and two that must be skipped without breaking the others
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testChannelBridge");
  node->declare_parameter(
    "test.channels",
    std::vector<std::string>({"flag", "gain", "velocity", "bad_type", "no_topic"}));
  node->declare_parameter("test.flag.mira_channel", std::string("/test/Flag"));
  node->declare_parameter("test.flag.type", std::string("bool"));
  node->declare_parameter("test.flag.ros_topic", std::string("bridged_flag"));
  node->declare_parameter("test.gain.mira_channel", std::string("/test/Gain"));
  node->declare_parameter("test.gain.type", std::string("float"));
  node->declare_parameter("test.gain.ros_topic", std::string("bridged_gain"));
  node->declare_parameter("test.velocity.mira_channel", std::string("/test/Velocity"));
  node->declare_parameter("test.velocity.type", std::string("velocity2"));
  node->declare_parameter("test.velocity.ros_topic", std::string("bridged_velocity"));
  node->declare_parameter("test.velocity.frame_id", std::string("test_frame"));
  node->declare_parameter("test.bad_type.mira_channel", std::string("/test/Bad"));
  node->declare_parameter("test.bad_type.type", std::string("not_a_type"));
  node->declare_parameter("test.bad_type.ros_topic", std::string("bridged_bad"));
  node->declare_parameter("test.no_topic.mira_channel", std::string("/test/NoTopic"));
  node->declare_parameter("test.no_topic.type", std::string("bool"));

  auto module = std::make_shared<scitos2_modules::ChannelBridge>();
  ASSERT_NO_THROW(module->configure(node, "test"));
  ASSERT_TRUE(module->activate());
  auto module_thread = std::thread([&]() {rclcpp::spin(node->get_node_base_interface());});

  auto sub_node = std::make_shared<rclcpp::Node>("testChannelBridgeSubscriber");
  std::atomic<int> flags{0};
  std::atomic<bool> flag_value{false};
  std::atomic<int> gains{0};
  std::atomic<int> velocities{0};
  std::string velocity_frame;
  std::atomic<double> velocity_x{0.0};
  auto flag_sub = sub_node->create_subscription<std_msgs::msg::Bool>(
    "bridged_flag", 10, [&](const std_msgs::msg::Bool & msg) {
      flag_value = msg.data;
      ++flags;
    });
  auto gain_sub = sub_node->create_subscription<std_msgs::msg::Float32>(
    "bridged_gain", 10, [&](const std_msgs::msg::Float32 &) {++gains;});
  auto velocity_sub = sub_node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "bridged_velocity", 10, [&](const geometry_msgs::msg::TwistStamped & msg) {
      velocity_frame = msg.header.frame_id;
      velocity_x = msg.twist.linear.x;
      ++velocities;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node);});

  // Nothing is published for the channels that were skipped
  EXPECT_EQ(sub_node->get_publishers_info_by_topic("bridged_bad").size(), 0u);

  auto start = std::chrono::steady_clock::now();
  while ((flags == 0 || gains == 0 || velocities == 0) &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
  {
    auto flag_writer = flag_pub.write();
    flag_writer->value() = true;
    flag_writer.finish();
    auto gain_writer = gain_pub.write();
    gain_writer->value() = 1.5f;
    gain_writer.finish();
    auto velocity_writer = velocity_pub.write();
    velocity_writer->value() = mira::Velocity2(0.3, 0.0, 0.1);
    velocity_writer.finish();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  EXPECT_GT(flags, 0);
  EXPECT_TRUE(flag_value);
  EXPECT_GT(gains, 0);
  ASSERT_GT(velocities, 0);
  EXPECT_EQ(velocity_frame, "test_frame");
  EXPECT_FLOAT_EQ(velocity_x, 0.3);

  // Deactivated, the data is not forwarded anymore
  EXPECT_TRUE(module->deactivate());
  module->cleanup();
  rclcpp::shutdown();
  module_thread.join();
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
