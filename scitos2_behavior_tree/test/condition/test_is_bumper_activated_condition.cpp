// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <chrono>

#include "scitos2_msgs/msg/bumper_status.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "scitos2_behavior_tree/condition/is_bumper_activated_condition.hpp"

class IsBumperActivatedConditionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("test_is_bumper_activated");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set(
      "node",
      node_);

    factory_->registerNodeType<scitos2_behavior_tree::IsBumperActivatedCondition>(
      "IsBumperActivated");

    bumper_pub_ = node_->create_publisher<scitos2_msgs::msg::BumperStatus>(
      "/bumper_status",
      rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    bumper_pub_.reset();
    node_.reset();
    factory_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static rclcpp::Publisher<scitos2_msgs::msg::BumperStatus>::SharedPtr bumper_pub_;
};

rclcpp::Node::SharedPtr IsBumperActivatedConditionTestFixture::node_ = nullptr;
BT::NodeConfiguration * IsBumperActivatedConditionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsBumperActivatedConditionTestFixture::factory_ = nullptr;
rclcpp::Publisher<scitos2_msgs::msg::BumperStatus>::SharedPtr
IsBumperActivatedConditionTestFixture::bumper_pub_ = nullptr;

TEST_F(IsBumperActivatedConditionTestFixture, test_behavior_power_supply_status)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsBumperActivated bumper_topic="/bumper_status"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  scitos2_msgs::msg::BumperStatus bumper_msg;
  bumper_msg.bumper_activated = false;
  bumper_pub_->publish(bumper_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  bumper_msg.bumper_activated = true;
  bumper_pub_->publish(bumper_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
