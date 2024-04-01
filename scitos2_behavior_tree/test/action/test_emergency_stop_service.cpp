// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav2_behavior_tree/utils/test_service.hpp"
#include "scitos2_behavior_tree/action/emergency_stop_service.hpp"

class EmergencyStopService : public TestService<scitos2_msgs::srv::EmergencyStop>
{
public:
  EmergencyStopService()
  : TestService("emergency_stop")
  {}
};

class EmergencyStopServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("emergency_stop_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      std::chrono::milliseconds(1000));

    factory_->registerNodeType<scitos2_behavior_tree::EmergencyStopService>("EmergencyStop");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<EmergencyStopService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr EmergencyStopServiceTestFixture::node_ = nullptr;
std::shared_ptr<EmergencyStopService> EmergencyStopServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration * EmergencyStopServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> EmergencyStopServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> EmergencyStopServiceTestFixture::tree_ = nullptr;

TEST_F(EmergencyStopServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root>
        <BehaviorTree ID="MainTree">
            <EmergencyStop service_name="emergency_stop"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  EmergencyStopServiceTestFixture::server_ = std::make_shared<EmergencyStopService>();
  std::thread server_thread([]() {
      rclcpp::spin(EmergencyStopServiceTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  std::cout << "All tests passed: " << all_successful << std::endl;

  return all_successful;
}
