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
#include "nav2_ros_common/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "scitos2_charging_dock/dock_saver.hpp"
#include "scitos2_msgs/srv/save_dock.hpp"

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("scan_publisher")
  {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), [this]() {
        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = now();
        msg.header.frame_id = "my_frame";
        msg.angle_min = -M_PI;
        msg.angle_max = M_PI;
        msg.angle_increment = 2 * M_PI / 5;
        msg.ranges = {1.0, 2.0, 3.0, 4.0, 5.0};
        msg.range_min = msg.ranges.front();
        msg.range_max = msg.ranges.back();
        scan_pub_->publish(msg);
      });
  }

protected:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

TEST(ScitosDockingSaver, saveDockEmpty) {
  rclcpp::init(0, nullptr);
  // Create the node
  auto node = std::make_shared<scitos2_charging_dock::DockSaver>();

  // Declare parameters for segmentation
  nav2::declare_parameter_if_not_declared(
    node, "dock_saver.segmentation.min_points", rclcpp::ParameterValue(10));
  nav2::declare_parameter_if_not_declared(
    node, "dock_saver.segmentation.distance_threshold", rclcpp::ParameterValue(0.01));
  nav2::declare_parameter_if_not_declared(
    node, "dock_saver.segmentation.min_width", rclcpp::ParameterValue(0.0));
  nav2::declare_parameter_if_not_declared(
    node, "dock_saver.segmentation.min_distance", rclcpp::ParameterValue(0.0));

  node->configure();
  node->activate();

  // Create the scan publisher node
  auto pub_node = std::make_shared<TestPublisher>();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node);});

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::SaveDock::Request>();
  auto client = node->create_client<scitos2_msgs::srv::SaveDock>("/dock_saver/save_dock");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);
  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::SaveDock::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Check the response
  EXPECT_FALSE(resp->result);

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
  pub_thread.join();
}

TEST(ScitosDockingSaver, saveDock) {
  rclcpp::init(0, nullptr);
  // Create the node
  auto node = std::make_shared<scitos2_charging_dock::DockSaver>();

  // Declare parameters for segmentation
  nav2::declare_parameter_if_not_declared(
    node, "dock_saver.segmentation.min_points", rclcpp::ParameterValue(0));
  nav2::declare_parameter_if_not_declared(
    node, "dock_saver.segmentation.distance_threshold", rclcpp::ParameterValue(0.01));
  nav2::declare_parameter_if_not_declared(
    node, "dock_saver.segmentation.min_width", rclcpp::ParameterValue(0.0));
  nav2::declare_parameter_if_not_declared(
    node, "dock_saver.segmentation.min_distance", rclcpp::ParameterValue(0.0));

  node->configure();
  node->activate();

  // Create the scan publisher node
  auto pub_node = std::make_shared<TestPublisher>();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node);});

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::SaveDock::Request>();
  auto client = node->create_client<scitos2_msgs::srv::SaveDock>("/dock_saver/save_dock");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_call(req);
  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::SaveDock::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Check the response
  EXPECT_TRUE(resp->result);

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
  pub_thread.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
