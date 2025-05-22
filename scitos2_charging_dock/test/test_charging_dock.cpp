// Copyright (c) 2024 Open Navigation LLC
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
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include "scitos2_charging_dock/charging_dock.hpp"

TEST(ScitosChargingDock, objectLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");

  auto dock = std::make_unique<scitos2_charging_dock::ChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Check initial states
  EXPECT_FALSE(dock->isCharging());
  EXPECT_TRUE(dock->disableCharging());
  EXPECT_TRUE(dock->hasStoppedCharging());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(ScitosChargingDock, batteryState)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto pub = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery", rclcpp::QoS(1));
  pub->on_activate();

  auto dock = std::make_unique<scitos2_charging_dock::ChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Not charging
  sensor_msgs::msg::BatteryState msg;
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  pub->publish(msg);
  rclcpp::Rate r(2);
  r.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_FALSE(dock->isCharging());
  EXPECT_TRUE(dock->hasStoppedCharging());

  // Charging
  sensor_msgs::msg::BatteryState msg2;
  msg2.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  pub->publish(msg2);
  rclcpp::Rate r1(2);
  r1.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_TRUE(dock->isCharging());
  EXPECT_FALSE(dock->hasStoppedCharging());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(ScitosChargingDock, stagingPose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto dock = std::make_unique<scitos2_charging_dock::ChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::Pose pose;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose, frame);
  EXPECT_NEAR(staging_pose.pose.position.x, -0.7, 0.01);
  EXPECT_NEAR(staging_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(staging_pose.pose.orientation), 0.0, 0.01);
  EXPECT_EQ(staging_pose.header.frame_id, frame);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(ScitosChargingDock, stagingPoseWithYawOffset)
{
  // Override the parameter default
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    {"my_dock.staging_yaw_offset", 3.14},
  }
  );

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test", options);
  auto dock = std::make_unique<scitos2_charging_dock::ChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::Pose pose;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose, frame);
  // Pose should be the same as default, but pointing in opposite direction
  EXPECT_NEAR(staging_pose.pose.position.x, -0.7, 0.01);
  EXPECT_NEAR(staging_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(staging_pose.pose.orientation), 3.14, 0.01);
  EXPECT_EQ(staging_pose.header.frame_id, frame);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(ScitosChargingDock, refinedPoseTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
  pub->on_activate();
  auto dock = std::make_unique<scitos2_charging_dock::ChargingDock>();

  // Create the TF
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Update parameters to read the test dock template
  std::string pkg = ament_index_cpp::get_package_share_directory("scitos2_charging_dock");
  std::string path = pkg + "/test/dock_test.pcd";
  nav2_util::declare_parameter_if_not_declared(
    node, "my_dock.perception.dock_template", rclcpp::ParameterValue(path));
  nav2_util::declare_parameter_if_not_declared(
    node, "my_dock.segmentation.distance_threshold", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "my_dock.segmentation.min_points", rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node, "my_dock.segmentation.min_width", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "my_dock.segmentation.min_distance", rclcpp::ParameterValue(0.0));

  dock->configure(node, "my_dock", tf_buffer);
  dock->activate();

  geometry_msgs::msg::PoseStamped pose;

  // Timestamps are outdated; this is after timeout
  EXPECT_FALSE(dock->isDocked());
  EXPECT_FALSE(dock->getRefinedPose(pose, ""));

  // Just call the function to set the staging pose in the perception module
  // befor the callback is called
  geometry_msgs::msg::Pose pose_stmp;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose_stmp, frame);

  // Publish a scan
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = node->now();
  scan.header.frame_id = "my_frame";
  // This three points are a match with the test dock template
  scan.angle_min = -std::atan2(0.1, 0.9);
  scan.angle_max = std::atan2(0.1, 0.9);
  scan.angle_increment = std::atan2(0.1, 0.9);
  scan.ranges = {0.9055, 1.0, 0.9055};
  scan.range_min = 0.9055;
  scan.range_max = 1.0;
  pub->publish(std::move(scan));
  rclcpp::spin_some(node->get_node_base_interface());

  // Just expect that the pose is refined
  // The actual values are not important for this test
  // as they depend on the perception module and its already tested
  pose.header.frame_id = "my_frame";
  EXPECT_TRUE(dock->getRefinedPose(pose, ""));
  EXPECT_FALSE(dock->isDocked());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
