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
#include "scitos2_modules/imu.hpp"

class IMUFixture : public scitos2_modules::IMU
{
public:
  IMUFixture()
  : scitos2_modules::IMU()
  {}

  geometry_msgs::msg::Vector3 miraToRosAcceleration(const mira::Point3f & acceleration)
  {
    return scitos2_modules::IMU::miraToRosAcceleration(acceleration);
  }

  geometry_msgs::msg::Vector3 miraToRosGyroscope(const mira::Point3f & gyroscope)
  {
    return scitos2_modules::IMU::miraToRosGyroscope(gyroscope);
  }
};

TEST(ScitosIMUTest, configure) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testIMU");

  // Create the module
  auto module = std::make_shared<IMUFixture>();
  module->configure(node, "test");
  module->activate();

  // Cleaning up
  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosIMUTest, dynamicParameters) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testIMU");

  // Create the module
  auto module = std::make_shared<IMUFixture>();
  module->configure(node, "test");
  module->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set the parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.robot_base_frame", "global_frame")});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.robot_base_frame").as_string(), "global_frame");

  // Cleaning up
  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosIMUTest, acceSubPub) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_imu_status");
  authority.start();
  // Create the MIRA publisher
  auto acc_pub = authority.publish<mira::Point3f>("/robot/Acceleration");
  auto gyro_pub = authority.publish<mira::Point3f>("/robot/Gyroscope");

  // Create and configure the module
  auto imu_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testIMU");
  auto module = std::make_shared<IMUFixture>();
  module->configure(imu_node, "test");
  module->activate();
  auto imu_thread = std::thread([&]() {rclcpp::spin(imu_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testIMUSubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 1,
    [&](const sensor_msgs::msg::Imu & /*msg*/) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the messages
  mira::Point3f acceleration;
  acceleration.x() = 1.0;
  acceleration.y() = 2.0;
  acceleration.z() = 3.0;
  auto acc_writer = acc_pub.write();
  acc_writer->value() = acceleration;
  acc_writer.finish();

  mira::Point3f gyroscope;
  gyroscope.x() = 1.0;
  gyroscope.y() = 2.0;
  gyroscope.z() = 3.0;
  auto gyro_writer = gyro_pub.write();
  gyro_writer->value() = gyroscope;
  gyro_writer.finish();

  // Wait for the message to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Check the received message
  EXPECT_EQ(sub->get_publisher_count(), 1);
  EXPECT_TRUE(received_msg);

  // Cleaning up
  module->deactivate();
  sub_node->deactivate();
  module->cleanup();
  sub_node->cleanup();
  sub_node->shutdown();
  rclcpp::shutdown();

  // Have to join thread after rclcpp is shut down otherwise test hangs
  imu_thread.join();
  sub_thread.join();
}

TEST(ScitosIMUTest, accelerationTest) {
  // Create the module
  auto module = std::make_shared<IMUFixture>();

  // Create the acceleration
  mira::Point3f acceleration;
  acceleration.x() = 1.0;
  acceleration.y() = 2.0;
  acceleration.z() = 0.0;

  // Convert the acceleration to ROS
  mira::Time time = mira::Time().now();
  auto ros_acceleration = module->miraToRosAcceleration(acceleration);

  // Check the values
  EXPECT_DOUBLE_EQ(ros_acceleration.x, 9.80665);
  EXPECT_DOUBLE_EQ(ros_acceleration.y, 2.0 * 9.80665);
  EXPECT_DOUBLE_EQ(ros_acceleration.z, 0.0);
}

TEST(ScitosIMUTest, gyroscopeTest) {
  // Create the module
  auto module = std::make_shared<IMUFixture>();

  // Create the gyroscope
  mira::Point3f gyroscope;
  gyroscope.x() = 1.0;
  gyroscope.y() = 2.0;
  gyroscope.z() = 0.0;

  // Convert the gyroscope to ROS
  mira::Time time = mira::Time().now();
  auto ros_gyroscope = module->miraToRosGyroscope(gyroscope);

  // Check the values
  EXPECT_DOUBLE_EQ(ros_gyroscope.x, M_PI / 180.0);
  EXPECT_DOUBLE_EQ(ros_gyroscope.y, 2.0 * M_PI / 180.0);
  EXPECT_DOUBLE_EQ(ros_gyroscope.z, 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
