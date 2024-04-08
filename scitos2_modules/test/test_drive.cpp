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
#include "nav2_util/node_utils.hpp"
#include "scitos2_modules/drive.hpp"

class DriveFixture : public scitos2_modules::Drive
{
public:
  DriveFixture()
  : scitos2_modules::Drive()
  {}

  void setBaseFrame(const std::string & base_frame)
  {
    base_frame_ = base_frame;
  }

  void setResetBumperInterval(const rclcpp::Duration & reset_bumper_interval)
  {
    reset_bumper_interval_ = reset_bumper_interval;
  }

  rclcpp::Time getLastBumperReset()
  {
    return last_bumper_reset_;
  }

  void setLastBumperReset(const rclcpp::Time & last_bumper_reset)
  {
    last_bumper_reset_ = last_bumper_reset;
  }

  void activateBumper()
  {
    bumper_activated_ = true;
  }

  void deactivateBumper()
  {
    bumper_activated_ = false;
  }

  void activateEmergencyStop()
  {
    emergency_stop_activated_ = true;
  }

  void deactivateEmergencyStop()
  {
    emergency_stop_activated_ = false;
  }

  visualization_msgs::msg::MarkerArray createBumperMarkers()
  {
    return scitos2_modules::Drive::createBumperMarkers();
  }

  bool isEmergencyStopReleased(scitos2_msgs::msg::EmergencyStopStatus msg)
  {
    return scitos2_modules::Drive::isEmergencyStopReleased(msg);
  }

  bool isBarrierCode(uint64_t code)
  {
    return scitos2_modules::Drive::isBarrierCode(code);
  }

  void resetMotorStopAfterTimeout(rclcpp::Time current_time)
  {
    scitos2_modules::Drive::resetMotorStopAfterTimeout(current_time);
  }

  nav_msgs::msg::Odometry miraToRosOdometry(
    const mira::robot::Odometry2 & odometry, const mira::Time & timestamp)
  {
    return scitos2_modules::Drive::miraToRosOdometry(odometry, timestamp);
  }

  geometry_msgs::msg::TransformStamped miraToRosTf(
    const mira::robot::Odometry2 & odometry, const mira::Time & timestamp)
  {
    return scitos2_modules::Drive::miraToRosTf(odometry, timestamp);
  }

  scitos2_msgs::msg::DriveStatus miraToRosDriveStatus(
    const uint32 & status, const mira::Time & timestamp)
  {
    return scitos2_modules::Drive::miraToRosDriveStatus(status, timestamp);
  }

  scitos2_msgs::msg::EmergencyStopStatus miraToRosEmergencyStopStatus(
    const uint32 & status, const mira::Time & timestamp)
  {
    return scitos2_modules::Drive::miraToRosEmergencyStopStatus(status, timestamp);
  }

  scitos2_msgs::msg::BarrierStatus miraToRosBarrierStatus(
    const uint64 & status, const mira::Time & timestamp)
  {
    return scitos2_modules::Drive::miraToRosBarrierStatus(status, timestamp);
  }
};

TEST(ScitosDriveTest, configure) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  // Set the parameter publish_tf to false
  nav2_util::declare_parameter_if_not_declared(
    node, "test.publish_tf", rclcpp::ParameterValue(false));
  module->configure(node, "test");
  module->activate();

  // Cleaning up
  module->deactivate();
  module->cleanup();

  // Now, we set an invalid footprint to show the error
  node->set_parameter(rclcpp::Parameter("test.footprint", "[[bad_string"));

  // Configure the module
  module->configure(node, "test");
  module->activate();

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();

  rclcpp::shutdown();
}

TEST(ScitosDriveTest, dynamicParameters) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set the parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.magnetic_barrier_enabled", true),
      rclcpp::Parameter("test.base_frame", "global_frame")});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.magnetic_barrier_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.base_frame").as_string(), "global_frame");

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, changeForce) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::ChangeForce::Request>();
  req->force = 100;
  auto client = node->create_client<scitos2_msgs::srv::ChangeForce>(
    "drive/change_force");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::ChangeForce::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, emergencyStop) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::EmergencyStop::Request>();
  auto client = node->create_client<scitos2_msgs::srv::EmergencyStop>(
    "drive/emergency_stop");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::EmergencyStop::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, enableMotors) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::EnableMotors::Request>();
  req->enable = true;
  auto client = node->create_client<scitos2_msgs::srv::EnableMotors>(
    "drive/enable_motors");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::EnableMotors::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, enableRfid) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::EnableRfid::Request>();
  req->enable = true;
  auto client = node->create_client<scitos2_msgs::srv::EnableRfid>(
    "drive/enable_rfid");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::EnableRfid::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, resetBarrierStop) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::ResetBarrierStop::Request>();
  auto client = node->create_client<scitos2_msgs::srv::ResetBarrierStop>(
    "drive/reset_barrier_stop");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::ResetBarrierStop::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, resetMotorStop) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::ResetMotorStop::Request>();
  auto client = node->create_client<scitos2_msgs::srv::ResetMotorStop>(
    "drive/reset_motor_stop");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::ResetMotorStop::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, resetOdometry) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::ResetOdometry::Request>();
  auto client = node->create_client<scitos2_msgs::srv::ResetOdometry>(
    "drive/reset_odometry");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::ResetOdometry::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, suspendBumper) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::SuspendBumper::Request>();
  auto client = node->create_client<scitos2_msgs::srv::SuspendBumper>(
    "drive/suspend_bumper");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::SuspendBumper::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, createBumperMarkers) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Set the footprint
  nav2_util::declare_parameter_if_not_declared(
    node, "test.footprint",
    rclcpp::ParameterValue("[[1, 2.2], [.3, -4e4], [-.3, -4e4], [-1, 2.2]]"));

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Deactivate the bumper
  module->deactivateBumper();
  // Create the bumper markers
  auto marker_array = module->createBumperMarkers();
  // Check the number of markers
  EXPECT_EQ(marker_array.markers.front().points.size(), 4);

  // Activate the bumper
  module->activateBumper();
  // Create the bumper markers
  marker_array = module->createBumperMarkers();
  // Check the number of markers
  EXPECT_EQ(marker_array.markers.front().points.size(), 4);
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, isEmergencyStopReleased) {
  // Create the module
  auto module = std::make_shared<DriveFixture>();
  // Create the message: both falses
  scitos2_msgs::msg::EmergencyStopStatus msg;
  msg.emergency_stop_activated = false;
  msg.emergency_stop_status = false;
  // Check the emergency stop: it should be false
  EXPECT_FALSE(module->isEmergencyStopReleased(msg));

  // Create the message: both trues
  msg.emergency_stop_activated = true;
  msg.emergency_stop_status = true;
  // Check the emergency stop: it should be false
  EXPECT_FALSE(module->isEmergencyStopReleased(msg));

  // Create the message: first true, second false
  msg.emergency_stop_activated = true;
  msg.emergency_stop_status = false;
  // Check the emergency stop: it should be true
  EXPECT_TRUE(module->isEmergencyStopReleased(msg));

  // Create the message: first false, second true
  msg.emergency_stop_activated = false;
  msg.emergency_stop_status = true;
  // Check the emergency stop: it should be false
  EXPECT_FALSE(module->isEmergencyStopReleased(msg));
}

TEST(ScitosDriveTest, isBarrierCode) {
  // Create the module
  auto module = std::make_shared<DriveFixture>();
  // Check the barrier code
  EXPECT_TRUE(module->isBarrierCode(scitos2_modules::MAGNETIC_BARRIER_RFID_CODE));
  EXPECT_FALSE(module->isBarrierCode(0));
}

TEST(ScitosDriveTest, resetMotorStopAfterTimeout) {
  // Create the module
  auto module = std::make_shared<DriveFixture>();

  // Activate the bumper and time greater
  module->activateBumper();
  module->setResetBumperInterval(rclcpp::Duration(1, 0));
  module->setLastBumperReset(rclcpp::Time(0, 0));
  // Get the current time
  rclcpp::Time current_time = rclcpp::Time(3, 0);
  // Reset the motor stop
  module->resetMotorStopAfterTimeout(current_time);
  // Check the bumper
  EXPECT_TRUE(module->getLastBumperReset() == current_time);

  // Activate the bumper and time smaller
  module->activateBumper();
  module->setResetBumperInterval(rclcpp::Duration(1, 0));
  module->setLastBumperReset(rclcpp::Time(0, 0));
  // Get the current time
  current_time = rclcpp::Time(0, 50);
  // Reset the motor stop
  module->resetMotorStopAfterTimeout(current_time);
  // Check the bumper
  EXPECT_TRUE(module->getLastBumperReset() == rclcpp::Time(0, 0));

  // Deactivate the bumper and time greater
  module->deactivateBumper();
  module->setResetBumperInterval(rclcpp::Duration(1, 0));
  module->setLastBumperReset(rclcpp::Time(0, 0));
  // Get the current time
  current_time = rclcpp::Time(3, 0);
  // Reset the motor stop
  module->resetMotorStopAfterTimeout(current_time);
  // Check the bumper
  EXPECT_TRUE(module->getLastBumperReset() == rclcpp::Time(0, 0));

  // Deactivate the bumper and time smaller
  module->deactivateBumper();
  module->setResetBumperInterval(rclcpp::Duration(1, 0));
  module->setLastBumperReset(rclcpp::Time(0, 0));
  // Get the current time
  current_time = rclcpp::Time(0, 50);
  // Reset the motor stop
  module->resetMotorStopAfterTimeout(current_time);
  // Check the bumper
  EXPECT_TRUE(module->getLastBumperReset() == rclcpp::Time(0, 0));
}

TEST(ScitosDriveTest, odometryTest) {
  // Create the module
  auto module = std::make_shared<DriveFixture>();

  // Create the odometry
  mira::robot::Odometry2 odometry;
  odometry.pose.x() = 1.0;
  odometry.pose.y() = 2.0;
  odometry.pose.phi() = M_PI_2;

  // Convert the odometry to ROS
  module->setBaseFrame("base_test");
  mira::Time time = mira::Time().now();
  auto ros_odometry = module->miraToRosOdometry(odometry, time);
  // Check the values
  EXPECT_EQ(ros_odometry.header.frame_id, "odom");
  EXPECT_EQ(ros_odometry.header.stamp, rclcpp::Time(time.toUnixNS()));
  EXPECT_EQ(ros_odometry.child_frame_id, "base_test");
  EXPECT_DOUBLE_EQ(ros_odometry.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(ros_odometry.pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(ros_odometry.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(ros_odometry.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(ros_odometry.pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(ros_odometry.pose.pose.orientation.z, 0.70710679664085752);
  EXPECT_DOUBLE_EQ(ros_odometry.pose.pose.orientation.w, 0.70710676573223719);
}

TEST(ScitosDriveTest, transformTest) {
  // Create the module
  auto module = std::make_shared<DriveFixture>();

  // Create the odometry
  mira::robot::Odometry2 odometry;
  odometry.pose.x() = 1.0;
  odometry.pose.y() = 2.0;
  odometry.pose.phi() = M_PI_2;

  // Convert the odometry to ROS
  module->setBaseFrame("base_test");
  mira::Time time = mira::Time().now();
  auto ros_tf = module->miraToRosTf(odometry, time);
  // Check the values
  EXPECT_EQ(ros_tf.header.frame_id, "odom");
  EXPECT_EQ(ros_tf.header.stamp, rclcpp::Time(time.toUnixNS()));
  EXPECT_EQ(ros_tf.child_frame_id, "base_test");
  EXPECT_DOUBLE_EQ(ros_tf.transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(ros_tf.transform.translation.y, 2.0);
  EXPECT_DOUBLE_EQ(ros_tf.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(ros_tf.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(ros_tf.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(ros_tf.transform.rotation.z, 0.70710679664085752);
  EXPECT_DOUBLE_EQ(ros_tf.transform.rotation.w, 0.70710676573223719);
}

TEST(ScitosDriveTest, chargerStatus) {
  // Create the module
  auto module = std::make_shared<DriveFixture>();
  // Create the charger status
  uint32_t status = 0xFFFFFFFF;
  // Convert the charger status to ROS
  module->setBaseFrame("base_test");
  mira::Time time = mira::Time().now();
  auto ros_status = module->miraToRosDriveStatus(status, time);
  // Check the values
  EXPECT_EQ(ros_status.header.frame_id, "base_test");
  EXPECT_EQ(ros_status.header.stamp, rclcpp::Time(time.toUnixNS()));
  EXPECT_EQ(ros_status.mode_normal, true);
  EXPECT_EQ(ros_status.mode_forced_stopped, true);
  EXPECT_EQ(ros_status.mode_freerun, true);
  EXPECT_EQ(ros_status.emergency_stop_activated, true);
  EXPECT_EQ(ros_status.emergency_stop_status, true);
  EXPECT_EQ(ros_status.bumper_front_activated, true);
  EXPECT_EQ(ros_status.bumper_front_status, true);
  EXPECT_EQ(ros_status.bumper_rear_activated, true);
  EXPECT_EQ(ros_status.bumper_rear_status, true);
  EXPECT_EQ(ros_status.safety_field_rear_laser, true);
  EXPECT_EQ(ros_status.safety_field_front_laser, true);
}

TEST(ScitosDriveTest, emergencyStopStatus) {
  // Create the module
  auto module = std::make_shared<DriveFixture>();
  // Create the charger status
  uint32_t status = 0xFFFFFFFF;
  // Convert the charger status to ROS
  module->setBaseFrame("base_test");
  mira::Time time = mira::Time().now();
  auto ros_status = module->miraToRosEmergencyStopStatus(status, time);
  // Check the values
  EXPECT_EQ(ros_status.header.frame_id, "base_test");
  EXPECT_EQ(ros_status.header.stamp, rclcpp::Time(time.toUnixNS()));
  EXPECT_EQ(ros_status.emergency_stop_activated, true);
  EXPECT_EQ(ros_status.emergency_stop_status, true);
}

TEST(ScitosDriveTest, barrierStatus) {
  // Create the module
  auto module = std::make_shared<DriveFixture>();
  // Create the charger status
  uint64_t status = scitos2_modules::MAGNETIC_BARRIER_RFID_CODE;
  // Convert the charger status to ROS
  module->setBaseFrame("base_test");
  mira::Time time = mira::Time().now();
  auto ros_status = module->miraToRosBarrierStatus(status, time);
  // Check the values
  EXPECT_EQ(ros_status.header.frame_id, "base_test");
  EXPECT_EQ(ros_status.header.stamp, rclcpp::Time(time.toUnixNS()));
  EXPECT_TRUE(ros_status.barrier_stopped);
  EXPECT_EQ(ros_status.last_detection_stamp, rclcpp::Time(time.toUnixNS()));
}

TEST(ScitosDriveTest, odometryPublisher) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_drive_odometry");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<mira::robot::Odometry2>("/robot/Odometry");

  // Create and configure the module
  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveSubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the message
  mira::robot::Odometry2 odometry;
  odometry.pose.x() = 1.0;
  odometry.pose.y() = 2.0;
  odometry.pose.phi() = M_PI_2;

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 1,
    [&](const nav_msgs::msg::Odometry & msg) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = odometry;
  writer.finish();

  // Check the received message
  EXPECT_EQ(sub->get_publisher_count(), 1);

  // Cleaning up
  module->deactivate();
  drive_node->deactivate();
  sub_node->deactivate();
  module->cleanup();
  drive_node->cleanup();
  sub_node->cleanup();
  drive_node->shutdown();
  sub_node->shutdown();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  drive_thread.join();
  sub_thread.join();
}

TEST(ScitosDriveTest, bumperPublisher) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_drive_bumper");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<bool>("/robot/Bumper");

  // Create and configure the module
  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveSubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the message
  bool bumper = true;

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<scitos2_msgs::msg::BumperStatus>(
    "bumper", 1,
    [&](const scitos2_msgs::msg::BumperStatus & msg) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = bumper;
  writer.finish();

  // Check the received message
  EXPECT_EQ(sub->get_publisher_count(), 1);

  // Cleaning up
  module->deactivate();
  drive_node->deactivate();
  sub_node->deactivate();
  module->cleanup();
  drive_node->cleanup();
  sub_node->cleanup();
  drive_node->shutdown();
  sub_node->shutdown();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  drive_thread.join();
  sub_thread.join();
}

TEST(ScitosDriveTest, mileagePublisher) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_drive_mileage");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<float>("/robot/Mileage");

  // Create and configure the module
  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveSubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the message
  float mileage = 1.0;

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<scitos2_msgs::msg::Mileage>(
    "mileage", 1,
    [&](const scitos2_msgs::msg::Mileage & msg) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = mileage;
  writer.finish();

  // Check the received message
  EXPECT_EQ(sub->get_publisher_count(), 1);

  // Cleaning up
  module->deactivate();
  drive_node->deactivate();
  sub_node->deactivate();
  module->cleanup();
  drive_node->cleanup();
  sub_node->cleanup();
  drive_node->shutdown();
  sub_node->shutdown();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  drive_thread.join();
  sub_thread.join();
}

TEST(ScitosDriveTest, driveStatusPublisher) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_drive_status");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<uint32>("/robot/DriveStatusPlain");

  // Create and configure the module
  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveSubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the message
  uint32 status = 0xFFFFFFFF;

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<scitos2_msgs::msg::DriveStatus>(
    "drive_status", 1,
    [&](const scitos2_msgs::msg::DriveStatus & msg) {
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
  drive_node->deactivate();
  sub_node->deactivate();
  module->cleanup();
  drive_node->cleanup();
  sub_node->cleanup();
  drive_node->shutdown();
  sub_node->shutdown();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  drive_thread.join();
  sub_thread.join();
}

TEST(ScitosDriveTest, rfidPublisher) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_drive_rfid");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<uint64>("/robot/RFIDUserTag");

  // Create and configure the module
  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveSubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the message
  uint64 rfid = scitos2_modules::MAGNETIC_BARRIER_RFID_CODE;

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<scitos2_msgs::msg::RfidTag>(
    "rfid", 1,
    [&](const scitos2_msgs::msg::RfidTag & msg) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = rfid;
  writer.finish();

  // Check the received message
  EXPECT_EQ(sub->get_publisher_count(), 1);

  // Cleaning up
  module->deactivate();
  drive_node->deactivate();
  sub_node->deactivate();
  module->cleanup();
  drive_node->cleanup();
  sub_node->cleanup();
  drive_node->shutdown();
  sub_node->shutdown();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  drive_thread.join();
  sub_thread.join();
}

TEST(ScitosDriveTest, velocityCommand) {
  rclcpp::init(0, nullptr);
  // Create the velocity publisher node
  auto pub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("velocity_publisher");
  pub_node->configure();
  // Create a publisher for the velocity
  auto vel_pub = pub_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  ASSERT_EQ(vel_pub->get_subscription_count(), 0);
  EXPECT_FALSE(vel_pub->is_activated());
  // Activate the publisher
  pub_node->activate();
  EXPECT_TRUE(vel_pub->is_activated());
  vel_pub->on_activate();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  // Create and configure the module
  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  // Create the message
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 1.0;
  vel.angular.z = 1.0;

  // Publish the message
  module->activateEmergencyStop();
  vel_pub->publish(vel);

  // Now deactivate the emergency stop
  module->deactivateEmergencyStop();
  // Publish the message
  vel_pub->publish(vel);

  // Cleaning up
  module->deactivate();
  drive_node->deactivate();
  pub_node->deactivate();
  module->cleanup();
  drive_node->cleanup();
  pub_node->cleanup();
  drive_node->shutdown();
  pub_node->shutdown();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  pub_thread.join();
  drive_thread.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
