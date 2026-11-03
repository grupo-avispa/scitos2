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

#include <fw/MicroUnit.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "scitos2_modules/drive.hpp"
#include "scitos2_modules/odometry_covariance.hpp"

class DriveFixture : public scitos2_modules::Drive
{
public:
  DriveFixture()
  : scitos2_modules::Drive()
  {}

  void setBaseFrame(const std::string & base_frame)
  {
    robot_base_frame_ = base_frame;
  }

  void setOdomFrame(const std::string & odom_frame)
  {
    odom_frame_ = odom_frame;
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

  bool isEmergencyStopActivated()
  {
    return emergency_stop_activated_;
  }

  visualization_msgs::msg::MarkerArray createBumperMarkers(std_msgs::msg::Header header)
  {
    return scitos2_modules::Drive::createBumperMarkers(header);
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

  void summarizeDriveStatus(uint32_t status, diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    scitos2_modules::Drive::summarizeDriveStatus(status, stat);
  }

  void fillDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    scitos2_modules::Drive::fillDiagnostics(stat);
  }

  geometry_msgs::msg::TwistStamped miraToRosTargetVelocity(
    const mira::Velocity2 & velocity, const mira::Time & timestamp)
  {
    return scitos2_modules::Drive::miraToRosTargetVelocity(velocity, timestamp);
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
  nav2::declare_parameter_if_not_declared(node, "test.publish_tf", rclcpp::ParameterValue(false));
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
    {rclcpp::Parameter("test.robot_base_frame", "global_frame"),
      rclcpp::Parameter("test.odom_frame", "odometry_frame"),
      rclcpp::Parameter("test.odom_topic", "odom_test_topic"),
      rclcpp::Parameter("test.magnetic_barrier_enabled", true),
      rclcpp::Parameter("test.publish_tf", false),
      rclcpp::Parameter("test.reset_bumper_interval", 20)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.robot_base_frame").as_string(), "global_frame");
  EXPECT_EQ(node->get_parameter("test.odom_frame").as_string(), "odometry_frame");
  EXPECT_EQ(node->get_parameter("test.odom_topic").as_string(), "odom_test_topic");
  EXPECT_EQ(node->get_parameter("test.magnetic_barrier_enabled").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.publish_tf").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.reset_bumper_interval").as_int(), 20);

  // Cleaning up
  module->deactivate();
  module->cleanup();
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

  // Without a real MIRA '/robot/Robot' resource, the call must be reported as failed
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->message, "MIRA service call failed");

  // Cleaning up
  module->deactivate();
  module->cleanup();
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

  // Without a real MIRA '/robot/Robot' resource, the call must be reported as failed:
  // a client must be able to tell a rejected e-stop apart from a successful one
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->message, "MIRA service call failed");

  // Cleaning up
  module->deactivate();
  module->cleanup();
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

  // Without a real MIRA '/robot/Robot' resource, the call must be reported as failed
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->message, "MIRA service call failed");

  // Cleaning up
  module->deactivate();
  module->cleanup();
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

  // Without a real MIRA '/robot/Robot' resource, the call must be reported as failed
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->message, "MIRA service call failed");

  // Cleaning up
  module->deactivate();
  module->cleanup();
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

  // Unlike the other Drive services, this one never calls MIRA: it always succeeds
  EXPECT_TRUE(resp->success);

  // Cleaning up
  module->deactivate();
  module->cleanup();
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

  // Without a real MIRA '/robot/Robot' resource, the call must be reported as failed
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->message, "MIRA service call failed");

  // Cleaning up
  module->deactivate();
  module->cleanup();
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

  // Without a real MIRA '/robot/Robot' resource, the call must be reported as failed
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->message, "MIRA service call failed");

  // Cleaning up
  module->deactivate();
  module->cleanup();
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

  // Without a real MIRA '/robot/Robot' resource, the call must be reported as failed
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->message, "MIRA service call failed");

  // Cleaning up
  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, createBumperMarkers) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");

  // Set the footprint
  nav2::declare_parameter_if_not_declared(
    node, "test.footprint",
    rclcpp::ParameterValue("[[1, 2.2], [.3, -4e4], [-.3, -4e4], [-1, 2.2]]"));

  // Create the module
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  // Deactivate the bumper
  module->deactivateBumper();
  // Create the bumper markers
  std_msgs::msg::Header header;
  header.frame_id = "base_test";
  header.stamp = rclcpp::Time(0, 0);
  auto marker_array = module->createBumperMarkers(header);
  // Check the number of markers
  EXPECT_EQ(marker_array.markers.front().points.size(), 5);
  EXPECT_EQ(marker_array.markers.front().header.frame_id, "base_test");
  EXPECT_EQ(marker_array.markers.front().header.stamp, rclcpp::Time(0, 0));
  EXPECT_EQ(marker_array.markers.front().type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(marker_array.markers.front().action, visualization_msgs::msg::Marker::ADD);
  EXPECT_NEAR(marker_array.markers.front().points[0].x, 1, 1e-3);
  EXPECT_NEAR(marker_array.markers.front().points[0].y, 2.2, 1e-3);
  EXPECT_NEAR(marker_array.markers.front().points[1].x, 0.3, 1e-3);
  EXPECT_NEAR(marker_array.markers.front().points[1].y, -4e4, 1e-3);
  EXPECT_NEAR(marker_array.markers.front().points[2].x, -0.3, 1e-3);
  EXPECT_NEAR(marker_array.markers.front().points[2].y, -4e4, 1e-3);
  EXPECT_NEAR(marker_array.markers.front().points[3].x, -1, 1e-3);
  EXPECT_NEAR(marker_array.markers.front().points[3].y, 2.2, 1e-3);

  // Activate the bumper
  module->activateBumper();
  // Create the bumper markers
  marker_array = module->createBumperMarkers(header);
  // Check the number of markers
  EXPECT_EQ(marker_array.markers.front().points.size(), 5);
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
  module->setOdomFrame("odom_test");
  mira::Time time = mira::Time().now();
  auto ros_odometry = module->miraToRosOdometry(odometry, time);
  // Check the values
  EXPECT_EQ(ros_odometry.header.frame_id, "odom_test");
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
  module->setOdomFrame("odom_test");
  mira::Time time = mira::Time().now();
  auto ros_tf = module->miraToRosTf(odometry, time);
  // Check the values
  EXPECT_EQ(ros_tf.header.frame_id, "odom_test");
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
  EXPECT_EQ(ros_status.mode_safe_drive, true);
  EXPECT_EQ(ros_status.error_sifas_internal, true);
  EXPECT_EQ(ros_status.error_motor_right, true);
  EXPECT_EQ(ros_status.error_firmare, true);
  EXPECT_EQ(ros_status.relay_active, true);
  EXPECT_EQ(ros_status.temperature_warning, true);
  EXPECT_EQ(ros_status.traction_loss, true);
  EXPECT_EQ(ros_status.software_safe_stop, true);
}

TEST(ScitosDriveTest, driveStatusBitLayout) {
  auto module = std::make_shared<DriveFixture>();
  mira::Time time = mira::Time().now();

  // The laser safety fields are bits 25 (rear) and 26 (front) in the struct of MetraLabs
  auto status = module->miraToRosDriveStatus(1u << 25, time);
  EXPECT_TRUE(status.safety_field_rear_laser);
  EXPECT_FALSE(status.safety_field_front_laser);
  status = module->miraToRosDriveStatus(1u << 26, time);
  EXPECT_FALSE(status.safety_field_rear_laser);
  EXPECT_TRUE(status.safety_field_front_laser);

  // Each flag of the second half of the word is decoded on its own
  status = module->miraToRosDriveStatus(1u << 16, time);
  EXPECT_TRUE(status.error_battery_low);
  EXPECT_FALSE(status.error_temperature_high);
  status = module->miraToRosDriveStatus(1u << 31, time);
  EXPECT_TRUE(status.software_safe_stop);
  EXPECT_FALSE(status.traction_loss);
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

TEST(ScitosDriveTest, diagnosticsSummarizeTheDriveStatus) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  auto module = std::make_shared<DriveFixture>();

  // Nothing raised
  diagnostic_updater::DiagnosticStatusWrapper stat;
  module->summarizeDriveStatus(1u, stat);
  EXPECT_EQ(stat.level, DiagnosticStatus::OK);
  EXPECT_EQ(stat.message, "Drive OK");

  // A bumper is a warning and the message names it
  module->summarizeDriveStatus(1u << 10, stat);
  EXPECT_EQ(stat.level, DiagnosticStatus::WARN);
  EXPECT_NE(stat.message.find("bumper_front_status"), std::string::npos);

  // The worst level wins
  module->summarizeDriveStatus((1u << 10) | (1u << 8), stat);
  EXPECT_EQ(stat.level, DiagnosticStatus::ERROR);
  EXPECT_NE(stat.message.find("emergency_stop_status"), std::string::npos);
}

TEST(ScitosDriveTest, diagnosticsReportMissingData) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveDiagnostics");
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");

  diagnostic_updater::DiagnosticStatusWrapper stat;
  module->fillDiagnostics(stat);
  EXPECT_EQ(stat.level, DiagnosticStatus::OK);

  // Active, but MIRA has not published any drive status
  module->activate();
  module->fillDiagnostics(stat);
  EXPECT_EQ(stat.level, DiagnosticStatus::ERROR);

  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, targetVelocity) {
  auto module = std::make_shared<DriveFixture>();
  module->setBaseFrame("base_test");
  mira::Time time = mira::Time().now();

  auto twist = module->miraToRosTargetVelocity(mira::Velocity2(0.5, 0.1, -0.25), time);

  EXPECT_EQ(twist.header.frame_id, "base_test");
  EXPECT_EQ(twist.header.stamp, rclcpp::Time(time.toUnixNS()));
  EXPECT_FLOAT_EQ(twist.twist.linear.x, 0.5);
  EXPECT_FLOAT_EQ(twist.twist.linear.y, 0.1);
  EXPECT_FLOAT_EQ(twist.twist.angular.z, -0.25);
}

TEST(ScitosDriveTest, targetVelocityPublisher) {
  rclcpp::init(0, nullptr);
  mira::Authority authority("/", "test_drive_target_velocity");
  authority.start();
  auto publisher = authority.publish<mira::Velocity2>("/robot/TargetVelocity");

  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveTarget");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveTargetSub");
  sub_node->configure();
  sub_node->activate();
  std::atomic<bool> received_msg{false};
  std::atomic<double> linear_x{0.0};
  auto sub = sub_node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_applied", 1,
    [&](const geometry_msgs::msg::TwistStamped & msg) {
      linear_x = msg.twist.linear.x;
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  auto writer = publisher.write();
  writer->value() = mira::Velocity2(0.4, 0.0, 0.0);
  writer.finish();

  // Wait for the message to be received
  auto start = std::chrono::steady_clock::now();
  while (!received_msg && std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(received_msg);
  EXPECT_FLOAT_EQ(linear_x, 0.4);

  module->deactivate();
  sub_node->deactivate();
  module->cleanup();
  sub_node->cleanup();
  sub_node->shutdown();
  rclcpp::shutdown();
  drive_thread.join();
  sub_thread.join();
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
    [&](const nav_msgs::msg::Odometry & /* msg */) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = odometry;
  writer.finish();

  // Wait for the message to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

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
    [&](const scitos2_msgs::msg::BumperStatus & /* msg */) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = bumper;
  writer.finish();

  // Wait for the message to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

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
    [&](const scitos2_msgs::msg::Mileage & /* msg */) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = mileage;
  writer.finish();

  // Wait for the message to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

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
    [&](const scitos2_msgs::msg::DriveStatus & /* msg */) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = status;
  writer.finish();

  // Wait for the message to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

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
  drive_thread.join();
  sub_thread.join();
}

TEST(ScitosDriveTest, driveStatusUpdatesEmergencyStopFlag) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_drive_status_estop");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<uint32>("/robot/DriveStatusPlain");

  // Create and configure the module
  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDrive");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  // Sanity check: the flag starts cleared
  EXPECT_FALSE(module->isEmergencyStopActivated());

  // Publish a status with bit 7 (emergency_stop_activated) set
  auto writer = publisher.write();
  writer->value() = static_cast<uint32>(1 << 7);
  writer.finish();

  // Wait for the message to be processed
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // driveStatusCallback() must have updated the member from the real status, not left it
  // at whatever it was initialized to in configure()
  EXPECT_TRUE(module->isEmergencyStopActivated());

  // Cleaning up
  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  drive_thread.join();
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
    [&](const scitos2_msgs::msg::RfidTag & /* msg */) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = rfid;
  writer.finish();

  // Wait for the message to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

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
  drive_thread.join();
  sub_thread.join();
}

TEST(ScitosDriveTest, barrierCodeSetsBarrierStopped) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_drive_barrier");
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

  // Create the subscriber
  scitos2_msgs::msg::BarrierStatus received_status;
  bool received_msg = false;
  auto sub = sub_node->create_subscription<scitos2_msgs::msg::BarrierStatus>(
    "barrier_status", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&](const scitos2_msgs::msg::BarrierStatus & msg) {
      received_status = msg;
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the magnetic barrier RFID code
  auto writer = publisher.write();
  writer->value() = scitos2_modules::MAGNETIC_BARRIER_RFID_CODE;
  writer.finish();

  // Wait for the message to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // rfidStatusCallback() must publish the freshly computed status (barrier_stopped = true),
  // not the stale member that only resetBarrierStop() ever touches
  ASSERT_TRUE(received_msg);
  EXPECT_TRUE(received_status.barrier_stopped);

  // Cleaning up
  module->deactivate();
  sub_node->deactivate();
  module->cleanup();
  sub_node->cleanup();
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
  auto vel_pub = pub_node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);
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
  geometry_msgs::msg::TwistStamped vel;
  vel.twist.linear.x = 1.0;
  vel.twist.angular.z = 1.0;

  // Publish the message
  module->activateEmergencyStop();
  vel_pub->publish(vel);

  // Now deactivate the emergency stop
  module->deactivateEmergencyStop();
  // Publish the message
  vel_pub->publish(vel);

  // Cleaning up
  module->deactivate();
  pub_node->deactivate();
  module->cleanup();
  pub_node->cleanup();
  pub_node->shutdown();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs
  pub_thread.join();
  drive_thread.join();
}

TEST(ScitosDriveTest, configureWithExpiredNodeThrows) {
  auto module = std::make_shared<DriveFixture>();
  EXPECT_THROW(
    module->configure(rclcpp_lifecycle::LifecycleNode::WeakPtr(), "test"), std::runtime_error);
}

TEST(ScitosDriveTest, reactivationAfterDeactivationSucceeds) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveTwice");
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");

  // The authority is only checked out on cleanup, so the module can be activated again
  EXPECT_TRUE(module->activate());
  EXPECT_TRUE(module->deactivate());
  EXPECT_TRUE(module->activate());
  EXPECT_TRUE(module->deactivate());
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosDriveTest, emptyFrameAndTopicNamesAreRejected) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveEmptyNames");
  auto module = std::make_shared<DriveFixture>();
  module->configure(node, "test");
  module->activate();

  for (const auto & name : {"robot_base_frame", "odom_frame", "odom_topic"}) {
    auto result = node->set_parameter(rclcpp::Parameter(std::string("test.") + name, ""));
    EXPECT_FALSE(result.successful) << name;
    EXPECT_FALSE(result.reason.empty()) << name;
  }

  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

/**
 * @brief Robot that records the velocities it is told to set.
 */
class FakeVelocityRobot : public mira::MicroUnit
{
public:
  FakeVelocityRobot() {}

  template<typename Reflector>
  void reflect(Reflector & r)
  {
    MIRA_REFLECT_BASE(r, mira::MicroUnit);
    r.method(
      "setVelocity", &FakeVelocityRobot::setVelocity, this, "Set the velocity", "v", "Velocity");
  }

  void setVelocity(mira::Velocity2 velocity)
  {
    last_x_ = velocity.x();
    ++calls_;
  }

  std::atomic<double> last_x_{-1.0};
  std::atomic<int> calls_{0};
};

/**
 * @brief Wait until a condition holds or a timeout expires.
 */
template<typename Condition>
bool waitUntil(Condition condition, std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (!condition() && std::chrono::steady_clock::now() - start < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return condition();
}

TEST(ScitosDriveTest, velocityCommandsReachTheRobotUnlessStaleOrEmergency) {
  rclcpp::init(0, nullptr);
  FakeVelocityRobot robot;
  robot.checkin("/robot", "Robot");
  robot.publishService(robot);
  robot.start();
  mira::Authority authority("/", "test_drive_velocity_commands");
  authority.start();
  auto status_publisher = authority.publish<uint32>("/robot/DriveStatusPlain");

  auto drive_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveCommands");
  auto module = std::make_shared<DriveFixture>();
  module->configure(drive_node, "test");
  module->activate();
  auto drive_thread = std::thread([&]() {rclcpp::spin(drive_node->get_node_base_interface());});

  auto pub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testDriveCommandsPub");
  pub_node->configure();
  pub_node->activate();
  auto vel_pub = pub_node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  vel_pub->on_activate();
  ASSERT_TRUE(waitUntil([&]() {return vel_pub->get_subscription_count() > 0;},
    std::chrono::seconds(5)));

  // A fresh command is sent to the robot
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = drive_node->get_clock()->now();
  command.twist.linear.x = 0.3;
  vel_pub->publish(command);
  ASSERT_TRUE(waitUntil([&]() {return robot.calls_ >= 1;}, std::chrono::seconds(5)));
  EXPECT_NEAR(robot.last_x_, 0.3, 1e-6);

  // One older than cmd_vel_max_age is discarded
  const int calls = robot.calls_;
  command.header.stamp = drive_node->get_clock()->now() - rclcpp::Duration::from_seconds(5.0);
  command.twist.linear.x = 0.5;
  vel_pub->publish(command);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(robot.calls_, calls);

  // One without stamp is never stale
  command.header.stamp = builtin_interfaces::msg::Time();
  vel_pub->publish(command);
  ASSERT_TRUE(waitUntil([&]() {return robot.calls_ > calls;}, std::chrono::seconds(5)));
  EXPECT_NEAR(robot.last_x_, 0.5, 1e-6);

  // The emergency stop sends a zero velocity, once, and blocks the commands afterwards
  auto writer = status_publisher.write();
  writer->value() = 1u << 7;
  writer.finish();
  ASSERT_TRUE(waitUntil([&]() {return robot.last_x_ == 0.0;}, std::chrono::seconds(5)));
  const int calls_after_stop = robot.calls_;
  command.twist.linear.x = 0.4;
  vel_pub->publish(command);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(robot.calls_, calls_after_stop);

  module->deactivate();
  pub_node->deactivate();
  module->cleanup();
  pub_node->cleanup();
  rclcpp::shutdown();
  drive_thread.join();
}

TEST(OdometryCovarianceTest, floorGrowthAndReset) {
  scitos2_modules::OdometryCovariance covariance;
  scitos2_modules::OdometryCovariance::Noise noise;
  noise.trans_trans = 0.1;
  noise.rot_rot = 0.2;
  covariance.setNoise(noise);

  // Without motion only the floor is left, and the unobservable axes are ignored
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  covariance.update(odometry);
  EXPECT_DOUBLE_EQ(odometry.pose.covariance[0], 1e-4);
  EXPECT_DOUBLE_EQ(odometry.pose.covariance[35], 1e-4);
  EXPECT_GE(odometry.pose.covariance[14], 1e5);
  EXPECT_DOUBLE_EQ(odometry.twist.covariance[0], 1e-4);

  // Travelling 2 m adds (0.1 * 2)^2 to x and y, and nothing to the yaw
  odometry.pose.pose.position.x = 2.0;
  odometry.twist.twist.linear.x = 1.0;
  covariance.update(odometry);
  EXPECT_NEAR(odometry.pose.covariance[0], 0.04 + 1e-4, 1e-12);
  EXPECT_NEAR(odometry.pose.covariance[7], 0.04 + 1e-4, 1e-12);
  EXPECT_DOUBLE_EQ(odometry.pose.covariance[35], 1e-4);
  EXPECT_NEAR(odometry.twist.covariance[0], 0.01 + 1e-4, 1e-12);

  // Turning 90 degrees adds (0.2 * pi / 2)^2 to the yaw
  odometry.pose.pose.orientation.z = std::sin(M_PI_4);
  odometry.pose.pose.orientation.w = std::cos(M_PI_4);
  odometry.twist.twist.linear.x = 0.0;
  odometry.twist.twist.angular.z = 1.0;
  covariance.update(odometry);
  EXPECT_NEAR(odometry.pose.covariance[35], std::pow(0.2 * M_PI_2, 2) + 1e-4, 1e-9);
  EXPECT_NEAR(odometry.twist.covariance[35], 0.04 + 1e-4, 1e-12);

  // After a reset the accumulated variance is forgotten
  covariance.reset();
  covariance.update(odometry);
  EXPECT_DOUBLE_EQ(odometry.pose.covariance[0], 1e-4);
  EXPECT_DOUBLE_EQ(odometry.pose.covariance[35], 1e-4);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
