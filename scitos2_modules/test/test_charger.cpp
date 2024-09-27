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
#include "scitos2_modules/charger.hpp"

class ChargerFixture : public scitos2_modules::Charger
{
public:
  ChargerFixture()
  : scitos2_modules::Charger()
  {}

  sensor_msgs::msg::BatteryState miraToRosBatteryState(
    const mira::robot::BatteryState & battery,
    const mira::Time & timestamp)
  {
    return scitos2_modules::Charger::miraToRosBatteryState(battery, timestamp);
  }

  scitos2_msgs::msg::ChargerStatus miraToRosChargerStatus(
    const uint8_t & status,
    const mira::Time & timestamp)
  {
    return scitos2_modules::Charger::miraToRosChargerStatus(status, timestamp);
  }
};

TEST(ScitosChargerTest, configure) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testCharger");

  // Create the module
  auto module = std::make_shared<ChargerFixture>();
  module->configure(node, "test");
  module->activate();

  // Cleaning up
  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosChargerTest, batteryPublisher) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_battery");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<mira::robot::BatteryState>("/robot/charger/Battery");

  // Create and configure the module
  auto charger_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testCharger");
  auto module = std::make_shared<ChargerFixture>();
  module->configure(charger_node, "test");
  module->activate();
  auto charger_thread = std::thread([&]() {rclcpp::spin(charger_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testChargerSubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the message
  mira::robot::BatteryState state;
  state.voltage = 5.0;

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery", 1,
    [&](const sensor_msgs::msg::BatteryState & /*msg*/) {
      RCLCPP_INFO(sub_node->get_logger(), "Received message");
      received_msg = true;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Publish the message
  auto writer = publisher.write();
  writer->value() = state;
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
  charger_thread.join();
  sub_thread.join();
}

TEST(ScitosChargerTest, statusPublisher) {
  rclcpp::init(0, nullptr);
  // Create the MIRA authority
  mira::Authority authority("/", "test_charger_status");
  authority.start();
  // Create the MIRA publisher
  auto publisher = authority.publish<uint8>("/robot/charger/ChargerStatus");

  // Create and configure the module
  auto charger_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testCharger");
  auto module = std::make_shared<ChargerFixture>();
  module->configure(charger_node, "test");
  module->activate();
  auto charger_thread = std::thread([&]() {rclcpp::spin(charger_node->get_node_base_interface());});

  // Create the susbcriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testChargerSubscriber");
  sub_node->configure();
  sub_node->activate();

  // Create the message
  uint8_t status = 0xFF;

  // Create the subscriber
  bool received_msg = false;
  auto sub = sub_node->create_subscription<scitos2_msgs::msg::ChargerStatus>(
    "charger_status", 1,
    [&](const scitos2_msgs::msg::ChargerStatus & /*msg*/) {
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
  charger_thread.join();
  sub_thread.join();
}

TEST(ScitosChargerTest, savePersistentError) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testCharger");

  // Create the module
  auto module = std::make_shared<ChargerFixture>();
  module->configure(node, "test");
  module->activate();

  // Create the client service
  auto req = std::make_shared<scitos2_msgs::srv::SavePersistentErrors::Request>();
  auto client = node->create_client<scitos2_msgs::srv::SavePersistentErrors>(
    "charger/save_persistent_errors");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<scitos2_msgs::srv::SavePersistentErrors::Response>();
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service call successful");
    resp = result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Cleaning up
  module->deactivate();
  module->cleanup();
  rclcpp::shutdown();
}

TEST(ScitosChargerTest, batteryStateNormal) {
  // Create the module
  auto module = std::make_shared<ChargerFixture>();

  // Create the MIRA BatteryState
  mira::robot::BatteryState battery_state;
  battery_state.voltage = 5.0;
  battery_state.current = 1.0;
  battery_state.lifeTime = 1;
  battery_state.lifePercent = 50;
  battery_state.charging = true;
  battery_state.powerSupplyPresent = true;
  battery_state.cellVoltage.push_back(1.0);
  battery_state.cellVoltage.push_back(2.0);

  // Convert to ROS BatteryState
  mira::Time time = mira::Time().now();
  auto battery = module->miraToRosBatteryState(battery_state, time);

  // Check the conversion
  EXPECT_EQ(battery.header.frame_id, "base_link");
  EXPECT_EQ(battery.header.stamp, rclcpp::Time(time.toUnixNS()));
  EXPECT_DOUBLE_EQ(battery.voltage, battery_state.voltage);
  EXPECT_DOUBLE_EQ(battery.current, -battery_state.current);
  EXPECT_DOUBLE_EQ(battery.charge, 0.01666666753590107);
  EXPECT_DOUBLE_EQ(battery.percentage, 0.5);
  EXPECT_EQ(
    battery.power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
  EXPECT_EQ(battery.cell_voltage.size(), battery_state.cellVoltage.size());
  for (size_t i = 0; i < battery.cell_voltage.size(); ++i) {
    EXPECT_EQ(battery.cell_voltage[i], battery_state.cellVoltage[i]);
  }
}

TEST(ScitosChargerTest, batteryStateFull) {
  // Create the module
  auto module = std::make_shared<ChargerFixture>();

  // Create the MIRA BatteryState
  mira::robot::BatteryState battery_state;
  battery_state.voltage = 5.0;
  battery_state.current = 1.0;
  battery_state.lifeTime = -1;
  battery_state.lifePercent = 100;
  battery_state.charging = false;
  battery_state.powerSupplyPresent = false;

  // Convert to ROS BatteryState
  auto battery = module->miraToRosBatteryState(battery_state, mira::Time());

  // Check the conversion
  EXPECT_DOUBLE_EQ(battery.voltage, battery_state.voltage);
  EXPECT_DOUBLE_EQ(battery.current, -battery_state.current);
  EXPECT_TRUE(isnan(battery.charge));
  EXPECT_DOUBLE_EQ(battery.percentage, 1.0);
  EXPECT_EQ(
    battery.power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL);
}

TEST(ScitosChargerTest, batteryStateNotCharging) {
  // Create the module
  auto module = std::make_shared<ChargerFixture>();

  // Create the MIRA BatteryState
  mira::robot::BatteryState battery_state;
  battery_state.voltage = 5.0;
  battery_state.current = 1.0;
  battery_state.lifeTime = -1;
  battery_state.lifePercent = 255;
  battery_state.charging = false;
  battery_state.powerSupplyPresent = true;

  // Convert to ROS BatteryState
  auto battery = module->miraToRosBatteryState(battery_state, mira::Time());

  // Check the conversion
  EXPECT_DOUBLE_EQ(battery.voltage, battery_state.voltage);
  EXPECT_DOUBLE_EQ(battery.current, -battery_state.current);
  EXPECT_TRUE(isnan(battery.charge));
  EXPECT_TRUE(isnan(battery.percentage));
  EXPECT_EQ(
    battery.power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING);
}

TEST(ScitosChargerTest, batteryStateDischarging) {
  // Create the module
  auto module = std::make_shared<ChargerFixture>();

  // Create the MIRA BatteryState
  mira::robot::BatteryState battery_state;
  battery_state.voltage = 5.0;
  battery_state.current = 1.0;
  battery_state.lifeTime = -1;
  battery_state.lifePercent = 255;
  battery_state.charging = false;
  battery_state.powerSupplyPresent = false;

  // Convert to ROS BatteryState
  auto battery = module->miraToRosBatteryState(battery_state, mira::Time());

  // Check the conversion
  EXPECT_DOUBLE_EQ(battery.voltage, battery_state.voltage);
  EXPECT_DOUBLE_EQ(battery.current, -battery_state.current);
  EXPECT_TRUE(isnan(battery.charge));
  EXPECT_TRUE(isnan(battery.percentage));
  EXPECT_EQ(
    battery.power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
}

TEST(ScitosChargerTest, batteryStateUnknown) {
  // Create the module
  auto module = std::make_shared<ChargerFixture>();

  // Create the MIRA BatteryState
  mira::robot::BatteryState battery_state;
  battery_state.voltage = 5.0;
  battery_state.current = -1.0;
  battery_state.lifeTime = -1;
  battery_state.lifePercent = 255;
  battery_state.charging = false;
  battery_state.powerSupplyPresent = false;

  // Convert to ROS BatteryState
  auto battery = module->miraToRosBatteryState(battery_state, mira::Time());

  // Check the conversion
  EXPECT_DOUBLE_EQ(battery.voltage, battery_state.voltage);
  EXPECT_DOUBLE_EQ(battery.current, -battery_state.current);
  EXPECT_TRUE(isnan(battery.charge));
  EXPECT_TRUE(isnan(battery.percentage));
  EXPECT_EQ(
    battery.power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
}

TEST(ScitosChargerTest, chargerStatus) {
  // Create the module
  auto module = std::make_shared<ChargerFixture>();

  // Convert to ROS ChargerStatus: all flags set
  uint8_t status = 0xFF;
  mira::Time time = mira::Time().now();
  auto charger = module->miraToRosChargerStatus(status, time);

  // Check the conversion
  EXPECT_EQ(charger.header.frame_id, "base_link");
  EXPECT_EQ(charger.header.stamp, rclcpp::Time(time.toUnixNS()));
  EXPECT_TRUE(charger.charging);
  EXPECT_TRUE(charger.empty);
  EXPECT_TRUE(charger.full);
  EXPECT_TRUE(charger.derating);
  EXPECT_TRUE(charger.charging_disabled);
  EXPECT_TRUE(charger.const_volt_mode);
  EXPECT_TRUE(charger.const_current_mode);
  EXPECT_TRUE(charger.internal_error_flag);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  mira::Framework framework(0, nullptr);
  framework.start();
  bool success = RUN_ALL_TESTS();
  return success;
}
