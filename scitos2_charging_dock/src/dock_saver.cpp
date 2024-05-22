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

#include "scitos2_charging_dock/dock_saver.hpp"

#include <string>
#include <memory>
#include <mutex>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace scitos2_charging_dock
{
DockSaver::DockSaver(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("dock_saver", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating the dock saver node");

  // Declare the node parameters
  declare_parameter("save_dock_timeout", 2.0);
}

DockSaver::~DockSaver()
{
}

nav2_util::CallbackReturn
DockSaver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  save_dock_timeout_ = std::make_shared<rclcpp::Duration>(
    rclcpp::Duration::from_seconds(get_parameter("save_dock_timeout").as_double()));

  // Create a service that saves the pointcloud from a dock to a file
  save_dock_service_ = create_service<scitos2_msgs::srv::SaveDock>(
    service_prefix + save_dock_service_name_,
    std::bind(&DockSaver::saveDockCallback, this, std::placeholders::_1, std::placeholders::_2));

  // perception_ = std::make_shared<Perception>(get_node_options());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockSaver::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockSaver::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockSaver::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  save_dock_service_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockSaver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool DockSaver::saveDockCallback(
  const std::shared_ptr<scitos2_msgs::srv::SaveDock::Request> request,
  std::shared_ptr<scitos2_msgs::srv::SaveDock::Response> response)
{
  RCLCPP_INFO(
    get_logger(), "Saving dock pointcloud from %s topic to file: %s", request->scan_topic.c_str(),
    request->dock_url.c_str());

  // Correct scan_topic if necessary
  std::string scan_topic = request->scan_topic;
  if (scan_topic == "") {
    scan_topic = "scan";
    RCLCPP_WARN(
      get_logger(), "Scan topic unspecified. Using default scan topic: %s.", scan_topic.c_str());
  }

  // Checking dock file name
  std::string filename = request->dock_url;
  if (filename == "") {
    filename = "dock_" + std::to_string(static_cast<int>(get_clock()->now().seconds()));
    RCLCPP_WARN(
      get_logger(), "Dock file unspecified. Dock will be saved to %s file", filename.c_str());
  }

  // Create a callback function that receives the scan from subscribed topic
  std::promise<sensor_msgs::msg::LaserScan::SharedPtr> prom;
  std::future<sensor_msgs::msg::LaserScan::SharedPtr> future_result = prom.get_future();
  auto scanCallback = [&prom](
    const sensor_msgs::msg::LaserScan::SharedPtr msg) -> void {
      prom.set_value(msg);
    };

  // Create new CallbackGroup for scan subscription
  auto callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto option = rclcpp::SubscriptionOptions();
  option.callback_group = callback_group;

  auto scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
    request->scan_topic, 10, scanCallback, option);

  // Create SingleThreadedExecutor to spin scan_sub in callback_group
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_callback_group(callback_group, get_node_base_interface());
  // Spin until dock message received
  auto timeout = save_dock_timeout_->to_chrono<std::chrono::nanoseconds>();
  auto status = executor.spin_until_future_complete(future_result, timeout);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to spin scan subscription");
    return false;
  }
  // scan_sub is no more needed
  scan_sub.reset();

  // Scan message received. Extracting dock pointcloud
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg = future_result.get();

  // Extract the dock pointcloud from the scan
  scitos2_charging_dock::Pcloud dock;
  /*if (!perception_->extractDockPointcloud(*scan_msg, dock)) {
    RCLCPP_ERROR(get_logger(), "Failed to extract dock pointcloud");
    return false;
  }*/

  // Store the dock pointcloud to a file
  response->result = perception_->storeDockPointcloud(filename, dock);

  return true;
}
}  // namespace scitos2_charging_dock

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(scitos2_charging_dock::DockSaver)
