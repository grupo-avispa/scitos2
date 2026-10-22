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

#include "scitos2_modules/channel_bridge.hpp"

#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace scitos2_modules
{

void ChannelBridge::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
  auto node = configureMira(parent, name);

  std::vector<std::string> channels;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".channels",
    rclcpp::ParameterValue(std::vector<std::string>()), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Names of the channels to bridge, each one configured under its own name"));
  node->get_parameter(plugin_name_ + ".channels", channels);

  for (const auto & id : channels) {
    const std::string prefix = plugin_name_ + "." + id;
    std::string mira_channel, type, ros_topic, frame_id = "base_link";
    declare_parameter_if_not_declared(
      node, prefix + ".mira_channel", rclcpp::ParameterValue(""),
      rcl_interfaces::msg::ParameterDescriptor().set__description("Channel of MIRA to bridge"));
    declare_parameter_if_not_declared(
      node, prefix + ".type", rclcpp::ParameterValue(""),
      rcl_interfaces::msg::ParameterDescriptor()
      .set__description(
        "Type of the channel: bool, float, uint8, uint32, velocity2, pose2 or point3f"));
    declare_parameter_if_not_declared(
      node, prefix + ".ros_topic", rclcpp::ParameterValue(""),
      rcl_interfaces::msg::ParameterDescriptor().set__description("ROS topic to publish to"));
    declare_parameter_if_not_declared(
      node, prefix + ".frame_id", rclcpp::ParameterValue(frame_id),
      rcl_interfaces::msg::ParameterDescriptor()
      .set__description("Frame of the messages that have a header"));
    node->get_parameter(prefix + ".mira_channel", mira_channel);
    node->get_parameter(prefix + ".type", type);
    node->get_parameter(prefix + ".ros_topic", ros_topic);
    node->get_parameter(prefix + ".frame_id", frame_id);

    if (mira_channel.empty() || ros_topic.empty()) {
      RCLCPP_ERROR(
        logger_, "The channel '%s' needs the parameters mira_channel and ros_topic, skipping it",
        id.c_str());
      continue;
    }
    if (!bridgeByType(node, mira_channel, type, ros_topic, frame_id)) {
      RCLCPP_ERROR(
        logger_, "The channel '%s' has the unsupported type '%s', skipping it",
        id.c_str(), type.c_str());
      continue;
    }
    RCLCPP_INFO(
      logger_, "Bridging the MIRA channel %s to the topic %s", mira_channel.c_str(),
      ros_topic.c_str());
  }

  RCLCPP_INFO(logger_, "Configured module : %s", plugin_name_.c_str());
}

void ChannelBridge::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type scitos2_module::ChannelBridge",
    plugin_name_.c_str());
  publishers_.clear();
  cleanupMira();
}

bool ChannelBridge::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type scitos2_module::ChannelBridge",
    plugin_name_.c_str());
  for (const auto & publisher : publishers_) {
    publisher->on_activate();
  }
  if (!startMira()) {
    return false;
  }
  is_active_ = true;
  return true;
}

bool ChannelBridge::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type scitos2_module::ChannelBridge",
    plugin_name_.c_str());
  is_active_ = false;
  for (const auto & publisher : publishers_) {
    publisher->on_deactivate();
  }
  return true;
}

template<typename MiraT, typename RosT, typename Convert>
void ChannelBridge::bridge(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & channel,
  const std::string & topic, const std::string & frame_id, Convert convert)
{
  auto publisher = node->create_publisher<RosT>(topic, rclcpp::SystemDefaultsQoS());
  publishers_.push_back(publisher);
  authority_->subscribe<MiraT>(
    channel, [this, publisher, frame_id, convert](mira::ChannelRead<MiraT> data) {
      if (!is_active_) {
        return;
      }
      RosT msg;
      convert(data->value(), rclcpp::Time(data->timestamp.toUnixNS()), frame_id, msg);
      publisher->publish(msg);
    });
}

bool ChannelBridge::bridgeByType(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & channel,
  const std::string & type, const std::string & topic, const std::string & frame_id)
{
  if (type == "bool") {
    bridge<bool, std_msgs::msg::Bool>(
      node, channel, topic, frame_id,
      [](bool value, const rclcpp::Time &, const std::string &, std_msgs::msg::Bool & msg) {
        msg.data = value;
      });
  } else if (type == "float") {
    bridge<float, std_msgs::msg::Float32>(
      node, channel, topic, frame_id,
      [](float value, const rclcpp::Time &, const std::string &, std_msgs::msg::Float32 & msg) {
        msg.data = value;
      });
  } else if (type == "uint8") {
    bridge<uint8, std_msgs::msg::UInt8>(
      node, channel, topic, frame_id,
      [](uint8 value, const rclcpp::Time &, const std::string &, std_msgs::msg::UInt8 & msg) {
        msg.data = value;
      });
  } else if (type == "uint32") {
    bridge<uint32, std_msgs::msg::UInt32>(
      node, channel, topic, frame_id,
      [](uint32 value, const rclcpp::Time &, const std::string &, std_msgs::msg::UInt32 & msg) {
        msg.data = value;
      });
  } else if (type == "velocity2") {
    bridge<mira::Velocity2, geometry_msgs::msg::TwistStamped>(
      node, channel, topic, frame_id,
      [](const mira::Velocity2 & value, const rclcpp::Time & stamp, const std::string & frame,
      geometry_msgs::msg::TwistStamped & msg) {
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.twist.linear.x = value.x();
        msg.twist.linear.y = value.y();
        msg.twist.angular.z = value.phi();
      });
  } else if (type == "pose2") {
    bridge<mira::Pose2, geometry_msgs::msg::PoseStamped>(
      node, channel, topic, frame_id,
      [](const mira::Pose2 & value, const rclcpp::Time & stamp, const std::string & frame,
      geometry_msgs::msg::PoseStamped & msg) {
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.pose.position.x = value.x();
        msg.pose.position.y = value.y();
        msg.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, value.phi()));
      });
  } else if (type == "point3f") {
    bridge<mira::Point3f, geometry_msgs::msg::PointStamped>(
      node, channel, topic, frame_id,
      [](const mira::Point3f & value, const rclcpp::Time & stamp, const std::string & frame,
      geometry_msgs::msg::PointStamped & msg) {
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.point.x = value.x();
        msg.point.y = value.y();
        msg.point.z = value.z();
      });
  } else {
    return false;
  }
  return true;
}

}  // namespace scitos2_modules

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(scitos2_modules::ChannelBridge, scitos2_core::Module)
