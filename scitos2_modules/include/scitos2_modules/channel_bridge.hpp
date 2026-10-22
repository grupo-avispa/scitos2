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

#ifndef SCITOS2_MODULES__CHANNEL_BRIDGE_HPP_
#define SCITOS2_MODULES__CHANNEL_BRIDGE_HPP_

#include <fw/Framework.h>
#include <geometry/Point.h>
#include <transform/Pose.h>
#include <transform/Velocity.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"
#include "scitos2_mira_utils/mira_module.hpp"

namespace scitos2_modules
{

/**
 * @class scitos2_modules::ChannelBridge
 * @brief Module that forwards channels of MIRA to ROS topics, configured only with parameters.
 *
 * Adding a datum of MIRA to ROS used to need a subscription, a conversion, a publisher and
 * tests. For the simple types it is now a few lines of YAML:
 *
 * @code{.yaml}
 * bridge:
 *   plugin: "scitos2_modules::ChannelBridge"
 *   channels: ["speed_limiter"]
 *   speed_limiter:
 *     mira_channel: "/robot/SpeedLimiterActive"
 *     type: "bool"
 *     ros_topic: "speed_limiter_active"
 * @endcode
 *
 * The supported types are `bool`, `float`, `uint8`, `uint32` (std_msgs), `velocity2`
 * (geometry_msgs/TwistStamped), `pose2` (geometry_msgs/PoseStamped) and `point3f`
 * (geometry_msgs/PointStamped). The stamped ones use `frame_id` as their frame, which is
 * `base_link` by default.
 */
class ChannelBridge : public scitos2_mira_utils::MiraModule
{
public:
  /**
   * @brief Construct for scitos2_modules::ChannelBridge
   */
  ChannelBridge() = default;

  /**
   * @brief Destructor for scitos2_modules::ChannelBridge
   */
  ~ChannelBridge() override = default;

  /**
   * @brief Configure the module.
   *
   * @param parent WeakPtr to node
   * @param name Name of plugin
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) override;

  /**
   * @brief Cleanup the module state machine.
   */
  void cleanup() override;

  /**
   * @brief Activate the module state machine.
   */
  bool activate() override;

  /**
   * @brief Deactivate the module state machine.
   */
  bool deactivate() override;

protected:
  /**
   * @brief Bridge one channel of MIRA to a ROS topic.
   *
   * @tparam MiraT Type of the channel of MIRA
   * @tparam RosT Type of the ROS message
   * @tparam Convert Callable that fills a ROS message from the data of MIRA and its header
   * @param node The node that owns the publisher
   * @param channel Channel of MIRA
   * @param topic ROS topic
   * @param frame_id Frame of the messages that have a header
   * @param convert Conversion from MIRA to ROS
   */
  template<typename MiraT, typename RosT, typename Convert>
  void bridge(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & channel,
    const std::string & topic, const std::string & frame_id, Convert convert);

  /**
   * @brief Create the bridge of a channel from its type name.
   *
   * @param node The node that owns the publisher
   * @param channel Channel of MIRA
   * @param type Name of the type (`bool`, `float`, `uint8`, `uint32`, `velocity2`, `pose2` or
   * `point3f`)
   * @param topic ROS topic
   * @param frame_id Frame of the messages that have a header
   * @return bool False if the type is not supported
   */
  bool bridgeByType(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & channel,
    const std::string & type, const std::string & topic, const std::string & frame_id);

  /** @brief Whether the data of MIRA is forwarded. */
  std::atomic<bool> is_active_{false};

  /** @brief Publishers of the bridged channels, to activate and deactivate them together. */
  std::vector<std::shared_ptr<rclcpp_lifecycle::ManagedEntityInterface>> publishers_;
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__CHANNEL_BRIDGE_HPP_
