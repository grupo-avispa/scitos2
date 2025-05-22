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

#ifndef SCITOS2_CHARGING_DOCK__PERCEPTION_HPP_
#define SCITOS2_CHARGING_DOCK__PERCEPTION_HPP_

// PCL
#include <pcl/point_types.h>

// C++
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "scitos2_charging_dock/cluster.hpp"
#include "scitos2_charging_dock/segmentation.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.h"

namespace scitos2_charging_dock
{

/**
 * @class scitos2_charging_dock::Perception
 * @brief Class to perform ICP matching on clusters to get the docking station.
 *
 */
class Perception
{
public:
  /**
   * @brief Create a perception instance. Configure ROS 2 parameters.
   *
   * @param node The ROS 2 node
   * @param name The name of the perception
   * @param tf The tf buffer
   */
  Perception(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief Destroy the Perception object
   */
  ~Perception();

  /**
   * @brief Store the dock pointcloud to a PCD file.
   *
   * @param filepath The path to the file
   * @param dock The dock to store
   * @return bool If the file was stored
   */
  bool storeDockPointcloud(std::string filepath, const Pcloud & dock);

  /**
   * @brief Get the dock pose from the scan.
   *
   * @param scan The scan to process
   * @return geometry_msgs::msg::PoseStamped The dock pose
   */
  geometry_msgs::msg::PoseStamped getDockPose(const sensor_msgs::msg::LaserScan & scan);

  /**
   * @brief Set the initial estimate of the dock pose.
   *
   * @param pose The initial estimate of the dock pose
   * @param frame The frame of the pose
   */
  void setInitialEstimate(const geometry_msgs::msg::Pose & pose, const std::string & frame);

  /**
   * @brief Extract clusters from a scan.
   *
   * @param scan The scan to process
   * @return Clusters The clusters
   */
  Clusters extractClustersFromScan(const sensor_msgs::msg::LaserScan & scan);

protected:
  /**
   * @brief Refine the cluster pose using Iterative Closest Point.
   *
   * @param cluster The cluster to perform ICP on
   * @param cloud_template The template to match with the cluster
   * @return bool If the ICP was successful and the template is aligned with the cluster
   */
  bool refineClusterPose(Cluster & cluster, const Pcloud & cloud_template);

  /**
   * @brief Refine all the clusters poses using Iterative Closest Point to find the dock.
   *
   * @param clusters The clusters to perform ICP on
   * @param dock_template A cluster with a cloud template to match with the clusters
   * @param dock The dock found
   * @return bool If the dock is found
   */
  bool refineAllClustersPoses(Clusters & clusters, const Cluster & dock_template, Cluster & dock);

  /**
   * @brief Load the dock template from a PCD file.
   *
   * @param filepath The path to the file
   * @param dock The loaded dock
   * @return bool If the file was loaded
   */
  bool loadDockPointcloud(std::string filepath, Pcloud & dock);

  /**
   * @brief Create a PointCloud2 message from a PCL pointcloud.
   *
   * @param cloud The pointcloud
   * @return sensor_msgs::msg::PointCloud2 The PointCloud2 message
   */
  sensor_msgs::msg::PointCloud2 createPointCloud2Msg(const Pcloud & cloud);

  /**
   * @brief Convert a Eigen matrix to a tf2 Transform.
   *
   * @param T The Eigen matrix
   * @return tf2::Transform The transform
   */
  tf2::Transform eigenToTransform(const Eigen::Matrix4f & T);

  /**
   * @brief Callback executed when a parameter change is detected.
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Debug flag for visualization
  bool debug_;
  // ICP parameters
  int icp_max_iter_;
  double icp_min_score_;
  double icp_max_corr_dis_;
  double icp_max_trans_eps_;
  double icp_max_eucl_fit_eps_;
  // Initial estimate of the dock pose
  geometry_msgs::msg::PoseStamped initial_estimate_pose_;
  // Segmentation
  std::unique_ptr<Segmentation> segmentation_;

  // A cluster with the dock template pointcloud
  Cluster dock_template_;
  // Last detected dock
  Cluster detected_dock_;
  // Dock found
  bool dock_found_;
  // Only use the first detection
  bool use_first_detection_{false};

  std::string name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("DockingPerception")};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Publishers for the dock cloud and dock template
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dock_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dock_template_pub_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;
};

}  // namespace scitos2_charging_dock

#endif  // SCITOS2_CHARGING_DOCK__PERCEPTION_HPP_
