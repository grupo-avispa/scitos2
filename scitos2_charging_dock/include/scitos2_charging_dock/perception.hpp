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
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "scitos2_charging_dock/cluster.hpp"
#include "scitos2_charging_dock/segmentation.hpp"

namespace scitos2_charging_dock
{

using Clusters = std::vector<Cluster>;
using Pcloud = pcl::PointCloud<pcl::PointXYZ>;

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
   */
  explicit Perception(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);

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
   * @brief Set the staging pose as the given dock pose estimate.
   *
   * @param pose The dock pose
   * @param frame_id The frame of the pose
   */
  void setStagingPose(const geometry_msgs::msg::Pose & pose, const std::string & frame);

protected:
  /**
   * @brief Convert the segments to clusters.
   *
   * @param frame_id The frame of the segments
   * @param segments The segments to convert
   * @return Clusters The clusters
   */
  Clusters segmentsToClusters(std::string frame_id, const Segments & segments);

  /**
   * @brief Refine the cluster pose using Iterative Closest Point.
   *
   * @param cluster The cluster to perform ICP
   * @param cloud_template The template to match
   * @return bool If the ICP was successful
   */
  bool refineClusterPose(Cluster & cluster, Pcloud::ConstPtr cloud_template);

  /**
   * @brief Load the dock template from a PCD file.
   *
   * @param filepath The path to the file
   * @param dock The loaded dock
   * @return bool If the file was loaded
   */
  bool loadDockPointcloud(std::string filepath, Pcloud & dock);

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
  // Staging pose and frmae
  geometry_msgs::msg::Pose staging_pose_;
  std::string staging_frame_;
  // Segmentation
  std::unique_ptr<Segmentation> segmentation_;
  // Dock template pointcloud
  Pcloud dock_template_;

  std::string name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("DockingPerception")};

  // Publishers for the dock cloud and dock template
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dock_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dock_template_pub_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;
};

}  // namespace scitos2_charging_dock

#endif  // SCITOS2_CHARGING_DOCK__PERCEPTION_HPP_
