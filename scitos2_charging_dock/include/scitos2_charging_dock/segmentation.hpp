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

#ifndef SCITOS2_CHARGING_DOCK__SEGMENTATION_HPP_
#define SCITOS2_CHARGING_DOCK__SEGMENTATION_HPP_

// C++
#include <mutex>
#include <vector>
#include <string>

// ROS
#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "scitos2_charging_dock/cluster.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace scitos2_charging_dock
{

using Clusters = std::vector<Cluster>;

/**
 * @class scitos2_charging_dock::Segmentation
 * @brief Class to perform segmentation on the laserscan to get the clusters.
 *
 */
class Segmentation
{
public:
  /**
   * @brief Create a segmentation instance. Configure ROS 2 parameters.
   *
   * @param node The ROS 2 node
   * @param name The name of the segmentation
   */
  explicit Segmentation(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);

  /**
   * @brief Destroy the Segmentation object
   */
  ~Segmentation() = default;

  /**
   * @brief Perform a segmentation using a euclidean distance base clustering.
   *
   * @param scan The laserscan to clustering
   * @param clusters The clusters obtained
   * @return bool If the segmentation was successful
   */
  bool performSegmentation(const sensor_msgs::msg::LaserScan & scan, Clusters & clusters);

  /**
   * @brief Filter the clusters based on the number of points, the distance between them, etc.
   *
   * @param clusters The clusters to filter
   * @return Clusters The filtered clusters
   */
  Clusters filterClusters(const Clusters & clusters);

  /**
   * @brief Segment a scan and filter the resulting clusters in one step.
   *
   * @param scan The scan to process
   * @return Clusters The clusters
   */
  Clusters extractClustersFromScan(const sensor_msgs::msg::LaserScan & scan);

  /**
   * @brief Load a pointcloud from a PCD file. Used both for the ICP dock template and by
   * DockSaver, which has no need for the rest of Perception (ICP, TF) just to read a PCD file.
   *
   * @param filepath The path to the file
   * @param dock The loaded pointcloud
   * @return bool If the file was loaded
   */
  static bool loadDockPointcloud(const std::string & filepath, Pcloud & dock);

  /**
   * @brief Store a pointcloud to a PCD file.
   *
   * @param filepath The path to the file
   * @param dock The pointcloud to store
   * @return bool If the file was stored
   */
  static bool storeDockPointcloud(const std::string & filepath, const Pcloud & dock);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:
  /**
   * @brief Convert a polar point to a cartesian point.
   *
   * @param range The range of the point
   * @param angle The angle of the point
   * @return geometry_msgs::msg::Point The cartesian point
   */
  geometry_msgs::msg::Point fromPolarToCartesian(double range, double angle);

  /**
   * @brief Convert a laserscan to a vector of points.
   *
   * @param scan The laserscan to convert
   * @return std::vector<geometry_msgs::msg::Point> The points
   */
  std::vector<geometry_msgs::msg::Point> scanToPoints(const sensor_msgs::msg::LaserScan & scan);

  /**
   * @brief Calculate the euclidean distance between two points.
   *
   * @param p1 The first point
   * @param p2 The second point
   * @return double The distance
   */
  double euclideanDistance(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

  /**
   * @brief Check if there is a jump between two points.
   *
   * @param p1 The first point
   * @param p2 The second point
   * @param threshold The threshold to consider a jump
   * @return bool If there is a jump
   */
  bool isJumpBetweenPoints(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double threshold);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Name of the segmentation
  std::string name_;

  // Distance threshold for the segmentation
  double distance_threshold_;
  // Minimum number of points in a cluster
  int min_points_cluster_;
  // Maximum number of points in a cluster
  int max_points_cluster_;
  // Minimum distance from sensor to the centroid of the cluster
  double min_avg_distance_from_sensor_;
  // Maximum distance from sensor to the centroid of the cluster
  double max_avg_distance_from_sensor_;
  // Minimum width of the cluster
  double min_cluster_width_;
  // Maximum width of the cluster
  double max_cluster_width_;
};

}  // namespace scitos2_charging_dock

#endif  // SCITOS2_CHARGING_DOCK__SEGMENTATION_HPP_
