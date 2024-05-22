/*
 * DOCKING PERCEPTION CLASS
 *
 * Copyright (c) 2020-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 *
 * This file is part of auto_docking.
 *
 * All rights reserved.
 *
 */

// PCL
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

// TF
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "scitos2_charging_dock/perception.hpp"

namespace scitos2_charging_dock
{

Perception::Perception(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.icp_min_score", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.icp_max_iter", rclcpp::ParameterValue(300));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.icp_max_corr_dis", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.icp_max_trans_eps", rclcpp::ParameterValue(0.000000001));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.icp_max_eucl_fit_eps", rclcpp::ParameterValue(0.000000001));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.enable_debug", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.dock_template_path", rclcpp::ParameterValue(""));

  node->get_parameter(name_ + ".perception.icp_min_score", icp_min_score_);
  node->get_parameter(name_ + ".perception.icp_max_iter", icp_max_iter_);
  node->get_parameter(name_ + ".perception.icp_max_corr_dis", icp_max_corr_dis_);
  node->get_parameter(name_ + ".perception.icp_max_trans_eps", icp_max_trans_eps_);
  node->get_parameter(name_ + ".perception.icp_max_eucl_fit_eps", icp_max_eucl_fit_eps_);
  node->get_parameter(name_ + ".perception.enable_debug", debug_);

  std::string dock_template_path;
  node->get_parameter(name_ + ".perception.dock_template_path", dock_template_path);
  loadDockPointcloud(dock_template_path, dock_template_);

  // Publishers
  if (debug_) {
    dock_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("dock/cloud", 1);
    dock_template_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "dock/template", 1);
  }

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Perception::dynamicParametersCallback, this, std::placeholders::_1));

  segmentation_ = std::make_unique<Segmentation>(node, name);
}

Perception::~Perception()
{
  segmentation_.reset();
}

geometry_msgs::msg::PoseStamped Perception::getDockPose(const sensor_msgs::msg::LaserScan & scan)
{
  // Perform segmentation on the scan and filter the segments
  auto segments = segmentation_->performSegmentation(scan);
  auto filtered_segments = segmentation_->filterSegments(segments);

  // Convert the segments to clusters
  auto clusters = segmentsToClusters(scan.header.frame_id, filtered_segments);

  // Perform ICP matching to find the dock pose in the clusters


  geometry_msgs::msg::PoseStamped pose;
  pose.header = scan.header;
  return pose;


  /*std::lock_guard<std::mutex> param_lock(mutex_);



  // Filter the scan between lower_angle_ and upper_angle_
  // auto filtered_scan = angularFilterScan(*scan_msg, lower_angle_, upper_angle_);
  auto filtered_scan = *scan_msg;

  // Project the laser scan into a point cloud
  auto sensor_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  projector_.projectLaser(filtered_scan, *sensor_cloud_msg);

  // Convert the cloud to PCL
  Pcloud::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*sensor_cloud_msg, *pcl_cloud);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(dock_template_.segment_cloud);
  icp.setInputTarget(pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  RCLCPP_INFO(
    logger_, "has converged: %s score: %f",
    icp.hasConverged() ? "true" : "false", icp.getFitnessScore());*/

  // Perform min cut segmentation to extract the dock
  /*pcl::IndicesPtr indices(new std::vector<int>);
  pcl::MinCutSegmentation<pcl::PointXYZ> seg;
  seg.setInputCloud(pcl_cloud);
  seg.setIndices(indices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointXYZ point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  foreground_points->points.push_back(point);
  seg.setForegroundPoints(foreground_points);
  seg.setSigma(0.25);
  seg.setRadius(3.0433856);
  seg.setNumberOfNeighbours(14);
  seg.setSourceWeight(0.8);

  std::vector<pcl::PointIndices> clusters;
  seg.extract(clusters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud();*/


  // Filter the cloud using a passthrough filter to remove points outside angle range
  /*PCloud::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(, 1.0);
  pass.filter(*cloud_filtered);*/
/*
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pcl_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.05); // 2cm
  ec.setMinClusterSize(3);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_cloud);
  ec.extract(cluster_indices);

  // Crea una nube de puntos para cada cluster
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  for (long unsigned int i = 0; i < cluster_indices.size(); i++) {
    clusters.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()));
    for (long unsigned int j = 0; j < cluster_indices[i].indices.size(); j++) {
      clusters[i]->push_back(pcl_cloud->at(cluster_indices[i].indices[j]));
    }
  }

  // Crea una nube de puntos con los clusters
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (long unsigned int i = 0; i < clusters.size(); i++) {
    for (long unsigned int j = 0; j < clusters[i]->size(); j++) {
      pcl::PointXYZRGB point;
      point.x = clusters[i]->at(j).x;
      point.y = clusters[i]->at(j).y;
      point.z = clusters[i]->at(j).z;
      point.r = static_cast<unsigned char>(255 * (i % 256));
      point.g = static_cast<unsigned char>(255 * ((i / 256) % 256));
      point.b = static_cast<unsigned char>(255 * (i / 65536));
      cloud_clusters->push_back(point);
    }
  }

  RCLCPP_INFO(logger_, "Number of clusters: %ld", clusters.size() );
*/
// Convert the cloud to ROS message
/*
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(Final, cloud_msg);
  cloud_msg.header.frame_id = scan_msg->header.frame_id;
  cloud_msg.header.stamp = scan_msg->header.stamp;
  cloud_pub_->publish(std::move(cloud_msg));
  */


  // Publish the template
  /* if (debug_) {
    sensor_msgs::msg::PointCloud2 dock_template_msg;
    pcl::toROSMsg(*(dock_template_.segment_cloud), dock_template_msg);
    dock_template_msg.header.frame_id = scan_msg->header.frame_id;
    dock_template_msg.header.stamp = scan_msg->header.stamp;
    dock_template_pub_->publish(std::move(dock_template_msg));
  } */
}

void Perception::setStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  staging_pose_ = pose;
  staging_frame_ = frame;
}

bool Perception::loadDockPointcloud(std::string filepath, Pcloud & dock)
{
  if (filepath.empty()) {
    RCLCPP_ERROR(logger_, "Couldn't load the dock from an empty file path");
  } else {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, dock) < 0) {
      RCLCPP_ERROR(logger_, "Failed to load the dock from PCD file");
    } else {
      RCLCPP_INFO(logger_, "Dock loaded from PCD file");
      return true;
    }
  }
  return false;
}

bool Perception::storeDockPointcloud(std::string filepath, const Pcloud & dock)
{
  bool success = false;
  if (pcl::io::savePCDFile<pcl::PointXYZ>(filepath, dock) < 0) {
    RCLCPP_ERROR(logger_, "Failed to save the dock to PCD file");
  } else {
    RCLCPP_INFO(logger_, "Dock saved to PCD file");
    return true;
  }
  return success;
}

Clusters Perception::segmentsToClusters(std::string frame_id, const Segments & segments)
{
  Clusters clusters;
  for (const auto & segment : segments) {
    Cluster cluster;
    cluster.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cluster.matched_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cluster.cloud->header.frame_id = frame_id;
    for (const auto & point : segment.points) {
      pcl::PointXYZ pcl_point;
      pcl_point.x = point.x;
      pcl_point.y = point.y;
      pcl_point.z = point.z;
      cluster.cloud->push_back(pcl_point);
    }
    clusters.push_back(cluster);
  }
  return clusters;
}

bool Perception::refineClusterPose(Cluster & cluster, Pcloud::ConstPtr cloud_template)
{
  // Prepare the ICP object
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(icp_max_iter_);
  icp.setMaxCorrespondenceDistance(icp_max_corr_dis_);
  icp.setTransformationEpsilon(icp_max_trans_eps_);
  icp.setEuclideanFitnessEpsilon(icp_max_eucl_fit_eps_);

  // Align the cluster to the template
  icp.setInputSource(cloud_template);
  icp.setInputTarget(cluster.cloud);
  Pcloud::Ptr matched_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  icp.align(*matched_cloud);

  // If the ICP converged, store the results on the cluster
  if (icp.hasConverged()) {
    // Get the final transformation
    auto icp_refinement = eigenToTransform(icp.getFinalTransformation());
    // Transform the pose to the matching frame
    tf2::Transform tf_stage;
    tf2::fromMsg(staging_pose_, tf_stage);
    tf2::Transform tf_correct = icp_refinement * tf_stage;
    // Convert to ROS msg
    geometry_msgs::msg::Pose pose_correct;
    tf2::toMsg(tf_correct, pose_correct);

    // Update the cluster
    cluster.icp_score = icp.getFitnessScore();
    cluster.icp_pose.header.frame_id = "map";
    cluster.icp_pose.header.stamp = clock_->now();
    cluster.icp_pose.pose = pose_correct;
    cluster.matched_cloud = matched_cloud;

    RCLCPP_DEBUG(logger_, "ICP score cluster = %f", cluster.icp_score);
    RCLCPP_DEBUG(
      logger_, "ICP transformation: (%f, %f)", cluster.icp_pose.pose.position.x,
      cluster.icp_pose.pose.position.y);

    return true;
  } else {
    RCLCPP_WARN(logger_, "ICP failed to converge. Please check the parameters");
    return false;
  }
}

tf2::Transform Perception::eigenToTransform(const Eigen::Matrix4f & T)
{
  return tf2::Transform(
    tf2::Matrix3x3(
      T(0, 0), T(0, 1), T(0, 2),
      T(1, 0), T(1, 1), T(1, 2),
      T(2, 0), T(2, 1), T(2, 2)),
    tf2::Vector3(T(0, 3), T(1, 3), T(2, 3)));
}

rcl_interfaces::msg::SetParametersResult Perception::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(dynamic_params_lock_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".perception.icp_max_iter") {
        icp_max_iter_ = parameter.as_int();
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".perception.icp_min_score") {
        icp_min_score_ = parameter.as_double();
      } else if (name == name_ + ".perception.icp_max_corr_dis") {
        icp_max_corr_dis_ = parameter.as_double();
      } else if (name == name_ + ".perception.icp_max_trans_eps") {
        icp_max_trans_eps_ = parameter.as_double();
      } else if (name == name_ + ".perception.icp_max_eucl_fit_eps") {
        icp_max_eucl_fit_eps_ = parameter.as_double();
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".perception.enable_debug") {
        debug_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace scitos2_charging_dock
