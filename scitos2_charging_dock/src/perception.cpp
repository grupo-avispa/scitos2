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

// PCL
#include <pcl/common/eigen.h>
#include <pcl/registration/icp.h>

// C++
#include <algorithm>

// TF
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scitos2_charging_dock/dock_selector.hpp"
#include "scitos2_charging_dock/perception.hpp"

namespace scitos2_charging_dock
{

Perception::Perception(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_buffer_ = tf;

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
    node, name_ + ".perception.dock_template", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.use_first_detection", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.max_yaw_error", rclcpp::ParameterValue(M_PI_2));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.width_tolerance_min", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".perception.width_tolerance_max", rclcpp::ParameterValue(1.25));

  node->get_parameter(name_ + ".perception.icp_min_score", icp_min_score_);
  node->get_parameter(name_ + ".perception.icp_max_iter", icp_max_iter_);
  node->get_parameter(name_ + ".perception.icp_max_corr_dis", icp_max_corr_dis_);
  node->get_parameter(name_ + ".perception.icp_max_trans_eps", icp_max_trans_eps_);
  node->get_parameter(name_ + ".perception.icp_max_eucl_fit_eps", icp_max_eucl_fit_eps_);
  node->get_parameter(name_ + ".perception.enable_debug", debug_);
  node->get_parameter(name_ + ".perception.use_first_detection", use_first_detection_);
  node->get_parameter(name_ + ".perception.max_yaw_error", max_yaw_error_);
  node->get_parameter(name_ + ".perception.width_tolerance_min", width_tolerance_min_);
  node->get_parameter(name_ + ".perception.width_tolerance_max", width_tolerance_max_);

  // Load the dock template
  std::string dock_template;
  node->get_parameter(name_ + ".perception.dock_template", dock_template);
  dock_template_loaded_ = Segmentation::loadDockPointcloud(dock_template, dock_template_.cloud);
  if (!dock_template_loaded_) {
    RCLCPP_ERROR(
      logger_,
      "Perception will never detect a dock until 'perception.dock_template' points to a "
      "valid PCD file. Generate one for your dock with the save_dock service.");
  }

  // Publishers: always created so that toggling enable_debug at runtime cannot
  // publish through a null pointer
  target_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("dock/target", 1);
  dock_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("dock/cloud", 1);
  dock_template_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("dock/template", 1);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Perception::dynamicParametersCallback, this, std::placeholders::_1));

  // Setup the segmentation object
  segmentation_ = std::make_unique<Segmentation>(node, name);

  // Set the verbosity level of the PCL library to shutout the console output
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

geometry_msgs::msg::PoseStamped Perception::getDockPose(const sensor_msgs::msg::LaserScan & scan)
{
  if (!dock_template_loaded_) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000,
      "Cannot detect the dock: no valid dock template has been loaded");
    return detected_dock_.pose;
  }

  // Extract clusters from the scan
  auto clusters = segmentation_->extractClustersFromScan(scan);

  // Refine the pose of each cluster to get the dock pose
  // If we only wants the first detection, just update the timestamp
  if (use_first_detection_ && dock_found_) {
    detected_dock_.pose.header.stamp = clock_->now();
  } else {
    if (refineAllClustersPoses(clusters, dock_template_, detected_dock_)) {
      dock_found_ = true;
    }
  }

  return detected_dock_.pose;
}

void Perception::setInitialEstimate(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  initial_estimate_pose_.pose = pose;
  initial_estimate_pose_.header.frame_id = frame;
  initial_estimate_pose_.header.stamp = clock_->now();
  dock_found_ = false;
}

bool Perception::refineClusterPose(Cluster & cluster, const Pcloud & cloud_template)
{
  bool success = false;

  // Prepare the ICP object
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(icp_max_iter_);
  icp.setMaxCorrespondenceDistance(icp_max_corr_dis_);
  icp.setTransformationEpsilon(icp_max_trans_eps_);
  icp.setEuclideanFitnessEpsilon(icp_max_eucl_fit_eps_);

  // Align the cluster to the template
  Pcloud::Ptr cloud_template_ptr(new Pcloud(cloud_template));
  Pcloud::Ptr cluster_cloud_ptr(new Pcloud(cluster.cloud));
  Pcloud matched_cloud;

  icp.setInputSource(cloud_template_ptr);
  icp.setInputTarget(cluster_cloud_ptr);
  icp.align(matched_cloud);

  // If the ICP converged, store the results on the cluster
  if (icp.hasConverged()) {
    // Get the final transformation
    auto icp_refinement = eigenToTransform(icp.getFinalTransformation());
    // Transform the pose to the matching frame
    tf2::Transform tf_stage;
    tf2::fromMsg(initial_estimate_pose_.pose, tf_stage);
    tf2::Transform tf_correct = icp_refinement * tf_stage;
    // Convert to ROS msg
    geometry_msgs::msg::Pose pose_correct;
    tf2::toMsg(tf_correct, pose_correct);

    // Update the cluster
    cluster.score = icp.getFitnessScore();
    cluster.pose.header.frame_id = initial_estimate_pose_.header.frame_id;
    cluster.pose.header.stamp = clock_->now();
    cluster.pose.pose = pose_correct;
    cluster.cloud = matched_cloud;

    success = true;

    RCLCPP_DEBUG(logger_, "ICP converged for cluster %i", cluster.id);
    RCLCPP_DEBUG(logger_, "ICP fitness score: %f", cluster.score);
    RCLCPP_DEBUG(
      logger_, "ICP transformation: (%f, %f)",
      cluster.pose.pose.position.x, cluster.pose.pose.position.y);
  } else {
    RCLCPP_DEBUG(logger_, "ICP did not converge for cluster %i", cluster.id);
  }

  return success;
}

bool Perception::refineAllClustersPoses(
  Clusters & clusters, const Cluster & dock_template, Cluster & dock)
{
  bool success = false;
  Clusters refined_candidates;

  for (auto & cluster : clusters) {
    // Discard clusters not valid (i.e. clusters not similar to dock by size, ...)
    if (!cluster.valid(dock_template.width(), width_tolerance_min_, width_tolerance_max_)) {
      continue;
    }

    // Prepares the template pointcloud for matching by transforming to the initial estimate pose.
    // Usually in global coordinates (map frame)
    Pcloud cloud_template_initial;
    tf2::Transform tf_stage;
    tf2::fromMsg(initial_estimate_pose_.pose, tf_stage);
    pcl_ros::transformPointCloud(dock_template.cloud, cloud_template_initial, tf_stage);
    cloud_template_initial.header.frame_id = initial_estimate_pose_.header.frame_id;

    // Transforms the target point cloud from the scan frame to the global frame (map frame)
    if (cluster.cloud.header.frame_id != initial_estimate_pose_.header.frame_id) {
      try {
        if (!tf_buffer_->canTransform(
            initial_estimate_pose_.header.frame_id, cluster.cloud.header.frame_id,
            rclcpp::Time(cluster.cloud.header.stamp * 1000), rclcpp::Duration::from_seconds(0.2)))
        {
          RCLCPP_WARN(
            logger_, "Could not transform %s to %s",
            cluster.cloud.header.frame_id.c_str(), initial_estimate_pose_.header.frame_id.c_str());
          continue;
        }
        auto tf_stamped = tf_buffer_->lookupTransform(
          initial_estimate_pose_.header.frame_id, cluster.cloud.header.frame_id,
          tf2::TimePointZero);
        pcl_ros::transformPointCloud(cluster.cloud, cluster.cloud, tf_stamped);
        cluster.cloud.header.frame_id = initial_estimate_pose_.header.frame_id;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          logger_, "Could not transform %s to %s: %s",
          cluster.cloud.header.frame_id.c_str(),
          initial_estimate_pose_.header.frame_id.c_str(), ex.what());
        continue;
      }
    }

    // Visualizes the target point cloud and estimated dock template pose
    if (debug_) {
      // Publish dock template and target cloud
      dock_template_pub_->publish(createPointCloud2Msg(dock_template.cloud));
      target_cloud_pub_->publish(createPointCloud2Msg(cluster.cloud));
    }

    // Refine the cluster to get the dock pose
    if (refineClusterPose(cluster, cloud_template_initial)) {
      refined_candidates.push_back(cluster);
    }
  }

  // Decide which (if any) refined candidate is the dock. Pure decision logic, kept out of
  // this method so it can be unit tested without a node, TF or PCL/ICP involved.
  auto selected = selectDock(
    refined_candidates, initial_estimate_pose_, {icp_min_score_, max_yaw_error_});

  if (selected.has_value()) {
    dock = *selected;
    // Publish the dock cloud
    if (debug_) {
      dock_cloud_pub_->publish(createPointCloud2Msg(dock.cloud));
    }
    success = true;
    RCLCPP_DEBUG(logger_, "Dock successfully identified at cluster %i", dock.id);
  } else {
    RCLCPP_WARN(logger_, "Unable to identify the dock");
  }

  return success;
}

sensor_msgs::msg::PointCloud2 Perception::createPointCloud2Msg(const Pcloud & cloud)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = clock_->now();
  return msg;
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
        if (parameter.as_int() <= 0) {
          result.successful = false;
          result.reason = "icp_max_iter must be positive";
          return result;
        }
        icp_max_iter_ = parameter.as_int();
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".perception.icp_min_score") {
        if (parameter.as_double() < 0.0) {
          result.successful = false;
          result.reason = "icp_min_score must not be negative";
          return result;
        }
        icp_min_score_ = parameter.as_double();
      } else if (name == name_ + ".perception.icp_max_corr_dis") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "icp_max_corr_dis must be positive";
          return result;
        }
        icp_max_corr_dis_ = parameter.as_double();
      } else if (name == name_ + ".perception.icp_max_trans_eps") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "icp_max_trans_eps must be positive";
          return result;
        }
        icp_max_trans_eps_ = parameter.as_double();
      } else if (name == name_ + ".perception.icp_max_eucl_fit_eps") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "icp_max_eucl_fit_eps must be positive";
          return result;
        }
        icp_max_eucl_fit_eps_ = parameter.as_double();
      } else if (name == name_ + ".perception.max_yaw_error") {
        if (parameter.as_double() < 0.0) {
          result.successful = false;
          result.reason = "max_yaw_error must not be negative";
          return result;
        }
        max_yaw_error_ = parameter.as_double();
      } else if (name == name_ + ".perception.width_tolerance_min") {
        if (parameter.as_double() < 0.0 || parameter.as_double() >= width_tolerance_max_) {
          result.successful = false;
          result.reason = "width_tolerance_min must not be negative and lower than "
            "width_tolerance_max";
          return result;
        }
        width_tolerance_min_ = parameter.as_double();
      } else if (name == name_ + ".perception.width_tolerance_max") {
        if (parameter.as_double() <= width_tolerance_min_) {
          result.successful = false;
          result.reason = "width_tolerance_max must be greater than width_tolerance_min";
          return result;
        }
        width_tolerance_max_ = parameter.as_double();
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".perception.enable_debug") {
        debug_ = parameter.as_bool();
      } else if (name == name_ + ".perception.use_first_detection") {
        use_first_detection_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace scitos2_charging_dock
