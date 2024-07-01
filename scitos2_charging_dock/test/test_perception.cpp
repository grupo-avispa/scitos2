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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "scitos2_charging_dock/perception.hpp"

class PerceptionFixture : public scitos2_charging_dock::Perception
{
public:
  PerceptionFixture(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
  : Perception(node, name, tf)
  {
  }

  bool loadDockPointcloud(std::string filepath, scitos2_charging_dock::Pcloud & dock)
  {
    return scitos2_charging_dock::Perception::loadDockPointcloud(filepath, dock);
  }

  scitos2_charging_dock::Clusters segmentsToClusters(
    std::string frame_id, const scitos2_charging_dock::Segments & segments)
  {
    return scitos2_charging_dock::Perception::segmentsToClusters(frame_id, segments);
  }

  tf2::Transform eigenToTransform(const Eigen::Matrix4f & T)
  {
    return scitos2_charging_dock::Perception::eigenToTransform(T);
  }

  bool refineClusterPose(
    scitos2_charging_dock::Cluster & cluster,
    scitos2_charging_dock::Pcloud::ConstPtr cloud_template)
  {
    return scitos2_charging_dock::Perception::refineClusterPose(cluster, cloud_template);
  }

  bool refineAllClustersPoses(
    scitos2_charging_dock::Clusters & clusters,
    scitos2_charging_dock::Pcloud::ConstPtr cloud_template,
    geometry_msgs::msg::PoseStamped & dock_pose)
  {
    return scitos2_charging_dock::Perception::refineAllClustersPoses(
      clusters, cloud_template, dock_pose);
  }
};

geometry_msgs::msg::Point create_point(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

TEST(ScitosDockingPerception, configure) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");

  // Declare debug parameter
  node->declare_parameter("test.perception.enable_debug", rclcpp::ParameterValue(true));

  // Create the perception object
  auto perception = std::make_shared<PerceptionFixture>(node, "test", nullptr);
}

TEST(ScitosDockingPerception, dynamicParameters) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");
  auto perception = std::make_shared<PerceptionFixture>(node, "test", nullptr);

  // Activate the node
  node->configure();
  node->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set the parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.perception.icp_max_iter", 5),
      rclcpp::Parameter("test.perception.icp_min_score", 0.5),
      rclcpp::Parameter("test.perception.icp_max_corr_dis", 0.5),
      rclcpp::Parameter("test.perception.icp_max_trans_eps", 0.5),
      rclcpp::Parameter("test.perception.icp_max_eucl_fit_eps", 0.5),
      rclcpp::Parameter("test.perception.enable_debug", false),
      rclcpp::Parameter("test.perception.use_first_detection", true)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.perception.icp_max_iter").as_int(), 5);
  EXPECT_DOUBLE_EQ(node->get_parameter("test.perception.icp_min_score").as_double(), 0.5);
  EXPECT_DOUBLE_EQ(node->get_parameter("test.perception.icp_max_corr_dis").as_double(), 0.5);
  EXPECT_DOUBLE_EQ(node->get_parameter("test.perception.icp_max_trans_eps").as_double(), 0.5);
  EXPECT_DOUBLE_EQ(node->get_parameter("test.perception.icp_max_eucl_fit_eps").as_double(), 0.5);
  EXPECT_EQ(node->get_parameter("test.perception.enable_debug").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.perception.use_first_detection").as_bool(), true);

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(ScitosDockingPerception, loadDockPointcloud) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");
  auto perception = std::make_shared<PerceptionFixture>(node, "test", nullptr);

  // Try to load an empty file
  std::string filename = "";
  scitos2_charging_dock::Pcloud dock;
  bool success = perception->loadDockPointcloud(filename, dock);
  // Check if the docking station was loaded
  EXPECT_FALSE(success);
  EXPECT_EQ(dock.size(), 0);

  // Try to load another docking station
  std::string pkg = ament_index_cpp::get_package_share_directory("scitos2_charging_dock");
  std::string path = pkg + "/test/empty_dock_test.pcd";
  success = perception->loadDockPointcloud(path, dock);
  // Check if the docking station was loaded
  EXPECT_FALSE(success);
  EXPECT_EQ(dock.size(), 0);

  // Try to load a valid docking station
  path = pkg + "/test/dock_test.pcd";
  success = perception->loadDockPointcloud(path, dock);
  // Check if the docking station was loaded
  EXPECT_TRUE(success);
  EXPECT_EQ(dock.size(), 3);
}

TEST(ScitosDockingPerception, storeDockPointcloud) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");
  auto perception = std::make_shared<PerceptionFixture>(node, "test", nullptr);

  // Store a pointcloud
  std::string pkg = ament_index_cpp::get_package_share_directory("scitos2_charging_dock");
  std::string path = pkg + "/test/empty_dock_test2.pcd";
  scitos2_charging_dock::Pcloud dock;
  dock.push_back(pcl::PointXYZ(0, 0, 0));
  bool success = perception->storeDockPointcloud(path, dock);

  // Check if the docking station was stored
  EXPECT_TRUE(success);
}

TEST(ScitosDockingPerception, segmentsToClusters) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");
  auto perception = std::make_shared<PerceptionFixture>(node, "test", nullptr);

  // Create a segment
  scitos2_charging_dock::Segment segment;
  segment.points.push_back(create_point(0, 0));
  segment.points.push_back(create_point(1, 0));
  segment.points.push_back(create_point(0, 1));
  scitos2_charging_dock::Segments segments;
  segments.push_back(segment);

  // Create another segment
  segment.points.clear();
  segment.points.push_back(create_point(2, 0));
  segment.points.push_back(create_point(3, 0));
  segment.points.push_back(create_point(2, 1));
  segment.points.push_back(create_point(2, 5));
  segments.push_back(segment);

  // Convert the segments to clusters
  auto clusters = perception->segmentsToClusters("test_link", segments);

  // Check if the clusters were created
  EXPECT_EQ(clusters.size(), 2);
  EXPECT_EQ(clusters[0].cloud->size(), 3);
  EXPECT_EQ(clusters[0].cloud->header.frame_id, "test_link");
  EXPECT_EQ(clusters[0].cloud->points[0].x, 0);
  EXPECT_EQ(clusters[0].cloud->points[0].y, 0);
  EXPECT_EQ(clusters[0].cloud->points[1].x, 1);
  EXPECT_EQ(clusters[0].cloud->points[1].y, 0);
  EXPECT_EQ(clusters[0].cloud->points[2].x, 0);
  EXPECT_EQ(clusters[0].cloud->points[2].y, 1);
  EXPECT_EQ(clusters[1].cloud->size(), 4);
  EXPECT_EQ(clusters[1].cloud->points[0].x, 2);
  EXPECT_EQ(clusters[1].cloud->points[0].y, 0);
  EXPECT_EQ(clusters[1].cloud->points[1].x, 3);
  EXPECT_EQ(clusters[1].cloud->points[1].y, 0);
  EXPECT_EQ(clusters[1].cloud->points[2].x, 2);
  EXPECT_EQ(clusters[1].cloud->points[2].y, 1);
  EXPECT_EQ(clusters[1].cloud->points[3].x, 2);
  EXPECT_EQ(clusters[1].cloud->points[3].y, 5);
  EXPECT_EQ(clusters[1].cloud->header.frame_id, "test_link");
}

TEST(ScitosDockingPerception, eigenToTransform) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");
  auto perception = std::make_shared<PerceptionFixture>(node, "test", nullptr);

  // Create a transformation matrix
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0) = 1;
  T(0, 1) = 2;
  T(0, 2) = 3;
  T(0, 3) = 4;
  T(1, 0) = 5;
  T(1, 1) = 6;
  T(1, 2) = 7;
  T(1, 3) = 8;
  T(2, 0) = 9;
  T(2, 1) = 10;
  T(2, 2) = 11;
  T(2, 3) = 12;
  T(3, 0) = 13;
  T(3, 1) = 14;
  T(3, 2) = 15;
  T(3, 3) = 16;

  // Convert the matrix to a transform
  auto transform = perception->eigenToTransform(T);

  // Check if the transform was created
  EXPECT_EQ(transform.getOrigin().getX(), 4);
  EXPECT_EQ(transform.getOrigin().getY(), 8);
  EXPECT_EQ(transform.getOrigin().getZ(), 12);
  EXPECT_EQ(transform.getBasis().getRow(0).getX(), 1);
  EXPECT_EQ(transform.getBasis().getRow(0).getY(), 2);
  EXPECT_EQ(transform.getBasis().getRow(0).getZ(), 3);
  EXPECT_EQ(transform.getBasis().getRow(1).getX(), 5);
  EXPECT_EQ(transform.getBasis().getRow(1).getY(), 6);
  EXPECT_EQ(transform.getBasis().getRow(1).getZ(), 7);
  EXPECT_EQ(transform.getBasis().getRow(2).getX(), 9);
  EXPECT_EQ(transform.getBasis().getRow(2).getY(), 10);
  EXPECT_EQ(transform.getBasis().getRow(2).getZ(), 11);
}

TEST(ScitosDockingPerception, refineClusterToPose) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");
  auto perception = std::make_shared<PerceptionFixture>(node, "test", nullptr);

  // Create a cluster
  scitos2_charging_dock::Cluster cluster;
  cluster.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cluster.cloud->push_back(pcl::PointXYZ(0, 0, 0));
  cluster.cloud->push_back(pcl::PointXYZ(1, 0, 0));
  cluster.cloud->push_back(pcl::PointXYZ(1, 1.5, 0));
  cluster.cloud->push_back(pcl::PointXYZ(1, 2, 0));

  // Create a template
  auto cloud_template = std::make_shared<scitos2_charging_dock::Pcloud>();
  cloud_template->push_back(pcl::PointXYZ(0, 0, 0));
  cloud_template->push_back(pcl::PointXYZ(1, 0, 0));
  cloud_template->push_back(pcl::PointXYZ(1, 1.5, 0));
  cloud_template->push_back(pcl::PointXYZ(1, 2, 0));

  // Set the initial pose
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 1;
  initial_pose.position.y = 1;

  // Refine the cluster pose
  perception->setInitialEstimate(initial_pose, "test_link");
  bool success = perception->refineClusterPose(cluster, cloud_template);

  // Check if the pose was refined
  EXPECT_TRUE(success);
  EXPECT_EQ(cluster.icp_pose.header.frame_id, "test_link");
  EXPECT_NEAR(cluster.icp_pose.pose.position.x, 1.0, 0.01);
  EXPECT_NEAR(cluster.icp_pose.pose.position.y, 1.0, 0.01);

  // Now set another cluster different from the template
  scitos2_charging_dock::Cluster cluster2;
  cluster2.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cluster2.cloud->push_back(pcl::PointXYZ(40, 50, 7));
  cluster2.cloud->push_back(pcl::PointXYZ(10, 30, 0));
  cluster2.cloud->push_back(pcl::PointXYZ(10, 1.5, 0));
  cluster2.cloud->push_back(pcl::PointXYZ(10, 90, 0));

  // Refine the cluster pose
  perception->setInitialEstimate(initial_pose, "test_link");
  success = perception->refineClusterPose(cluster2, cloud_template);

  // Check if the pose was refined
  EXPECT_FALSE(success);
}

TEST(ScitosDockingPerception, refineAllClustersPoses) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");

  // Create the TF
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Set debug mode
  nav2_util::declare_parameter_if_not_declared(
    node, "test.perception.enable_debug", rclcpp::ParameterValue(true));
  node->configure();

  // Create the perception module
  auto perception = std::make_shared<PerceptionFixture>(node, "test", tf_buffer);

  // Create two clusters
  scitos2_charging_dock::Clusters clusters;
  scitos2_charging_dock::Cluster cluster;
  cluster.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cluster.cloud->header.frame_id = "test_link";
  cluster.cloud->push_back(pcl::PointXYZ(0, 0, 0));
  cluster.cloud->push_back(pcl::PointXYZ(1, 0, 0));
  cluster.cloud->push_back(pcl::PointXYZ(1, 1.5, 0));
  cluster.cloud->push_back(pcl::PointXYZ(1, 2, 0));
  clusters.push_back(cluster);

  scitos2_charging_dock::Cluster cluster2;
  cluster2.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cluster2.cloud->header.frame_id = "test_link";
  cluster2.cloud->push_back(pcl::PointXYZ(2, 0, 0));
  cluster2.cloud->push_back(pcl::PointXYZ(3, 0, 0));
  cluster2.cloud->push_back(pcl::PointXYZ(2, 1, 0));
  cluster2.cloud->push_back(pcl::PointXYZ(2, 5, 0));
  clusters.push_back(cluster2);

  // Create a template
  auto cloud_template = std::make_shared<scitos2_charging_dock::Pcloud>();
  cloud_template->header.frame_id = "test_link";
  cloud_template->push_back(pcl::PointXYZ(0, 0, 0));
  cloud_template->push_back(pcl::PointXYZ(1, 0, 0));
  cloud_template->push_back(pcl::PointXYZ(1, 1.5, 0));
  cloud_template->push_back(pcl::PointXYZ(1, 2, 0));

  // Set the initial pose
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 0.0;
  initial_pose.position.y = 0.0;
  perception->setInitialEstimate(initial_pose, "test_link");

  // Refine the clusters poses
  geometry_msgs::msg::PoseStamped dock_pose;
  bool success = perception->refineAllClustersPoses(clusters, cloud_template, dock_pose);

  // Check if the dock is found
  EXPECT_TRUE(success);
  EXPECT_EQ(dock_pose.header.frame_id, "test_link");
  EXPECT_NEAR(dock_pose.pose.position.x, 0.0, 0.01);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
