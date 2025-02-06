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
#include "ament_index_cpp/get_package_share_directory.hpp"
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

  sensor_msgs::msg::PointCloud2 createPointCloud2Msg(const scitos2_charging_dock::Pcloud & cloud)
  {
    return scitos2_charging_dock::Perception::createPointCloud2Msg(cloud);
  }

  tf2::Transform eigenToTransform(const Eigen::Matrix4f & T)
  {
    return scitos2_charging_dock::Perception::eigenToTransform(T);
  }

  bool refineClusterPose(
    scitos2_charging_dock::Cluster & cluster, const scitos2_charging_dock::Pcloud & cloud_template)
  {
    return scitos2_charging_dock::Perception::refineClusterPose(cluster, cloud_template);
  }

  bool refineAllClustersPoses(
    scitos2_charging_dock::Clusters & clusters,
    const scitos2_charging_dock::Cluster dock_template,
    scitos2_charging_dock::Cluster & dock)
  {
    return scitos2_charging_dock::Perception::refineAllClustersPoses(clusters, dock_template, dock);
  }

  bool getDockFound() {return dock_found_;}

  void setUseFirstDetection(bool use_first_detection)
  {
    use_first_detection_ = use_first_detection;
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

TEST(ScitosDockingPerception, createPointcloudMsg) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");
  auto perception = std::make_shared<PerceptionFixture>(node, "test", nullptr);

  // Create a pointcloud
  scitos2_charging_dock::Pcloud cloud;
  cloud.header.frame_id = "test_link";
  cloud.push_back(pcl::PointXYZ(0, 0, 0));
  cloud.push_back(pcl::PointXYZ(1, 0, 0));
  cloud.push_back(pcl::PointXYZ(1, 1.5, 0));
  cloud.push_back(pcl::PointXYZ(1, 2, 0));

  // Create a PointCloud2 message
  auto msg = perception->createPointCloud2Msg(cloud);

  // Check if the message was created
  EXPECT_EQ(msg.header.frame_id, "test_link");
  EXPECT_EQ(msg.width, 4);
  EXPECT_EQ(msg.height, 1);
  EXPECT_EQ(msg.fields.size(), 3);
  EXPECT_EQ(msg.fields[0].name, "x");
  EXPECT_EQ(msg.fields[1].name, "y");
  EXPECT_EQ(msg.fields[2].name, "z");
  EXPECT_EQ(msg.point_step, 16);
  EXPECT_EQ(msg.row_step, 64);
  EXPECT_EQ(msg.is_dense, true);
  EXPECT_EQ(msg.is_bigendian, false);
  EXPECT_EQ(msg.data.size(), 64);
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
  cluster.cloud.push_back(pcl::PointXYZ(0, 0, 0));
  cluster.cloud.push_back(pcl::PointXYZ(1, 0, 0));
  cluster.cloud.push_back(pcl::PointXYZ(1, 1.5, 0));
  cluster.cloud.push_back(pcl::PointXYZ(1, 2, 0));

  // Create a template
  scitos2_charging_dock::Pcloud cloud_template;
  cloud_template.push_back(pcl::PointXYZ(0, 0, 0));
  cloud_template.push_back(pcl::PointXYZ(1, 0, 0));
  cloud_template.push_back(pcl::PointXYZ(1, 1.5, 0));
  cloud_template.push_back(pcl::PointXYZ(1, 2, 0));

  // Set the initial pose
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 1;
  initial_pose.position.y = 1;

  // Refine the cluster pose
  perception->setInitialEstimate(initial_pose, "test_link");
  bool success = perception->refineClusterPose(cluster, cloud_template);

  // Check if the pose was refined
  EXPECT_TRUE(success);
  EXPECT_EQ(cluster.pose.header.frame_id, "test_link");
  EXPECT_NEAR(cluster.pose.pose.position.x, 1.0, 0.01);
  EXPECT_NEAR(cluster.pose.pose.position.y, 1.0, 0.01);

  // Now set another cluster different from the template
  cluster.clear();
  cluster.cloud.push_back(pcl::PointXYZ(40, 50, 7));
  cluster.cloud.push_back(pcl::PointXYZ(10, 30, 0));
  cluster.cloud.push_back(pcl::PointXYZ(10, 1.5, 0));
  cluster.cloud.push_back(pcl::PointXYZ(10, 90, 0));

  // Refine the cluster pose
  perception->setInitialEstimate(initial_pose, "test_link");
  success = perception->refineClusterPose(cluster, cloud_template);

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
  cluster.cloud.header.frame_id = "test_link";
  cluster.cloud.push_back(pcl::PointXYZ(0, 0, 0));
  cluster.cloud.push_back(pcl::PointXYZ(1, 0, 0));
  cluster.cloud.push_back(pcl::PointXYZ(1, 1.5, 0));
  cluster.cloud.push_back(pcl::PointXYZ(1, 2, 0));
  clusters.push_back(cluster);
  cluster.clear();

  cluster.cloud.push_back(pcl::PointXYZ(2, 0, 0));
  cluster.cloud.push_back(pcl::PointXYZ(3, 0, 0));
  cluster.cloud.push_back(pcl::PointXYZ(2, 1, 0));
  cluster.cloud.push_back(pcl::PointXYZ(2, 5, 0));
  clusters.push_back(cluster);
  cluster.clear();

  // Create a template
  scitos2_charging_dock::Cluster dock_template;
  dock_template.cloud.header.frame_id = "test_link";
  dock_template.cloud.push_back(pcl::PointXYZ(0, 0, 0));
  dock_template.cloud.push_back(pcl::PointXYZ(1, 0, 0));
  dock_template.cloud.push_back(pcl::PointXYZ(1, 1.5, 0));
  dock_template.cloud.push_back(pcl::PointXYZ(1, 2, 0));

  // Set the initial pose
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 0.0;
  initial_pose.position.y = 0.0;
  perception->setInitialEstimate(initial_pose, "test_link");

  // Refine the clusters poses
  scitos2_charging_dock::Cluster dock;
  bool success = perception->refineAllClustersPoses(clusters, dock_template, dock);

  // Check if the dock is found
  EXPECT_TRUE(success);
  EXPECT_EQ(dock.pose.header.frame_id, "test_link");
  EXPECT_NEAR(dock.pose.pose.position.x, 0.0, 0.01);
}

TEST(ScitosDockingPerception, getDockPose) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_test");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Set the dock template
  std::string pkg = ament_index_cpp::get_package_share_directory("scitos2_charging_dock");
  std::string path = pkg + "/test/dock_test.pcd";
  nav2_util::declare_parameter_if_not_declared(
    node, "test.perception.dock_template", rclcpp::ParameterValue(path));
  nav2_util::declare_parameter_if_not_declared(
    node, "test.segmentation.distance_threshold", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "test.segmentation.min_points", rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node, "test.segmentation.min_width", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "test.segmentation.min_distance", rclcpp::ParameterValue(0.0));
  node->configure();

  // Create the perception module
  auto perception = std::make_shared<PerceptionFixture>(node, "test", tf_buffer);

  // Set the initial pose
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 0.0;
  initial_pose.position.y = 0.0;
  perception->setInitialEstimate(initial_pose, "test_link");

  // Create two scans
  sensor_msgs::msg::LaserScan scan_template;
  scan_template.header.stamp = node->now();
  scan_template.header.frame_id = "test_link";
  // This three points are a match with the test dock template
  scan_template.angle_min = -std::atan2(0.1, 0.9);
  scan_template.angle_max = std::atan2(0.1, 0.9);
  scan_template.angle_increment = std::atan2(0.1, 0.9);
  scan_template.ranges = {0.9055, 1.0, 0.9055};
  scan_template.range_min = 0.9055;
  scan_template.range_max = 1.0;

  sensor_msgs::msg::LaserScan scan_random;
  scan_random = scan_template;
  scan_random.ranges = {5.2, 2.0, 2.25, 4.5, 8.2};

  // The dock is not found yet
  EXPECT_FALSE(perception->getDockFound());

  // Try to get the dock pose with a random scan
  // and the use_first_detection parameter set to true
  perception->setUseFirstDetection(true);
  auto dock_pose = perception->getDockPose(scan_random);
  EXPECT_FALSE(perception->getDockFound());

  // Try to get the dock pose with a scan that matches the template
  // and the use_first_detection parameter set to true
  perception->setUseFirstDetection(false);
  dock_pose = perception->getDockPose(scan_template);
  EXPECT_TRUE(perception->getDockFound());
  EXPECT_EQ(dock_pose.header.frame_id, "test_link");
  EXPECT_NEAR(dock_pose.pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(dock_pose.pose.position.y, 0.0, 0.01);

  // Now set the use_first_detection parameter to true with a dock found
  perception->setUseFirstDetection(true);
  dock_pose = perception->getDockPose(scan_random);
  EXPECT_TRUE(perception->getDockFound());

  // Now set the use_first_detection parameter to false with a dock found
  perception->setUseFirstDetection(false);
  dock_pose = perception->getDockPose(scan_template);
  EXPECT_TRUE(perception->getDockFound());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
