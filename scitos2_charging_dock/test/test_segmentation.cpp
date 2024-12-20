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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "scitos2_charging_dock/segmentation.hpp"

class SegmentationFixture : public scitos2_charging_dock::Segmentation
{
public:
  explicit SegmentationFixture(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
  : Segmentation(node, name)
  {
  }

  geometry_msgs::msg::Point fromPolarToCartesian(double range, double angle)
  {
    return scitos2_charging_dock::Segmentation::fromPolarToCartesian(range, angle);
  }

  std::vector<geometry_msgs::msg::Point> scanToPoints(const sensor_msgs::msg::LaserScan & scan)
  {
    return scitos2_charging_dock::Segmentation::scanToPoints(scan);
  }

  double euclideanDistance(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
  {
    return scitos2_charging_dock::Segmentation::euclideanDistance(p1, p2);
  }

  bool isJumpBetweenPoints(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double threshold)
  {
    return scitos2_charging_dock::Segmentation::isJumpBetweenPoints(p1, p2, threshold);
  }

  void setDistanceThreshold(double distance_threshold)
  {
    distance_threshold_ = distance_threshold;
  }

  void setFilteringParams(
    int min_points_cluster, int max_points_cluster,
    double min_avg_distance_from_sensor, double max_avg_distance_from_sensor,
    double min_cluster_width, double max_cluster_width)
  {
    min_points_cluster_ = min_points_cluster;
    max_points_cluster_ = max_points_cluster;
    min_avg_distance_from_sensor_ = min_avg_distance_from_sensor;
    max_avg_distance_from_sensor_ = max_avg_distance_from_sensor;
    min_cluster_width_ = min_cluster_width;
    max_cluster_width_ = max_cluster_width;
  }

  double getMinPointsSegment() {return min_points_cluster_;}
  double getMaxPointsSegment() {return max_points_cluster_;}
  double getMinAvgDistanceFromSensor() {return min_avg_distance_from_sensor_;}
  double getMaxAvgDistanceFromSensor() {return max_avg_distance_from_sensor_;}
  double getMinSegmentWidth() {return min_cluster_width_;}
  double getMaxSegmentWidth() {return max_cluster_width_;}
};

geometry_msgs::msg::Point create_point(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

// Create a vector of points with a distance between them
std::vector<geometry_msgs::msg::Point> create_points(int num_points, double distance)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (int i = 0; i < num_points; i++) {
    geometry_msgs::msg::Point point;
    point.x = i * distance;
    point.y = i * distance;
    points.push_back(point);
  }
  return points;
}

// Create a laser scan with a distance between the points
sensor_msgs::msg::LaserScan create_scan(int num_points, double distance)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = 2 * M_PI / distance;
  for (int i = 0; i < num_points; i++) {
    double range = std::hypot(i * distance, i * distance);
    scan.ranges.push_back(range);
  }
  scan.range_min = scan.ranges.front();
  scan.range_max = scan.ranges.back();
  return scan;
}

TEST(SegmentationTest, dynamicParameters) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Activate the node
  node->configure();
  node->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set the parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.segmentation.min_points", 1),
      rclcpp::Parameter("test.segmentation.max_points", 3),
      rclcpp::Parameter("test.segmentation.distance_threshold", 1.0),
      rclcpp::Parameter("test.segmentation.min_distance", 10.0),
      rclcpp::Parameter("test.segmentation.max_distance", 0.1),
      rclcpp::Parameter("test.segmentation.min_width", 10.0),
      rclcpp::Parameter("test.segmentation.max_width", 0.1)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.segmentation.min_points").as_int(), 1);
  EXPECT_EQ(node->get_parameter("test.segmentation.max_points").as_int(), 3);
  EXPECT_EQ(node->get_parameter("test.segmentation.distance_threshold").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.segmentation.min_distance").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.segmentation.max_distance").as_double(), 0.1);
  EXPECT_EQ(node->get_parameter("test.segmentation.min_width").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.segmentation.max_width").as_double(), 0.1);

  // Cleaning up
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

TEST(SegmentationTest, euclideanDistance) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Calculate the euclidean distance between two points
  geometry_msgs::msg::Point point1;
  point1.x = 1.0;
  point1.y = 1.0;
  geometry_msgs::msg::Point point2;
  point2.x = 3.0;
  point2.y = 3.0;
  double distance = segmentation->euclideanDistance(point1, point2);
  // Check the distance
  EXPECT_DOUBLE_EQ(distance, 2.8284271247461903);
}

TEST(SegmentationTest, fromPolarToCartesian) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Convert a polar point to a cartesian point
  geometry_msgs::msg::Point point = segmentation->fromPolarToCartesian(1.0, 0.0);
  // Check the point
  EXPECT_DOUBLE_EQ(point.x, 1.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);

  // Convert a polar point to a cartesian point
  point = segmentation->fromPolarToCartesian(1.0, M_PI / 2);
  // Check the point
  EXPECT_NEAR(point.x, 0.0, 1e-3);
  EXPECT_DOUBLE_EQ(point.y, 1.0);
}

TEST(SegmentationTest, scanToPoints) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Convert a laser scan to a vector of points
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = 2 * M_PI / 5;
  scan.ranges = {1.0, 2.0, 3.0, 4.0, 5.0};
  scan.range_min = scan.ranges.front();
  scan.range_max = scan.ranges.back();
  scan.ranges.push_back(0.0);
  scan.ranges.push_back(7.0);
  auto points = segmentation->scanToPoints(scan);
  // Check the number of points as the last two points are out of range
  EXPECT_EQ(points.size(), 5);
  // Check the points
  double phi = scan.angle_min;
  for (int i = 0; i < 5; i++) {
    EXPECT_NEAR(points[i].x, scan.ranges[i] * cos(phi), 1e-6);
    EXPECT_NEAR(points[i].y, scan.ranges[i] * sin(phi), 1e-6);
    phi += scan.angle_increment;
  }
}

TEST(SegmentationTest, isJumpBetweenPoints) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Create two points with a distance of 1.0
  geometry_msgs::msg::Point point1;
  point1.x = 0.0;
  point1.y = 0.0;
  geometry_msgs::msg::Point point2;
  point2.x = 1.0;
  point2.y = 1.0;

  // Check if there is a jump between two points
  EXPECT_TRUE(segmentation->isJumpBetweenPoints(point1, point2, 0.1));
  EXPECT_FALSE(segmentation->isJumpBetweenPoints(point1, point2, 2.0));
}

TEST(SegmentationTest, performSegmentation) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Create an empty scan
  sensor_msgs::msg::LaserScan scan;
  scitos2_charging_dock::Clusters clusters;
  EXPECT_FALSE(segmentation->performSegmentation(scan, clusters));

  // Set the jump distance to 0.1
  segmentation->setDistanceThreshold(0.1);
  // Create a scan with points with a distance of 0.05
  scan = create_scan(3, 0.05);
  // Perform the segmentation
  EXPECT_TRUE(segmentation->performSegmentation(scan, clusters));
  // Check the number of clusters
  // Should be 1 cluster as the jump distance is 0.1
  // and the distance between points is 0.05
  EXPECT_EQ(clusters.size(), 1);
  // Check the number of points in the cluster
  // Should be 3 points in the cluster
  EXPECT_EQ(clusters.front().size(), 3);
  // Check the points in the cluster
  double phi = scan.angle_min;
  for (int i = 0; i < clusters.front().size(); i++) {
    auto point = clusters.front().cloud.points[i];
    EXPECT_NEAR(point.x, scan.ranges[i] * cos(phi), 1e-6);
    EXPECT_NEAR(point.y, scan.ranges[i] * sin(phi), 1e-6);
    phi += scan.angle_increment;
  }

  // Now we set the points with a distance of 1.0
  scan = create_scan(3, 1.0);
  // Perform the segmentation
  clusters.clear();
  EXPECT_TRUE(segmentation->performSegmentation(scan, clusters));
  // Check the number of clusters
  // Should be 1 cluster per point as the jump distance is 0.1
  // and the distance between points is 1.0
  EXPECT_EQ(clusters.size(), 3);
  // Check the number of points in the cluster
  for (const auto & cluster : clusters) {
    EXPECT_EQ(cluster.size(), 1);
  }
  // Check the points in the cluster
  phi = scan.angle_min;
  for (size_t i = 0; i < clusters.size(); i++) {
    auto point = clusters[i].cloud.points[0];
    EXPECT_NEAR(point.x, scan.ranges[i] * cos(phi), 1e-6);
    EXPECT_NEAR(point.y, scan.ranges[i] * sin(phi), 1e-6);
    phi += scan.angle_increment;
  }
}

TEST(SegmentationTest, filteringClusters) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Set the filtering parameters
  segmentation->setFilteringParams(1, 3, 1.0, 10.0, 0.1, 10.0);

  // Set a cluster list with 0 clusters
  std::vector<scitos2_charging_dock::Cluster> cluster_list;
  auto filtered_clusters = segmentation->filterClusters(cluster_list);
  // Check the filtered clusters
  EXPECT_EQ(filtered_clusters.size(), 0);

  // Set a cluster list with 4 clusters of 1 point
  cluster_list.clear();
  scitos2_charging_dock::Cluster cluster;
  cluster.push_back(create_point(0.0, 0.0));
  cluster_list.push_back(cluster);
  cluster_list.push_back(cluster);
  cluster_list.push_back(cluster);
  cluster_list.push_back(cluster);
  // Filter the clusters
  filtered_clusters = segmentation->filterClusters(cluster_list);
  // Check the filtered clusters
  EXPECT_EQ(filtered_clusters.size(), 0);

  // Set a cluster list with 2 clusters with centroid below the minimum distance
  cluster_list.clear();
  cluster.clear();
  cluster.push_back(create_point(0.0, 0.0));
  cluster_list.push_back(cluster);
  cluster.clear();
  cluster.push_back(create_point(0.5, 0.5));
  cluster_list.push_back(cluster);
  // Filter the clusters
  filtered_clusters = segmentation->filterClusters(cluster_list);
  // Check the filtered clusters
  EXPECT_EQ(filtered_clusters.size(), 0);

  // Set a cluster list with 2 clusters with centroid above the maximum distance
  cluster_list.clear();
  cluster.clear();
  cluster.push_back(create_point(11.0, 11.0));
  cluster_list.push_back(cluster);
  cluster.clear();
  cluster.push_back(create_point(12.0, 12.0));
  cluster_list.push_back(cluster);
  // Filter the clusters
  filtered_clusters = segmentation->filterClusters(cluster_list);
  // Check the filtered clusters
  EXPECT_EQ(filtered_clusters.size(), 0);

  // Set a cluster list with 2 clusters with width below the minimum width
  cluster_list.clear();
  cluster.clear();
  cluster.push_back(create_point(0.0, 0.0));
  cluster.push_back(create_point(0.0, 0.0));
  cluster_list.push_back(cluster);
  cluster.clear();
  cluster.push_back(create_point(1.0, 1.0));
  cluster.push_back(create_point(1.0, 1.0));
  cluster_list.push_back(cluster);
  // Filter the clusters
  filtered_clusters = segmentation->filterClusters(cluster_list);
  // Check the filtered clusters
  EXPECT_EQ(filtered_clusters.size(), 0);

  // Set a cluster list with 2 clusters with width above the maximum width
  cluster_list.clear();
  cluster.clear();
  cluster.push_back(create_point(0.0, 0.0));
  cluster.push_back(create_point(10.0, 10.0));
  cluster_list.push_back(cluster);
  cluster.clear();
  cluster.push_back(create_point(0.0, 0.0));
  cluster.push_back(create_point(15.0, 15.0));
  cluster_list.push_back(cluster);
  // Filter the clusters
  filtered_clusters = segmentation->filterClusters(cluster_list);
  // Check the filtered clusters
  EXPECT_EQ(filtered_clusters.size(), 0);

  // Set a cluster list with several clusters
  cluster_list.clear();
  cluster.clear();
  cluster.push_back(create_point(0.0, 0.0));
  cluster_list.push_back(cluster);
  cluster.clear();
  cluster.push_back(create_point(0.0, 0.0));
  cluster.push_back(create_point(1.0, 1.0));
  cluster_list.push_back(cluster);
  cluster.clear();
  cluster.push_back(create_point(2.0, 2.0));
  cluster.push_back(create_point(3.0, 3.0));
  cluster.push_back(create_point(4.0, 4.0));
  cluster_list.push_back(cluster);
  cluster.clear();
  cluster.push_back(create_point(5.0, 5.0));
  cluster.push_back(create_point(6.0, 6.0));
  cluster.push_back(create_point(7.0, 7.0));
  cluster.push_back(create_point(8.0, 8.0));
  cluster_list.push_back(cluster);
  cluster.clear();

  EXPECT_EQ(cluster_list.size(), 4);
  // Filter the clusters
  filtered_clusters = segmentation->filterClusters(cluster_list);
  // Check the filtered clusters
  EXPECT_EQ(filtered_clusters.size(), 1);
  EXPECT_EQ(filtered_clusters[0].centroid().x, 3.0);
  EXPECT_EQ(filtered_clusters[0].centroid().y, 3.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
