// Copyright (c) 2024 Open Navigation LLC
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
    int min_points_segment, int max_points_segment,
    double min_avg_distance_from_sensor, double max_avg_distance_from_sensor,
    double min_segment_width, double max_segment_width)
  {
    min_points_segment_ = min_points_segment;
    max_points_segment_ = max_points_segment;
    min_avg_distance_from_sensor_ = min_avg_distance_from_sensor;
    max_avg_distance_from_sensor_ = max_avg_distance_from_sensor;
    min_segment_width_ = min_segment_width;
    max_segment_width_ = max_segment_width;
  }

  double getMinPointsSegment() {return min_points_segment_;}
  double getMaxPointsSegment() {return max_points_segment_;}
  double getMinAvgDistanceFromSensor() {return min_avg_distance_from_sensor_;}
  double getMaxAvgDistanceFromSensor() {return max_avg_distance_from_sensor_;}
  double getMinSegmentWidth() {return min_segment_width_;}
  double getMaxSegmentWidth() {return max_segment_width_;}
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
    EXPECT_DOUBLE_EQ(points[i].x, scan.ranges[i] * cos(phi));
    EXPECT_DOUBLE_EQ(points[i].y, scan.ranges[i] * sin(phi));
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
  bool jump = segmentation->isJumpBetweenPoints(point1, point2, 0.1);
  // Check the result
  EXPECT_TRUE(jump);

  // Check if there is a jump between two points
  jump = segmentation->isJumpBetweenPoints(point1, point2, 2.0);
  // Check the result
  EXPECT_FALSE(jump);
}

TEST(SegmentationTest, performSegmentation) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Set the jump distance to 0.1
  segmentation->setDistanceThreshold(0.1);

  // Set points with a distance of 0.05
  auto scan = create_scan(3, 0.05);
  // Perform the segmentation
  auto segments = segmentation->performSegmentation(scan);
  // Check the number of segments
  // Should be 1 segment as the jump distance is 0.1
  // and the distance between points is 0.05
  EXPECT_EQ(segments.size(), 1);
  // Check the number of points in the segment
  // Should be 3 points in the segment
  EXPECT_EQ(segments.front().size(), 3);

  // Now we set the points with a distance of 1.0
  scan = create_scan(3, 1.0);
  // Perform the segmentation
  segments = segmentation->performSegmentation(scan);
  // Check the number of segments
  // Should be 1 segment per point as the jump distance is 0.1
  // and the distance between points is 1.0
  EXPECT_EQ(segments.size(), 3);
  // Check the number of points in the segment
  for (auto segment : segments) {
    EXPECT_EQ(segment.size(), 1);
  }
}

TEST(SegmentationTest, filteringSegments) {
  // Create a node
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("segmentation_test");
  auto segmentation = std::make_shared<SegmentationFixture>(node, "test");

  // Set the filtering parameters
  segmentation->setFilteringParams(1, 3, 1.0, 10.0, 0.1, 10.0);

  // Set a segment list with 0 segments
  std::vector<scitos2_charging_dock::Segment> segment_list;
  auto filtered_segments = segmentation->filterSegments(segment_list);
  // Check the filtered segments
  EXPECT_EQ(filtered_segments.size(), 0);

  // Set a segment list with 4 segments of 1 point
  segment_list.clear();
  scitos2_charging_dock::Segment segment;
  segment.points.push_back(create_point(0.0, 0.0));
  segment_list.push_back(segment);
  segment_list.push_back(segment);
  segment_list.push_back(segment);
  segment_list.push_back(segment);
  // Filter the segments
  filtered_segments = segmentation->filterSegments(segment_list);
  // Check the filtered segments
  EXPECT_EQ(filtered_segments.size(), 0);

  // Set a segment list with 2 segments with centroid below the minimum distance
  segment_list.clear();
  segment.clear();
  segment.points.push_back(create_point(0.0, 0.0));
  segment_list.push_back(segment);
  segment.clear();
  segment.points.push_back(create_point(0.5, 0.5));
  segment_list.push_back(segment);
  // Filter the segments
  filtered_segments = segmentation->filterSegments(segment_list);
  // Check the filtered segments
  EXPECT_EQ(filtered_segments.size(), 0);

  // Set a segment list with 2 segments with centroid above the maximum distance
  segment_list.clear();
  segment.clear();
  segment.points.push_back(create_point(11.0, 11.0));
  segment_list.push_back(segment);
  segment.clear();
  segment.points.push_back(create_point(12.0, 12.0));
  segment_list.push_back(segment);
  // Filter the segments
  filtered_segments = segmentation->filterSegments(segment_list);
  // Check the filtered segments
  EXPECT_EQ(filtered_segments.size(), 0);

  // Set a segment list with 2 segments with width below the minimum width
  segment_list.clear();
  segment.clear();
  segment.points.push_back(create_point(0.0, 0.0));
  segment.points.push_back(create_point(0.0, 0.0));
  segment_list.push_back(segment);
  segment.clear();
  segment.points.push_back(create_point(1.0, 1.0));
  segment.points.push_back(create_point(1.0, 1.0));
  segment_list.push_back(segment);
  // Filter the segments
  filtered_segments = segmentation->filterSegments(segment_list);
  // Check the filtered segments
  EXPECT_EQ(filtered_segments.size(), 0);

  // Set a segment list with 2 segments with width above the maximum width
  segment_list.clear();
  segment.clear();
  segment.points.push_back(create_point(0.0, 0.0));
  segment.points.push_back(create_point(10.0, 10.0));
  segment_list.push_back(segment);
  segment.clear();
  segment.points.push_back(create_point(0.0, 0.0));
  segment.points.push_back(create_point(15.0, 15.0));
  segment_list.push_back(segment);
  // Filter the segments
  filtered_segments = segmentation->filterSegments(segment_list);
  // Check the filtered segments
  EXPECT_EQ(filtered_segments.size(), 0);

  // Set a segment list with several segments
  segment_list.clear();
  segment.clear();
  segment.points.push_back(create_point(0.0, 0.0));
  segment_list.push_back(segment);
  segment.points.push_back(create_point(0.0, 0.0));
  segment.points.push_back(create_point(1.0, 1.0));
  segment_list.push_back(segment);
  segment.clear();
  segment.points.push_back(create_point(2.0, 2.0));
  segment.points.push_back(create_point(3.0, 3.0));
  segment.points.push_back(create_point(4.0, 4.0));
  segment_list.push_back(segment);
  segment.clear();
  segment.points.push_back(create_point(5.0, 5.0));
  segment.points.push_back(create_point(6.0, 6.0));
  segment.points.push_back(create_point(7.0, 7.0));
  segment.points.push_back(create_point(8.0, 8.0));
  segment_list.push_back(segment);
  // Filter the segments
  filtered_segments = segmentation->filterSegments(segment_list);
  // Check the filtered segments
  EXPECT_EQ(filtered_segments.size(), 1);
  EXPECT_EQ(filtered_segments[0].centroid().x, 3.0);
  EXPECT_EQ(filtered_segments[0].centroid().y, 3.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
