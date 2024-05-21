// Copyright (c) 2017 Alberto J. Tudela Roldán
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

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

#include "scitos2_charging_dock/segmentation.hpp"

namespace scitos2_charging_dock
{

Segmentation::Segmentation(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "segmentation.distance_threshold", rclcpp::ParameterValue(0.04));
  nav2_util::declare_parameter_if_not_declared(
    node, "segmentation.min_points", rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(
    node, "segmentation.max_points", rclcpp::ParameterValue(200));
  nav2_util::declare_parameter_if_not_declared(
    node, "segmentation.min_distance", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "segmentation.max_distance", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "segmentation.min_width", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, "segmentation.max_width", rclcpp::ParameterValue(1.0));

  node->get_parameter("segmentation.distance_threshold", distance_threshold_);
  node->get_parameter("segmentation.min_points", min_points_segment_);
  node->get_parameter("segmentation.max_points", max_points_segment_);
  node->get_parameter("segmentation.min_distance", min_avg_distance_from_sensor_);
  node->get_parameter("segmentation.max_distance", max_avg_distance_from_sensor_);
  node->get_parameter("segmentation.min_width", min_segment_width_);
  node->get_parameter("segmentation.max_width", max_segment_width_);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Segmentation::dynamicParametersCallback, this, std::placeholders::_1));
}

/* This code is a simplification of the segmentation from package laser_segmentation
 * by Alberto Tudela.
 */
Segments Segmentation::performSegmentation(const sensor_msgs::msg::LaserScan & scan)
{
  // Convert the scan to points
  auto points = scanToPoints(scan);

  // Create the first segment
  Segment current_segment;
  current_segment.points.push_back(points.front());

  // Create the segments
  Segments segments;
  for (uint64_t p = 1; p < points.size(); p++) {
    if (isJumpBetweenPoints(points[p - 1], points[p], distance_threshold_)) {
      segments.push_back(current_segment);
      current_segment.points.clear();
    }
    current_segment.points.push_back(points[p]);
  }
  segments.push_back(current_segment);

  return segments;
}

Segments Segmentation::filterSegments(const Segments & segments)
{
  Segments filtered_segments;
  filtered_segments.reserve(segments.size());

  const double squared_min_segment_width = min_segment_width_ * min_segment_width_;
  const double squared_max_segment_width = max_segment_width_ * max_segment_width_;

  for (const auto & segment : segments) {
    // By number of points
    if (segment.size() < min_points_segment_ || segment.size() > max_points_segment_) {
      continue;
    }

    // By distance to sensor
    if (segment.centroid_length() < min_avg_distance_from_sensor_ ||
      segment.centroid_length() > max_avg_distance_from_sensor_)
    {
      continue;
    }

    // By width
    if (segment.width_squared() < squared_min_segment_width ||
      segment.width_squared() > squared_max_segment_width)
    {
      continue;
    }

    filtered_segments.push_back(segment);
  }
  return filtered_segments;
}

geometry_msgs::msg::Point Segmentation::fromPolarToCartesian(double range, double angle)
{
  geometry_msgs::msg::Point point;
  point.x = range * cos(angle);
  point.y = range * sin(angle);
  return point;
}

std::vector<geometry_msgs::msg::Point> Segmentation::scanToPoints(
  const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<geometry_msgs::msg::Point> points;
  double phi = scan.angle_min;
  double angle_resolution = scan.angle_increment;
  for (const auto r : scan.ranges) {
    if (r >= scan.range_min && r <= scan.range_max) {
      points.push_back(fromPolarToCartesian(r, phi));
    }
    phi += angle_resolution;
  }
  return points;
}

double Segmentation::euclideanDistance(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

bool Segmentation::isJumpBetweenPoints(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double threshold)
{
  return euclideanDistance(p1, p2) > threshold;
}

rcl_interfaces::msg::SetParametersResult
Segmentation::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();
    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      if (name == "segmentation.min_points") {
        min_points_segment_ = parameter.as_int();
      } else if (name == "segmentation.max_points") {
        max_points_segment_ = parameter.as_int();
      }
    } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      if (name == "segmentation.distance_threshold") {
        distance_threshold_ = parameter.as_double();
      } else if (name == "segmentation.min_distance") {
        min_avg_distance_from_sensor_ = parameter.as_double();
      } else if (name == "segmentation.max_distance") {
        max_avg_distance_from_sensor_ = parameter.as_double();
      } else if (name == "segmentation.min_width") {
        min_segment_width_ = parameter.as_double();
      } else if (name == "segmentation.max_width") {
        max_segment_width_ = parameter.as_double();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace scitos2_charging_dock
