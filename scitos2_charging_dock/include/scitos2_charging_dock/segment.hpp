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

#ifndef SCITOS2_CHARGING_DOCK__SEGMENT_HPP_
#define SCITOS2_CHARGING_DOCK__SEGMENT_HPP_

// C++
#include <cmath>
#include <vector>

// ROS
#include "geometry_msgs/msg/point.hpp"

namespace scitos2_charging_dock
{

/**
 * @struct scitos2_charging_dock::Segment
 * @brief Struct to store a segment of points.
 */
struct Segment
{
  /**
   * @brief Get the size of the segment.
   * @return int The size of the segment.
   */
  int size() const {return points.size();}

  /**
   * @brief Clear the segment.
   */
  void clear() {points.clear();}

  /**
   * @brief Get the width squared of the segment.
   * @return double The width squared of the segment.
   */
  double width_squared() const
  {
    double dx = points.back().x - points.front().x;
    double dy = points.back().y - points.front().y;
    return dx * dx + dy * dy;
  }

  /**
   * @brief Get the centroid of the segment.
   *
   * @return geometry_msgs::msg::Point The centroid of the segment.
   */
  geometry_msgs::msg::Point centroid() const
  {
    geometry_msgs::msg::Point centroid;
    for (const auto & point : points) {
      centroid.x += point.x;
      centroid.y += point.y;
      centroid.z += point.z;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    centroid.z /= points.size();
    return centroid;
  }

  /**
   * @brief Get the length of the centroid.
   *
   * @return double The length of the centroid.
   */
  double centroid_length() const
  {
    auto c = centroid();
    return sqrt(c.x * c.x + c.y * c.y);
  }

  std::vector<geometry_msgs::msg::Point> points;
};

}  // namespace scitos2_charging_dock

#endif  // SCITOS2_CHARGING_DOCK__SEGMENT_HPP_
