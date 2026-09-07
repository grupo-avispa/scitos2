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

// C++
#include <algorithm>
#include <cmath>

// ROS
#include "angles/angles.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "scitos2_charging_dock/dock_selector.hpp"

namespace scitos2_charging_dock
{

std::optional<Cluster> selectDock(
  const Clusters & candidates,
  const geometry_msgs::msg::PoseStamped & initial_estimate,
  const DockSelectionParams & params)
{
  const double initial_yaw = tf2::getYaw(initial_estimate.pose.orientation);

  Clusters potential_docks;
  for (const auto & candidate : candidates) {
    // The dock is asymmetric: a refined pose whose orientation is far from the initial
    // estimate is a mismatch (e.g. ICP settling ~180 deg flipped), not a valid correction
    const double refined_yaw = tf2::getYaw(candidate.pose.pose.orientation);
    const double yaw_error = angles::shortest_angular_distance(initial_yaw, refined_yaw);
    if (std::abs(yaw_error) > params.max_yaw_error) {
      continue;
    }

    if (candidate.score < params.icp_min_score) {
      potential_docks.push_back(candidate);
    }
  }

  if (potential_docks.empty()) {
    return std::nullopt;
  }

  // Select the candidate with the best (lowest) ICP score
  return *std::min_element(potential_docks.begin(), potential_docks.end());
}

}  // namespace scitos2_charging_dock
