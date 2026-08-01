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

#ifndef SCITOS2_CHARGING_DOCK__DOCK_SELECTOR_HPP_
#define SCITOS2_CHARGING_DOCK__DOCK_SELECTOR_HPP_

#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "scitos2_charging_dock/cluster.hpp"

namespace scitos2_charging_dock
{

/**
 * @brief Thresholds used to decide whether an ICP-refined candidate is the dock.
 */
struct DockSelectionParams
{
  // Candidates with an ICP fitness score at or above this are rejected
  double icp_min_score;
  // Candidates whose refined yaw differs from the initial estimate by more than this
  // (rad) are rejected: the dock is asymmetric, so a match far from the initial estimate
  // is a mismatch, not a correction
  double max_yaw_error;
};

/**
 * @brief Select the best dock candidate among a set of ICP-refined clusters.
 *
 * Pure decision logic: no ROS node, no TF, no publishers involved, so it can be unit
 * tested directly against plain Cluster/PoseStamped values. Perception is responsible for
 * segmenting the scan and running ICP to produce the candidates this function chooses from.
 *
 * @param candidates Clusters already refined by ICP (valid score and pose)
 * @param initial_estimate Initial pose estimate provided by the docking server
 * @param params Selection thresholds
 * @return std::optional<Cluster> The selected dock, or std::nullopt if none qualifies
 */
std::optional<Cluster> selectDock(
  const Clusters & candidates,
  const geometry_msgs::msg::PoseStamped & initial_estimate,
  const DockSelectionParams & params);

}  // namespace scitos2_charging_dock

#endif  // SCITOS2_CHARGING_DOCK__DOCK_SELECTOR_HPP_
