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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "scitos2_charging_dock/dock_selector.hpp"

using scitos2_charging_dock::Cluster;
using scitos2_charging_dock::Clusters;
using scitos2_charging_dock::DockSelectionParams;
using scitos2_charging_dock::selectDock;

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

TEST(DockSelectorTest, noCandidatesReturnsNullopt) {
  Clusters candidates;
  DockSelectionParams params{0.01, M_PI_2};

  auto result = selectDock(candidates, makePose(0, 0, 0), params);

  EXPECT_FALSE(result.has_value());
}

TEST(DockSelectorTest, picksTheLowestScoreAmongValidCandidates) {
  Clusters candidates;

  Cluster best;
  best.id = 1;
  best.score = 0.005;
  best.pose = makePose(0, 0, 0.05);
  candidates.push_back(best);

  Cluster worse;
  worse.id = 2;
  worse.score = 0.008;
  worse.pose = makePose(0, 0, 0.02);
  candidates.push_back(worse);

  DockSelectionParams params{0.01, M_PI_2};
  auto result = selectDock(candidates, makePose(0, 0, 0.0), params);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, 1);
}

TEST(DockSelectorTest, rejectsScoreAtOrAboveThreshold) {
  Clusters candidates;
  Cluster bad;
  bad.score = 0.5;
  bad.pose = makePose(0, 0, 0);
  candidates.push_back(bad);

  DockSelectionParams params{0.01, M_PI_2};
  auto result = selectDock(candidates, makePose(0, 0, 0), params);

  EXPECT_FALSE(result.has_value());
}

TEST(DockSelectorTest, rejectsCandidateFlippedFromInitialEstimate) {
  // The dock is asymmetric: a candidate refined ~180 deg away from the initial estimate is
  // a mismatch, not a valid correction, even with an excellent ICP score
  Clusters candidates;
  Cluster flipped;
  flipped.score = 0.001;
  flipped.pose = makePose(0, 0, M_PI);
  candidates.push_back(flipped);

  DockSelectionParams params{0.01, M_PI_2};
  auto result = selectDock(candidates, makePose(0, 0, 0.0), params);

  EXPECT_FALSE(result.has_value());
}

TEST(DockSelectorTest, acceptsCandidateWithinYawTolerance) {
  Clusters candidates;
  Cluster candidate;
  candidate.id = 7;
  candidate.score = 0.001;
  candidate.pose = makePose(0, 0, M_PI_4);
  candidates.push_back(candidate);

  DockSelectionParams params{0.01, M_PI_2};
  auto result = selectDock(candidates, makePose(0, 0, 0.0), params);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, 7);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
