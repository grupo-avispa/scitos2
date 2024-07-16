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
#include "scitos2_charging_dock/cluster.hpp"

TEST(ScitosDockingCluster, sizeAndClear) {
  scitos2_charging_dock::Cluster cluster;
  cluster.cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cluster.cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

  EXPECT_EQ(cluster.size(), 2);
  EXPECT_EQ(cluster.cloud.points[0].x, 1.0);
  EXPECT_EQ(cluster.cloud.points[0].y, 2.0);
  EXPECT_EQ(cluster.cloud.points[0].z, 3.0);
  EXPECT_EQ(cluster.cloud.points[1].x, 4.0);
  EXPECT_EQ(cluster.cloud.points[1].y, 5.0);
  EXPECT_EQ(cluster.cloud.points[1].z, 6.0);

  cluster.clear();
  EXPECT_EQ(cluster.size(), 0);
}

TEST(ScitosDockingCluster, pushBack) {
  scitos2_charging_dock::Cluster cluster;
  geometry_msgs::msg::Point point;
  point.x = 1.0;
  point.y = 2.0;
  point.z = 3.0;
  cluster.push_back(point);

  point.x = 4.0;
  point.y = 5.0;
  point.z = 6.0;
  cluster.push_back(point);

  EXPECT_EQ(cluster.size(), 2);
  EXPECT_EQ(cluster.cloud.points[0].x, 1.0);
  EXPECT_EQ(cluster.cloud.points[0].y, 2.0);
  EXPECT_EQ(cluster.cloud.points[0].z, 3.0);
  EXPECT_EQ(cluster.cloud.points[1].x, 4.0);
  EXPECT_EQ(cluster.cloud.points[1].y, 5.0);
  EXPECT_EQ(cluster.cloud.points[1].z, 6.0);
}

TEST(ScitosDockingCluster, centroid) {
  scitos2_charging_dock::Cluster cluster;
  cluster.cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cluster.cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

  auto centroid = cluster.centroid();

  EXPECT_DOUBLE_EQ(2.5, centroid.x);
  EXPECT_DOUBLE_EQ(3.5, centroid.y);
  EXPECT_DOUBLE_EQ(4.5, centroid.z);
}

TEST(ScitosDockingCluster, centroidLength) {
  scitos2_charging_dock::Cluster cluster;
  cluster.cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cluster.cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
  EXPECT_DOUBLE_EQ(cluster.centroid_length(), 4.3011626335213133);
}

TEST(ScitosDockingCluster, width) {
  scitos2_charging_dock::Cluster cluster;

  // No cloud
  EXPECT_DOUBLE_EQ(cluster.width(), 0.0);

  cluster.cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cluster.cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
  EXPECT_DOUBLE_EQ(cluster.width(), 4.2426406871192848);
}

TEST(ScitosDockingCluster, widthSquared) {
  scitos2_charging_dock::Cluster cluster;
  cluster.cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cluster.cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
  EXPECT_DOUBLE_EQ(cluster.width_squared(), 18);
}

TEST(ScitosDockingCluster, valid) {
  scitos2_charging_dock::Cluster cluster;

  // No cloud
  EXPECT_FALSE(cluster.valid(0.0));

  cluster.cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cluster.cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
  EXPECT_FALSE(cluster.valid(0.0));
  EXPECT_TRUE(cluster.valid(5.0));
  EXPECT_FALSE(cluster.valid(10.0));
}

TEST(ScitosDockingCluster, operators) {
  scitos2_charging_dock::Cluster cluster1;
  cluster1.score = 1.0;
  scitos2_charging_dock::Cluster cluster2;
  cluster2.score = 2.0;
  EXPECT_TRUE(cluster1 < cluster2);
  EXPECT_FALSE(cluster1 > cluster2);

  cluster1.score = 3.0;
  cluster2.score = 2.0;
  EXPECT_FALSE(cluster1 < cluster2);
  EXPECT_TRUE(cluster1 > cluster2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
