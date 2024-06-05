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

TEST(ScitosDockingCluster, getCentroid) {
  scitos2_charging_dock::Cluster cluster;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cloud->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
  cluster.cloud = cloud;

  auto centroid = cluster.getCentroid();

  EXPECT_DOUBLE_EQ(2.5, centroid.x);
  EXPECT_DOUBLE_EQ(3.5, centroid.y);
  EXPECT_DOUBLE_EQ(4.5, centroid.z);
}

TEST(ScitosDockingCluster, width) {
  scitos2_charging_dock::Cluster cluster;

  // No cloud
  EXPECT_DOUBLE_EQ(cluster.width(), 0.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cloud->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
  cluster.cloud = cloud;
  EXPECT_DOUBLE_EQ(cluster.width(), 4.2426406871192848);
}

TEST(ScitosDockingCluster, valid) {
  scitos2_charging_dock::Cluster cluster;

  // No cloud
  EXPECT_FALSE(cluster.valid(0.0));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  cloud->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
  cluster.cloud = cloud;

  EXPECT_FALSE(cluster.valid(0.0));
  EXPECT_TRUE(cluster.valid(5.0));
  EXPECT_FALSE(cluster.valid(10.0));
}

TEST(ScitosDockingCluster, operators) {
  scitos2_charging_dock::Cluster cluster1;
  cluster1.icp_score = 1.0;
  scitos2_charging_dock::Cluster cluster2;
  cluster2.icp_score = 2.0;
  EXPECT_TRUE(cluster1 < cluster2);
  EXPECT_FALSE(cluster1 > cluster2);

  cluster1.icp_score = 3.0;
  cluster2.icp_score = 2.0;
  EXPECT_FALSE(cluster1 < cluster2);
  EXPECT_TRUE(cluster1 > cluster2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
