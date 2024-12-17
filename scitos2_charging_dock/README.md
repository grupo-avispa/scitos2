# scitos2_charging_dock

## Overview

This package contains the implementation of the charging dock plugin for the SCITOS and TORY robots from MetraLabs using **[opennav_docking] server**.

The plugin is responsible for detecting the charging dock and obtaining the final refined pose of the dock in the robot's frame. It uses the Iterative Closest Point (ICP) algorithm to align the template of the charging dock (previously recorded) to the current pointcloud of the dock. The plugin also uses the battery state of the robot to determine when to stop the docking process.

A **save_dock** service is provided to save the current pointcloud of the charging dock as the template for future matching.

![Docking](<doc/Docking Animation.gif>)

## Charging Dock plugin

### Subscribed Topics

* **`scan`** ([sensor_msgs/LaserScan])

	Topic where the laser scan data is published.

* **`battery`** ([sensor_msgs/BatteryState])

	Battery state of the robot.

### Published Topics

* **`dock/cloud`** ([sensor_msgs/PointCloud2])

	Pointcloud of the charging station extracted from the laser scan data. This can be enable using *debug* parameter.

* **`dock/template`** ([sensor_msgs/PointCloud2])

	Pointcloud of the recorded charging station used for matching. This can be enable using *debug* parameter.

* **`dock/target`** ([sensor_msgs/PointCloud2])

	Pointcloud of the current cluster used in the current matching. This can be enable using *debug* parameter.

### Parameters

* **`docking_threshold`** (double, default: 0.05)

	The pose threshold to the docking pose where `isDocked() = true`.

* **`staging_x_offset`** (double, default: -0.7)

	Staging pose offset forward (negative) of dock pose (m).

* **`staging_yaw_offset`** (double, default: 0.0)

	Staging pose angle relative to dock pose (rad).

* **`external_detection_timeout`** (double, default: 1.0)

	Timeout at which if the newest detection update does not meet to fail.

* **`external_detection_translation_x`** (double, default: -0.20)

	X offset from detected pose for docking pose (m).

* **`external_detection_translation_y`** (double, default: 0.0)

	Y offset from detected pose for docking pose (m).

* **`external_detection_rotation_roll`** (double, default: -1.57)

	Roll offset from detected pose for docking pose (rad).

* **`external_detection_rotation_pitch`** (double, default: 1.57)

	Pitch offset from detected pose for docking pose (rad).

* **`external_detection_rotation_yaw`** (double, default: 0.0)

	Yaw offset from detected pose for docking pose (rad).

* **`filter_coef`** (double, default: 0.1)

	Dock external detection method filtering algorithm coefficient.

* **`perception.debug`** (bool, default: false)

	Option to visualize the current point clouds used in ICP matching. 

* **`perception.icp_min_score`** (double, default: 0.01)

	ICP Fitness Score Threshold.

* **`perception.icp_max_iter`** (int, default: 200)

	Max number of iterations to fit template cloud to the target cloud.

* **`perception.icp_max_corr_dis`** (double, default: 1.0)

	Max allowable distance for matches in meters.

* **`perception.icp_max_trans_eps`** (double, default: 1.0e-8)

	Max allowable translation squared difference between two consecutive transformations.

* **`perception.icp_max_eucl_fit_eps`** (double, default: 1.0e-8)

	Maximum allowed Euclidean error between two consecutive steps in the ICP loop.

* **`perception.dock_template`** (string, default: "")

	Path to the pointcloud file of the charging station used for matching.

* **`perception.segmentation.distance_threshold`** (double, default: 0.04)

	The maximum distance between points in a cluster.

* **`perception.segmentation.min_points`** (int, default: 25)

	The minimum number of points required for a cluster to be considered valid.

* **`perception.segmentation.max_points`** (int, default: 400)

	The maximum number of points allowed in a cluster.

* **`perception.segmentation.min_distance`** (double, default: 0.0)

	The minimum distance from the sensor to a point in a cluster.

* **`perception.segmentation.max_distance`** (double, default: 2.0)

	The maximum distance from the sensor to a point in a cluster.

* **`perception.segmentation.min_width`** (double, default: 0.3)

	The minimum width of a cluster.

* **`perception.segmentation.max_width`** (double, default: 1.0)

	The maximum width of a cluster.


[opennav_docking]: https://github.com/open-navigation/opennav_docking
[sensor_msgs/LaserScan]: https://docs.ros2.org/humble/api/sensor_msgs/msg/LaserScan.html
[sensor_msgs/BatteryState]: https://docs.ros2.org/humble/api/sensor_msgs/msg/BatteryState.html
[sensor_msgs/PointCloud2]: https://docs.ros2.org/humble/api/sensor_msgs/msg/PointCloud2.html