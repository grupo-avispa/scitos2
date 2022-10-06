# scitos_mira

![ROS2](https://img.shields.io/badge/ros2-galactic-purple?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/license-GPL-green)

## Overview

This package contaning drivers for Metralabs Scitos robot base (HG4 version) using the MIRA framework.

Several sensors and components of SCITOS robots are exposed as ROS  published / subscribed topics and services. At the moment the included ``modules`` are:

* **Charger**: this module monitor the state of the battery and the charging station.
* **Drive**: this module control the motors (velocity commands, odometry, bumpers, ...).

**Keywords:** ROS2, scitos, mira, metralabs

**Author: Alberto Tudela<br />**

The scitos_mira package has been tested under [ROS2] Galactic on [Ubuntu] 20.04. This code is mainly based on [scitos_drivers](https://github.com/strands-project/scitos_drivers/) but ported to ROS2; expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/galactic/) (middleware for robotics),
- [Boost](https://www.boost.org/) (C++ source libraries)

		rosdep install -i --from-path src --rosdistro galactic -y

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

	cd colcon_workspace/src
	git clone https://github.com/ajtudela/scitos_mira.git
	cd ../
	rosdep install -i --from-path src --rosdistro galactic -y
	colcon build

## Usage

First, edit the configuration file *mira_config.yaml* in the ``config`` folder. You MUST edit the modules you want to use and the  configuration of your SCITOS robot in xml format.

Then, run the scitos_mira node with:

	ros2 launch scitos_mira mira_launch.py 

## Modules

### Charger

This module monitor the state of the battery and the charging station.

#### Published Topics

* **`battery_state`** ([sensor_msgs/BatteryState])

	State of the battery.

* **`charger_status`** ([scitos_msgs/ChargerStatus])

	State of the charger.

#### Services

* **`charger/save_persistent_errors`** ([scitos_msgs/SavePersistentErrors])

	This services takes a filename as a string and saves the persistent errors of the charger.

## Nodes

### Drive

This module control the motors (velocity commands, odometry, bumpers, ...).

#### Subscribed Topics

* **`cmd_vel`** ([geometry_msgs/Twist])

	Velocity of the robot sent to the motor controller.

#### Published Topics

* **`odom`** ([nav_msgs/Odometry])

	Odometry of the robot. This is also published as a TF between `/odom` and `/base_footprint`.

* **`bumper`** ([scitos_msgs/BumperStatus])

	State of the robot bumper.

* **`mileage`** ([scitos_msgs/Mileage])

	The distance in metres that the robot has travelled since the beginning of time.

* **`drive_status`** ([scitos_msgs/DriveStatus])

	The state of the motors, free-run mode, emergency button status, bumper status, ...

* **`emergency_stop_status`** ([scitos_msgs/EmergencyStopStatus])

	The state of the emergency buttons.

#### Services

* **`change_force`** ([scitos_msgs/ChangeForce])

	Change the force applied to the motors.

* **`emergency_stop`** ([scitos_msgs/EmergencyStop])

	This empty request/response service stops the robot. It is equivalent to the bumper being pressed - the motor stop is engaged, and can be reset with /reset_motorstop.

* **`enable_motors`** ([scitos_msgs/EnableMotors])

	This service takes a `std_msgs::Bool enabled` in the request, and gives an empty response. Disabling the motors is the same as placing the robot into "Free Run" mode from the status display.

* **`reset_motorstop`** ([scitos_msgs/ResetMotorStop])

	This service is an empty request and empty response. It turns off the motor stop, which is engaged when the robot bumps into something. It can only be turned off if the robot is not longer in collision.

* **`reset_odometry`** ([scitos_msgs/ResetOdometry])

	This empty request/response service sets the robot odometry to zero.

* **`suspend_bumper`** ([scitos_msgs/SuspendBumper])

	This service requests temporarily disable the bumper.

## Future work
- [ ] Convert nodes to LifeCycleNodes.
- [ ] Add Drive parameters.
- [ ] Add other modules.
- [ ] Improve parameter loading.

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/galactic/
[nav_msgs/Odometry]: http://docs.ros2.org/galactic/api/nav_msgs/msg/Odometry.html
[geometry_msgs/Twist]: http://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html
[sensor_msgs/BatteryState]: https://docs.ros2.org/galactic/api/sensor_msgs/msg/BatteryState.html
[scitos_msgs/BumperStatus]: ../scitos_msgs/msg/BumperStatus.msg
[scitos_msgs/ChargerStatus]: ../scitos_msgs/msg/ChargerStatus.msg
[scitos_msgs/DriveStatus]: ../scitos_msgs/msg/DriveStatus.msg
[scitos_msgs/EmergencyStopStatus]: ../scitos_msgs/msg/EmergencyStopStatus.msg
[scitos_msgs/Mileage]: ../scitos_msgs/msg/Mileage.msg
[scitos_msgs/ChangeForce]: ../scitos_msgs/srv/ChangeForce.msg
[scitos_msgs/EmergencyStop]: ../scitos_msgs/srv/EmergencyStop.msg
[scitos_msgs/EnableMotors]: ../scitos_msgs/srv/EnableMotors.msg
[scitos_msgs/ResetMotorStop]: ../scitos_msgs/srv/ResetMotorStop.msg
[scitos_msgs/ResetOdometry]: ../scitos_msgs/srv/ResetOdometry.msg
[scitos_msgs/SavePersistentErrors]: ../srv/msg/SavePersistentErrors.msg
[scitos_msgs/SuspendBumper]: ../scitos_msgs/srv/SuspendBumper.msg