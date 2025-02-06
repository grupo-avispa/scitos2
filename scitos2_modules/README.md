# scitos2_modules

## Overview

This package encapsulates MIRA authorities as modules, some of which expose the robot's functionality to ROS 2 via topics and services. Currently, the following `modules` are supported:

* **Charger**: Monitors the state of the battery and the charging station, providing real-time updates on battery levels and charging status.

* **Display**: Manages the status display on the robot's base, allowing for dynamic updates to the displayed information.

* **Drive**: Controls the robot's motors. This includes handling velocity commands, tracking odometry, managing bumpers, and more.

* **EBC**: Manages the power supply for additional devices connected to the robot, ensuring efficient power distribution and usage.

## Modules

### Charger

The Charger module monitors the state of the battery and the charging station.

#### Published Topics

* **`battery_state`** ([sensor_msgs/BatteryState])

	Publishes the current state of the battery.

* **`charger_status`** ([scitos2_msgs/ChargerStatus])

	Publishes the current state of the charger.

#### Services

* **`charger/save_persistent_errors`** ([scitos2_msgs/SavePersistentErrors])

	This service takes a filename as a string and saves the persistent errors of the charger to the specified file.

### Display

The Display module manages the status display on the robot's base.

#### Subscribed Topics

* **`user_menu_selected`** ([scitos2_msgs/MenuEntry])

	This topic is published when a user selects one of the sub-menus.

#### Parameters

* **`user_menu_enabled`** (bool, default: false)

	Enables or disables the user menu entry.

* **`menu_name`** (string, default: "User Menu")

	Specifies the name of the user menu entry in the main menu of the status display.

* **`menu_entry_name_1`** (string, default: "Menu Entry 1")

	Specifies the name of the first sub-menu entry in the user menu of the status display.

* **`menu_entry_name_2`** (string, default: "Menu Entry 2")

	Specifies the name of the second sub-menu entry in the user menu of the status display.

* **`menu_entry_name_3`** (string, default: "Menu Entry 3")

	Specifies the name of the third sub-menu entry in the user menu of the status display.

### Drive

The Drive module controls the robot's motors, handling velocity commands, odometry, bumpers, and more.

#### Subscribed Topics

* **`cmd_vel`** ([geometry_msgs/Twist])

	Subscribes to the velocity of the robot sent to the motor controller.

#### Published Topics

* **`odom`** ([nav_msgs/Odometry])

	Publishes the robot's odometry. This is also published as a TF between `/odom` and `/base_footprint`.

* **`bumper`** ([scitos2_msgs/BumperStatus])

	Publishes the current state of the robot's bumper.

* **`bumper_viz`** ([visualization_msgs/MarkerArray])

	Publishes markers with the state of the robot's bumper: red if it is activated, white otherwise.

* **`mileage`** ([scitos2_msgs/Mileage])

	Publishes the distance in meters that the robot has traveled since the beginning of time.

* **`drive_status`** ([scitos2_msgs/DriveStatus])

	Publishes the state of the motors, free-run mode, emergency button status, bumper status, and more.

* **`emergency_stop_status`** ([scitos2_msgs/EmergencyStopStatus])

	Publishes the current state of the emergency buttons.

* **`barrier_status`** ([scitos2_msgs/BarrierStatus])

	Publishes the current state of the magnetic barrier.

* **`rfid`** ([scitos2_msgs/RfidTag])

 	Publishes the readings of the RFID sensor.

#### Services

* **`change_force`** ([scitos2_msgs/ChangeForce])

	This service changes the force applied to the motors.

* **`emergency_stop`** ([scitos2_msgs/EmergencyStop])

	This service stops the robot. It is equivalent to the bumper being pressed - the motor stop is engaged, and can be reset with `/reset_motorstop`.

* **`enable_rfid`** ([scitos2_msgs/EnableRfid])

	This service takes a `std_msgs::Bool enabled` in the request, and gives an empty response. It is used to enable or disable the RFID sensor.

* **`enable_motors`** ([scitos2_msgs/EnableMotors])

	This service takes a `std_msgs::Bool enabled` in the request, and gives an empty response. Disabling the motors is the same as placing the robot into "Free Run" mode from the status display.

* **`reset_barrier_stop`** ([scitos2_msgs/ResetBarrierStop])

	This service is an empty request and empty response. It turns off the magnetic strip detector.

* **`reset_motorstop`** ([scitos2_msgs/ResetMotorStop])

	This service is an empty request and empty response. It turns off the motor stop, which is engaged when the robot bumps into something. It can only be turned off if the robot is not longer in collision.

* **`reset_odometry`** ([scitos2_msgs/ResetOdometry])

	This service sets the robot's odometry to zero.

* **`suspend_bumper`** ([scitos2_msgs/SuspendBumper])

	This service requests to temporarily disable the bumper.

#### Parameters

* **`robot_base_frame`** (string, default: base_link)

	Specifies the name of the base frame of the robot.

* **`odom_frame`** (string, default: odom)

	Specifies the name of the odometry frame when publishing the TF.

* **`odom_topic`** (string, default: odom)

	Specifies the name of the odometry topic.

* **`magnetic_barrier_enabled`** (bool, default: false)

	This parameter should be set to true to enable the magnetic strip detector to cut out the motors.

* **`publish_tf`** (bool, default: true)

	This parameter should be set to true to publish the TF between `odom_frame` and `robot_base_frame`.

* **`reset_bumper_interval`** (int, default: 0)

	This parameter sets the interval in milliseconds to reset motor stop when the bumper is pressed. If set to 0, the motor stop will not be reset.

* **`footprint`** (string, default: "")

	Specifies the list of points that define the footprint of the robot. The format is the same as the one used in the `nav2_costmap_2d` package.

* **`robot_radius`** (double, default: 0.5)

	Specifies the radius of the robot in meters.

### EBC

The EBC module controls the power for extra devices.

#### Parameters

* **`mcu_5v_enabled`** (bool, default: true)

	Enables or disables 5V at MCU.

* **`mcu_12v_enabled`** (bool, default: true)

	Enables or disables 12V at MCU.

* **`mcu_24v_enabled`** (bool, default: true)

	Enables or disables 24V at MCU.

* **`port0_5v_enabled`** (bool, default: true)

	Enables or disables 5V at port 0.

* **`port0_12v_enabled`** (bool, default: true)

	Enables or disables 12V at port 0.

* **`port0_24v_enabled`** (bool, default: true)

	Enables or disables 24V at port 0.

* **`port1_5v_enabled`** (bool, default: true)

	Enables or disables 5V at port 1.

* **`port1_12v_enabled`** (bool, default: true)

	Enables or disables 12V at port 1.

* **`port1_24v_enabled`** (bool, default: true)

	Enables or disables 24V at port 1.

* **`mcu_5v_max_current`** (double, default: 2.5)

	Sets the maximum current for MCU 5V in A. The value must be between 0-2.5A.

* **`mcu_12v_max_current`** (double, default: 2.5)

	Sets the maximum current for MCU 12V in A. The value must be between 0-2.5A.

* **`mcu_24v_max_current`** (double, default: 2.5)

	Sets the maximum current for MCU 24V in A. The value must be between 0-2.5A.

* **`port0_5v_max_current`** (double, default: 2.5)

	Sets the maximum current for port 0 5V in A. The value must be between 0-2.5A.

* **`port0_12v_max_current`** (double, default: 2.5)

	Sets the maximum current for port 0 12V in A. The value must be between 0-2.5A.

* **`port0_24v_max_current`** (double, default: 2.5)

	Sets the maximum current for port 0 24V in A. The value must be between 0-2.5A.

* **`port1_5v_max_current`** (double, default: 2.5)

	Sets the maximum current for port 1 5V in A. The value must be between 0-2.5A.

* **`port1_12v_max_current`** (double, default: 2.5)

	Sets the maximum current for port 1 12V in A. The value must be between 0-4A.

* **`port1_24v_max_current`** (double, default: 4)

	Sets the maximum current for port 1 24V in A. The value must be between 0-4A.


[nav_msgs/Odometry]: http://docs.ros2.org/jazzy/api/nav_msgs/msg/Odometry.html
[geometry_msgs/Twist]: http://docs.ros2.org/jazzy/api/geometry_msgs/msg/Twist.html
[visualization_msgs/MarkerArray]: http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html
[sensor_msgs/BatteryState]: https://docs.ros2.org/jazzy/api/sensor_msgs/msg/BatteryState.html
[scitos2_msgs/BarrierStatus]: ../scitos2_msgs/msg/BarrierStatus.msg
[scitos2_msgs/BumperStatus]: ../scitos2_msgs/msg/BumperStatus.msg
[scitos2_msgs/ChargerStatus]: ../scitos2_msgs/msg/ChargerStatus.msg
[scitos2_msgs/DriveStatus]: ../scitos2_msgs/msg/DriveStatus.msg
[scitos2_msgs/EmergencyStopStatus]: ../scitos2_msgs/msg/EmergencyStopStatus.msg
[scitos2_msgs/MenuEntry]: ../scitos2_msgs/msg/MenuEntry.msg
[scitos2_msgs/Mileage]: ../scitos2_msgs/msg/Mileage.msg
[scitos2_msgs/RfidTag]: ../scitos2_msgs/msg/RfidTag.msg
[scitos2_msgs/ChangeForce]: ../scitos2_msgs/srv/ChangeForce.msg
[scitos2_msgs/EmergencyStop]: ../scitos2_msgs/srv/EmergencyStop.msg
[scitos2_msgs/EnableRfid]: ./scitos2_msgs/srv/EnableRfid.msg
[scitos2_msgs/EnableMotors]: ../scitos2_msgs/srv/EnableMotors.msg
[scitos2_msgs/ResetBarrierStop]: ../scitos2_msgs/srv/ResetBarrierStop.msg
[scitos2_msgs/ResetMotorStop]: ../scitos2_msgs/srv/ResetMotorStop.msg
[scitos2_msgs/ResetOdometry]: ../scitos2_msgs/srv/ResetOdometry.msg
[scitos2_msgs/SavePersistentErrors]: ../srv/msg/SavePersistentErrors.msg
[scitos2_msgs/SuspendBumper]: ../scitos2_msgs/srv/SuspendBumper.msg