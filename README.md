# scitos_msgs

![ROS2](https://img.shields.io/badge/ros2-galactic-purple?logo=ros&logoColor=white)
![License](https://img.shields.io/badge/license-MIT-green)
[![Build](https://github.com/ajtudela/scitos_msgs/actions/workflows/build.yml/badge.svg?branch=galactic)](https://github.com/ajtudela/scitos_msgs/actions/workflows/build.yml)

## Overview
This package provides messages and services relating to Metralabs Scitos robot base.

## Messages (.msg)
* [BarrierStatus](msg/BarrierStatus.msg): Describes the current state of the barrier.
* [BumperStatus](msg/BumperStatus.msg): Describes the current state of the bumper.
* [BatteryState](msg/BatteryState.msg): **THIS MESSAGE IS DEPRECATED AS OF GALACTIC, use sensor_msgs/BatteryState instead**
* [ChargerStatus](msg/ChargerStatus.msg): **THIS MESSAGE IS DEPRECATED AS OF GALACTIC, use sensor_msgs/BatteryState instead**
* [DriveStatus](msg/DriveStatus.msg): Describes the current hardware state.
* [EmergencyStopStatus](msg/EmergencyStopStatus.msg): Describes the current state of the emergency stop button.

## Services (.srv)
* [ChangeForce](srv/ChangeForce.srv): Request change the force applied to the motors.
* [EmergencyStop](srv/EmergencyStop.srv): Request performs an emergency stop and sets the motor emergency stop flag.
* [EnableMotors](srv/EnableMotors.srv): Request enable or disable the motors.
* [EnableRfid](srv/EnableRfid.srv): Request enable or disable the RFID reader.
* [ResetMotorStop](srv/ResetMotorStop.srv): Request reset the motor stop flags (bumper, emergency stop flags, etc).
* [ResetOdometry](srv/ResetOdometry.srv): Request reset the odometry to zero.
* [SavePersistentErrors](srv/SavePersistentErrors.srv): Request save the hardware errors to a file.
* [SuspendBumper](srv/SuspendBumper.srv): Request temporarily disable the bumper.