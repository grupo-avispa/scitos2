# scitos2_msgs

## Overview
This package contains messages and services for the Metralabs Scitos robot base.

## Messages (.msg)
* [BarrierStatus](msg/BarrierStatus.msg): Provides information about the current status of the barrier.
* [BumperStatus](msg/BumperStatus.msg): Provides information about the current status of the bumper.
* [BatteryState](msg/BatteryState.msg): **DEPRECATED AS OF GALACTIC. Please use sensor_msgs/BatteryState instead**
* [ChargerStatus](msg/ChargerStatus.msg): Provides information about the current status of the charger.
* [DriveStatus](msg/DriveStatus.msg): Provides information about the current status of the hardware.
* [EmergencyStopStatus](msg/EmergencyStopStatus.msg): Provides information about the current status of the emergency stop button.
* [MenuEntry](msg/MenuEntry.msg): Represents the entry number for the built-in status display.
* [Mileage](msg/Mileage.msg): Represents the total distance that the robot has traveled.
* [RfidTag](msg/RfidTag.msg): Represents the code of an RFID tag.

## Services (.srv)
* [ChangeForce](srv/ChangeForce.srv): Service to change the force applied to the motors.
* [EmergencyStop](srv/EmergencyStop.srv): Service to perform an emergency stop and set the motor emergency stop flag.
* [EnableMotors](srv/EnableMotors.srv): Service to enable or disable the motors.
* [EnableRfid](srv/EnableRfid.srv): Service to enable or disable the RFID reader.
* [SaveDock](srv/SaveDock.srv): Service to record the save the current dock pointcloud as a PCD file.
* [ResetBarrierStop](srv/ResetBarrierStop.srv): Service to reset the magnetic barrier stop flag.
* [ResetMotorStop](srv/ResetMotorStop.srv): Service to reset the motor stop flags (bumper, emergency stop flags, etc).
* [ResetOdometry](srv/ResetOdometry.srv): Service to reset the odometry to zero.
* [SavePersistentErrors](srv/SavePersistentErrors.srv): Service to save the hardware errors to a file.
* [SuspendBumper](srv/SuspendBumper.srv): Service to temporarily disable the bumper.