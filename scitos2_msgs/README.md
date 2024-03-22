# scitos2_msgs

## Overview
This package provides messages and services relating to Metralabs Scitos robot base.

## Messages (.msg)
* [BarrierStatus](msg/BarrierStatus.msg): Describes the current state of the barrier.
* [BumperStatus](msg/BumperStatus.msg): Describes the current state of the bumper.
* [BatteryState](msg/BatteryState.msg): **THIS MESSAGE IS DEPRECATED AS OF GALACTIC, use sensor_msgs/BatteryState instead**
* [ChargerStatus](msg/ChargerStatus.msg): Describes the current state of the charger.
* [DriveStatus](msg/DriveStatus.msg): Describes the current hardware state.
* [EmergencyStopStatus](msg/EmergencyStopStatus.msg): Describes the current state of the emergency stop button.
* [MenuEntry](msg/MenuEntry.msg): Describes the entry number for the embedded status display.
* [Mileage](msg/Mileage.msg): Describes the distance that the robot has travelled.
* [RfidTag](msg/RfidTag.msg): Describes the code of a RFID tag.

## Services (.srv)
* [ChangeForce](srv/ChangeForce.srv): Request change the force applied to the motors.
* [EmergencyStop](srv/EmergencyStop.srv): Request performs an emergency stop and sets the motor emergency stop flag.
* [EnableMotors](srv/EnableMotors.srv): Request enable or disable the motors.
* [EnableRfid](srv/EnableRfid.srv): Request enable or disable the RFID reader.
* [ResetBarrierStop](srv/ResetBarrierStop.srv): Request reset the magnetic barrier stop flag.
* [ResetMotorStop](srv/ResetMotorStop.srv): Request reset the motor stop flags (bumper, emergency stop flags, etc).
* [ResetOdometry](srv/ResetOdometry.srv): Request reset the odometry to zero.
* [SavePersistentErrors](srv/SavePersistentErrors.srv): Request save the hardware errors to a file.
* [SuspendBumper](srv/SuspendBumper.srv): Request temporarily disable the bumper.