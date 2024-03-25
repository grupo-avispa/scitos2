# scitos2_behavior_tree

## Overview

This package provides several behavior tree plugins for the SCITOS robots. These plugins are designed to facilitate the control and management of the robot's actions and responses. The behavior tree nodes are implemented using the [BehaviorTree.CPP] library, which provides the core functionality for behavior tree processing.

The plugins included in this package are:

* **EmergencyStop**: An action plugin that immediately halts the robot's movements. This is typically used in situations where the robot encounters an unexpected obstacle or error.

* **ResetMotorStop**: An action plugin that resets the robot's motor stop. This is typically used after the robot has been halted due to an emergency stop or other interruption, and it is safe for the robot to resume movement.

* **IsBumperActivated**: A condition plugin that checks whether the robot's bumper has been activated. This is typically used to detect collisions or close proximity to obstacles.

[BehaviorTree.CPP]: https://github.com/BehaviorTree/BehaviorTree.CPP