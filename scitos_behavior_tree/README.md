# scitos_behavior_tree

## Overview

This package contaning several behavior trees plugins for the SCITOS robots. The behavior trees are implemented using the [BehaviorTree.CPP] for the core Behavior Tree processing.

The plugins included in this package are:
* **ResetMotorStop**: this action plugin reset the motor stop of the robot.
* **IsBumperActivated**: this condition plugin checks if the bumper is activated.


**Keywords:** ROS2, scitos, mira, behavior tree, behavior tree plugin

**Author: Alberto Tudela<br />**

The scitos_behavior_tree package has been tested under [ROS2] Humble on [Ubuntu] 22.04. This code is mainly based on [scitos_drivers](https://github.com/strands-project/scitos_drivers/) but ported to ROS2; expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/humble/) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

	cd colcon_workspace/src
	git clone https://github.com/grupo-avispa/scitos2.git
	cd ../
	rosdep install -i --from-path src --rosdistro humble -y
	colcon build --symlink-install

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/humble/
[BehaviorTree.CPP]: https://github.com/BehaviorTree/BehaviorTree.CPP