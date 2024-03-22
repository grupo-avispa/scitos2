# scitos2

![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/grupo-avispa/scitos2)

## Overview

A ROS 2 stack that contains modules, messages and services for Metralabs robots using the MIRA framework, such as the SCITOS, TORY, MORPHIA, etc. robots. The stack is composed of the following packages:

 * [scitos_behavior_tree]: behavior tree nodes used to add functionality to your behaviors (emergency stop, reset motor stop, etc.).
 * [scitos2_core]: abstract interface (virtual base classes) for the Scitos Modules.
 * [scitos2_mira]: main node that interfaces with the MIRA framework.
 * [scitos2_modules]: implementation of the MIRA authorities as modules.
 * [scitos2_msgs]: messages and services relating to Metralabs Scitos robot base.

**Keywords:** ROS, ROS2, metralabs, mira, scitos

**Author: Alberto Tudela<br />**

The scitos2 package has been tested under [ROS2] Humble on [Ubuntu] 22.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/humble/) (middleware for robotics),
- [MIRA](https://www.mira-project.org/) (Metralabs middleware for robotic applications),

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

	cd colcon_workspace/src
	git clone https://github.com/grupo-avispa/scitos2.git
	cd ../
	rosdep install -i --from-path src --rosdistro humble -y
	colcon build --symlink-install

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/humble/
[scitos_behavior_tree]: /scitos_behavior_tree
[scitos2_mira]: /scitos2_mira
[scitos2_modules]: /scitos2_modules
[scitos2_core]: /scitos2_core
[scitos2_msgs]: /scitos2_msgs
