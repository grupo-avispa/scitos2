# scitos2_mira

## Overview

This package contains the main lifecycle node to interface with the MIRA framework for Metralabs robots. The node is responsible for starting and stopping the MIRA framework, as well as loading the configuration files for the robot. The node also interfaces with the MIRA framework to provide the ROS 2 interface for the robot's modules.

## Usage

First, edit the configuration file *default.yaml* in the ``params`` folder with the modules you want to use in your robot.

Second, edit the launch file *default.launch.py* in the ``launch`` folder and change the value of the ``scitos_config`` parameter with the path of the configuration of your SCITOS robot.

Then, run the scitos2_mira node with:

	ros2 launch scitos2_mira mira_launch.py

## Nodes

### mira_framework

Lifecycle node to interface with the MIRA framework.

#### Parameters

* **`module_plugins`** (string array, default: "")

	List of modules to load.

* **`scitos_config`** (string, default: "")

	Path to the SCITOS robot configuration file in XML format. This parameter must point to your SCITOSDriver.xml robot config file that must be installed when you installed MIRA software. Usually, this file is located in the ``/opt/SCITOS/`` folder.
