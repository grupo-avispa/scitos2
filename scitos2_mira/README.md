# scitos2_mira

## Overview

The `scitos2_mira` package serves as the primary interface between the MIRA framework and Metralabs robots. This package contains the main lifecycle node responsible for initiating and terminating the MIRA framework, loading the robot's configuration files, and providing a ROS 2 interface for the robot's modules.

## Usage

1. Modify the *default.yaml* configuration file located in the `params` directory to specify the modules you wish to utilize with your robot.

2. Update the *default.launch.py* launch file in the `launch` directory. Change the `scitos_config` parameter to the path of your SCITOS robot's configuration.

3. Launch the `scitos2_mira` node using the following command:

```bash
ros2 launch scitos2_mira mira_launch.py
```

## Nodes

### mira_framework

A lifecycle node that interfaces with the MIRA framework.

#### Parameters

* **`module_plugins`** (string array, default: "")

	Specifies the modules to be loaded.

* **`scitos_config`** (string, default: "")

	Specifies the path to the SCITOS robot configuration file in XML format. This parameter should point to your SCITOSDriver.xml robot config file, which should have been installed during the MIRA software installation. Typically, this file is located in the ``/opt/SCITOS/ directory``.
