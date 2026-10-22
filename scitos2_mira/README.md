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

## Setup udev rules

This rules are necessary to allow the SCITOS robot to be accessed by the MIRA framework. This rules should be installed during the MIRA software installation. Hoverer, if you need to install them manually, follow the instructions below.

```bash
cd scitos2/scitos2_mira
sudo cp rules/99-SCITOS.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Nodes

### mira_framework

A lifecycle node that interfaces with the MIRA framework.

#### Parameters

* **`module_plugins`** (string array, default: "")

	Specifies the modules to be loaded.

* **`mira_robot_resource`** (string, default: "/robot/Robot")

	The MIRA resource to wait for before activating the modules, which write properties of the robot when they are activated.

* **`mira_resource_timeout`** (int, default: 5000)

	Milliseconds to wait for that resource. If it does not appear the modules are activated anyway.

* **`mira_port`** (int, default: 0)

	Port where the MIRA framework listens, so that `miracenter` or `mirainspect` can connect to the robot while ROS is running (`mirainspect channel list -k<port>`). 0 disables it.

* **`mira_known_frameworks`** (string array, default: [])

	Other MIRA frameworks to connect to, as `host:port`.

* **`mira_variables`** (string array, default: [])

	Variables of the MIRA configuration, as `name=value`. The first definition of a variable wins.

* **`save_processed_config`** (string, default: "")

	File where MIRA saves the configuration once its variables are resolved, to debug it.

* **`mira_log_level`** (int, default: -1)

	Severity of the messages of MIRA, from 0 (critical) to 5 (trace). -1 follows the log level of the node.

* **`scitos_config`** (string, default: "")

	Specifies the path to the SCITOS robot configuration file in XML format. This parameter should point to your SCITOSDriver.xml robot config file, which should have been installed during the MIRA software installation. Typically, this file is located in the ``/opt/SCITOS/ directory``.
