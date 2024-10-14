# scitos2

![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/grupo-avispa/scitos2)
[![Build](https://github.com/grupo-avispa/scitos2/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/grupo-avispa/scitos2/actions/workflows/build.yml)
[![Docker image](https://github.com/grupo-avispa/scitos2/actions/workflows/docker_image.yml/badge.svg?branch=main)](https://github.com/grupo-avispa/scitos2/actions/workflows/docker_image.yml)
[![codecov](https://codecov.io/gh/grupo-avispa/scitos2/graph/badge.svg?token=794XFYV0FK)](https://codecov.io/gh/grupo-avispa/scitos2)

## Overview

`scitos2` is a ROS 2 stack designed for Metralabs robots that utilize the MIRA framework, including models such as SCITOS, TORY, MORPHIA, etc. This stack comprises several packages, each serving a unique purpose:

 * [scitos2_behavior_tree]: This package contains behavior tree nodes that extend your robot's functionalities, such as emergency stop, reset motor stop, etc.
 * [scitos2_charging_dock]: This package contains the implementation of the charging dock plugin for the SCITOS and TORY robots from MetraLabs using the opennav_docking server.
 * [scitos2_common]: This package provides common functionalities for the scitos2 stack.
 * [scitos2_core]: This package provides the abstract interface (virtual base classes) for the Scitos Modules.
 * [scitos2_mira]: This is the main node that interfaces with the MIRA framework.
 * [scitos2_modules]: This package implements the MIRA authorities as modules.
 * [scitos2_msgs]: This package contains messages and services related to the Metralabs Scitos robot base.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/rolling/) (middleware for robotics)
- [MIRA](https://www.mira-project.org/) (Metralabs middleware for robotic applications)

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using the following command:
```bash
cd colcon_workspace/src
git clone https://github.com/grupo-avispa/scitos2.git
cd ../
rosdep install -i --from-path src --rosdistro rolling -y
colcon build --symlink-install
```

### Docker

A Dockerfile is provided in the repository to build the `scitos2` stack. This setup ensures that you have a consistent and isolated environment. The image sets a launch file to start the `mira_framework` with the default configuration file: `drive` and `charger` modules. This configuration can be modified by changing the `CMD` instruction in the Dockerfile.

To build and run the `scitos2` stack using Docker, follow these steps:

#### Building the Docker image

First, build the Docker image using the provided Dockerfile in the repository:
```bash
cd /path/to/scitos2
docker build -t grupoavispa/scitos2:latest .
```

#### Running the Docker container

Once the image is built, you can run the container using the docker compose file provided in the repository. Before running the container:
- Install the `udev` rules on your host machine to allow the container to access the robot's hardware.
- Make sure the license file is in the `/opt/MIRA-licenses` directory on your host machine.
- Install the configuration files for the robot in the `/opt/SCITOS` directory on your host machine.
- Update the `docker-compose.yml` file by replacing the XXX placeholders with the correct elements.

Then, run the container using the following command:
```bash
docker-compose up
```

[scitos2_behavior_tree]: /scitos2_behavior_tree
[scitos2_charging_dock]: /scitos2_charging_dock
[scitos2_common]: /scitos2_common
[scitos2_core]: /scitos2_core
[scitos2_mira]: /scitos2_mira
[scitos2_modules]: /scitos2_modules
[scitos2_msgs]: /scitos2_msgs
