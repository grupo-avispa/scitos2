#!/usr/bin/env python3

'''
    Launches a Scitos node to control the robot base.
'''

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Default filenames and where to find them
    scitos_dir = get_package_share_directory('scitos_mira')

    # Read the YAML parameters file.
    default_mira_param_file = os.path.join(scitos_dir, 'config', 'default.yaml')
    default_scitos_param_file = os.path.join(scitos_dir, 'resources', 'CLARC', 'SCITOSDriver.xml')

    # Create the launch configuration variables.
    mira_param_file = LaunchConfiguration('mira_param_file', default=default_mira_param_file)
    scitos_param_file = LaunchConfiguration('scitos_param_file', default=default_scitos_param_file)

    # Map these variables to arguments: can be set from the command line or a default will be used
    mira_param_file_launch_arg = DeclareLaunchArgument(
        'mira_param_file',
        default_value=default_mira_param_file,
        description='Full path to the Mira parameter file to use'
    )
    sicks300_param_file_launch_arg = DeclareLaunchArgument(
        'scitos_param_file',
        default_value=default_scitos_param_file,
        description='Full path to the Scitos parameter file to use'
    )

    # Scitos mira
    scitos_mira_node = Node(
        package = 'scitos_mira',
        namespace = '',
        executable = 'scitos_mira',
        #name = 'scitos_mira',     # Uncomment to 'hide' charger, drive, etc. nodes under scitos mira.
        parameters = [default_mira_param_file, {'scitos_config': default_scitos_param_file}],
        emulate_tty = True
    )

    return LaunchDescription([
        mira_param_file_launch_arg,
        sicks300_param_file_launch_arg,
        scitos_mira_node,
    ])