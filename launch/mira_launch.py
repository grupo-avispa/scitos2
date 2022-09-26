# basic launch

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Read the YAML parameters file.
    config = os.path.join(
        get_package_share_directory('scitos_mira'),
        'config',
        'mira_config.yaml'
        )

    # Prepare the sicks300_2 node.
    scitos_mira_node = Node(
            package = 'scitos_mira',
            namespace = '',
            executable = 'scitos_mira',
            name = 'scitos_mira',
            parameters = [config],
            emulate_tty = True
        )

    return LaunchDescription([
        scitos_mira_node,
    ])