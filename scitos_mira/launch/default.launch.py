#!/usr/bin/env python3

'''
    Launches a Scitos MIRA node.
'''

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from nav2_common.launch import RewrittenYaml

import launch.events
import lifecycle_msgs.msg

def generate_launch_description():

    # Default filenames and where to find them
    scitos_dir = get_package_share_directory('scitos_mira')

    # Read the YAML parameters file.
    default_mira_param_file = os.path.join(scitos_dir, 'params', 'default.yaml')
    default_scitos_config_file = os.path.join('/opt/SCITOS/', 'SCITOSRobotAttributes.xml')

    # Create the launch configuration variables.
    mira_param_file = LaunchConfiguration('mira_param_file', default = default_mira_param_file)
    scitos_config_file = LaunchConfiguration('scitos_config_file', default = default_scitos_config_file)

    # Map these variables to arguments: can be set from the command line or a default will be used
    mira_param_file_launch_arg = DeclareLaunchArgument(
        'mira_param_file',
        default_value = default_mira_param_file,
        description = 'Full path to the Mira parameter file to use'
    )

    scitos_config_file_launch_arg = DeclareLaunchArgument(
        'scitos_config_file',
        default_value = default_scitos_config_file,
        description = 'Full path to the Scitos parameter file to use'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'scitos_config': scitos_config_file
    }

    configured_params = RewrittenYaml(
        source_file = mira_param_file,
        root_key = '',
        param_rewrites = param_substitutions,
        convert_types = True
    )

    # Prepare the Scitos mira
    scitos_mira_node = LifecycleNode(
        package = 'scitos_mira',
        namespace = '',
        executable = 'scitos_mira',
        name = 'scitos_mira',
        parameters = [configured_params],
        emulate_tty = True
    )

    # When the scitos node reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_node_reaches_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = scitos_mira_node,
            goal_state = 'inactive',
            entities = [
                EmitEvent(event = ChangeState(
                    lifecycle_node_matcher = launch.events.matches_action(scitos_mira_node),
                    transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Make the scitos node take the 'configure' transition.
    emit_event_to_request_that_node_does_configure_transition = EmitEvent(
            event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(scitos_mira_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        mira_param_file_launch_arg,
        scitos_config_file_launch_arg,
        register_event_handler_for_node_reaches_inactive_state,
        emit_event_to_request_that_node_does_configure_transition,
        scitos_mira_node,
    ])