#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # GUI node
    gui_node = Node(
        package='codebotler_amrl_impl',
        executable='gui.py',
        name='gui',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Robot actions node
    actions_node = Node(
        package='codebotler_amrl_impl',
        executable='actions.py',
        name='robot_low_level_actions',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        gui_node,
        actions_node
    ])
