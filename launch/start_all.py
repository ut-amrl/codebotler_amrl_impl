#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # TODO: take care of transitioning this part properly yourselves
    # External ROS1 launch file that needs ROS2 equivalent
    ut_jackal_launch_file = os.path.join(
        get_package_share_directory('ut_jackal'),
        'launch',
        'autonomy.launch.py'
    )
    
    # Include the ut_jackal autonomy stack
    ut_jackal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ut_jackal_launch_file)
    )
    
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
        ut_jackal_launch,
        gui_node,
        actions_node
    ])
