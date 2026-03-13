"""
collaborative_speed.launch.py
=============================
Launches the collaborative_speed monitor node.

Note:
    Requires real hardware with the FANUC GPIO controller running.
    Will not receive messages in mock hardware mode.

Usage:
    ros2 launch fanuc_tools collaborative_speed.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('fanuc_tools'),
        'motion',
        'collaborative_speed.yaml'
    )

    collaborative_speed_node = Node(
        package='fanuc_tools',
        executable='collaborative_speed',
        name='collaborative_speed_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        collaborative_speed_node,
    ])
