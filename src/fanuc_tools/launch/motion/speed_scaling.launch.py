"""
speed_scaling.launch.py
=======================
Launches the speed_scaling node on its own.
Run this alongside move_joint to control speed during motion.

Usage:
    ros2 launch fanuc_tools speed_scaling.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('fanuc_tools'),
        'motion',
        'speed_scaling.yaml'
    )

    speed_scaling_node = Node(
        package='fanuc_tools',
        executable='speed_scaling',
        name='speed_scaling_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        speed_scaling_node,
    ])
