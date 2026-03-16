"""
speed_scaling.launch.py
=======================
Launches the speed_scaling node on its own.
Run this alongside move_joint to control speed during motion.

Usage:
    ros2 launch fanuc_tools speed_scaling.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    speed_scaling_node = Node(
        package='fanuc_tools',
        executable='speed_scaling',
        name='speed_scaling_node',
        output='screen'
    )

    return LaunchDescription([
        speed_scaling_node,
    ])
