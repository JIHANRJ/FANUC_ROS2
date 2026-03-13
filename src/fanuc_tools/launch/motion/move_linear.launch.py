"""
move_linear.launch.py
=====================
Launches MoveIt with the pointer tool + move_linear node.

Usage:
    ros2 launch fanuc_tools move_linear.launch.py use_mock:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz'
    )
    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='true',
        description='Use mock hardware'
    )

    config = os.path.join(
        get_package_share_directory('fanuc_tools'),
        'motion', 'move_linear.yaml'
    )

    fanuc_moveit_share = get_package_share_directory('fanuc_moveit_config')
    fanuc_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fanuc_moveit_share, 'launch', 'fanuc_moveit.launch.py')
        ),
        launch_arguments={
            'robot_model': 'crx10ia_l_pointer',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_mock': LaunchConfiguration('use_mock'),
        }.items()
    )

    move_linear_node = Node(
        package='fanuc_tools',
        executable='move_linear',
        name='move_linear_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        use_rviz_arg,
        use_mock_arg,
        fanuc_moveit_launch,
        move_linear_node,
    ])
