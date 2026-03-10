"""
move_joint.launch.py
====================
Launch file for the move_joint example.

Usage:
    ros2 launch fanuc_tools move_joint.launch.py
    ros2 launch fanuc_tools move_joint.launch.py use_rviz:=false
    ros2 launch fanuc_tools move_joint.launch.py use_mock:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ───────────────────────────────────────────────────────
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='crx10ia_l',
        description='FANUC robot model name'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualisation'
    )

    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Use mock hardware interface'
    )

    robot_model = LaunchConfiguration('robot_model')
    use_rviz    = LaunchConfiguration('use_rviz')
    use_mock    = LaunchConfiguration('use_mock')

    # ── Config file path ───────────────────────────────────────────────────────
    fanuc_tools_share = get_package_share_directory('fanuc_tools')
    move_joint_config = os.path.join(
        fanuc_tools_share, 'motion', 'move_joint.yaml'
    )

    # ── Include FANUC MoveIt bringup ───────────────────────────────────────────
    fanuc_moveit_share  = get_package_share_directory('fanuc_moveit_config')
    fanuc_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fanuc_moveit_share, 'launch', 'fanuc_moveit.launch.py')
        ),
        launch_arguments={
            'robot_model': robot_model,
            'use_rviz':    use_rviz,
            'use_mock':    use_mock,
        }.items()
    )

    # ── move_joint node ────────────────────────────────────────────────────────
    move_joint_node = Node(
        package='fanuc_tools',
        executable='move_joint',
        name='move_joint_node',
        output='screen',
        parameters=[move_joint_config]
    )

    return LaunchDescription([
        robot_model_arg,
        use_rviz_arg,
        use_mock_arg,
        fanuc_moveit_launch,
        move_joint_node,
    ])
