"""
load_pointer.launch.py
======================
Loads the CRX-10iA/L pointer robot description and starts robot_state_publisher.

Usage:
    ros2 launch fanuc_tools load_pointer.launch.py robot_model:=crx10ia_l_pointer
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='crx10ia_l_pointer',
        description='FANUC robot model name'
    )

    robot_model = LaunchConfiguration('robot_model')

    robot_description = ParameterValue(
        value=Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('fanuc_hardware_interface'),
                'robot',
                PythonExpression(["'", robot_model, ".urdf.xacro'"]),
            ]),
            ' ', 'robot_model:=', robot_model,
            ' ', 'robot_series:=crx',
            ' ', 'robot_ip:=1.1.1.1',
            ' ', 'use_mock:=true',
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }]
    )

    return LaunchDescription([
        robot_model_arg,
        robot_state_publisher,
    ])