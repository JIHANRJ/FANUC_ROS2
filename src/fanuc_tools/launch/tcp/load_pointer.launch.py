"""
load_pointer.launch.py
======================
Loads the CRX-10iA/L pointer robot description and starts robot_state_publisher.

Usage:
    ros2 launch fanuc_tools load_pointer.launch.py robot_model:=crx10ia_l_pointer
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='crx10ia_l_pointer',
        description='FANUC robot model name'
    )

    robot_model = LaunchConfiguration('robot_model')
    robot_model_str = robot_model.perform(None) if hasattr(robot_model, 'perform') else 'crx10ia_l_pointer'

    hardware_share = get_package_share_directory('fanuc_hardware_interface')
    robot_urdf_path = os.path.join(
        hardware_share,
        'robot',
        f'{robot_model_str}.urdf.xacro'
    )

    robot_description_content = xacro.process_file(
        robot_urdf_path,
        mappings={
            'robot_model': robot_model_str,
            'robot_ip': '1.1.1.1',
            'use_mock': 'true',
            'robot_series': 'crx',
            'gpio_configuration': '',
        }
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
        }]
    )

    return LaunchDescription([
        robot_model_arg,
        robot_state_publisher,
    ])