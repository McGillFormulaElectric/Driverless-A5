"""Bring up the local A5 scenario publishers and grader.

Loads scenario parameters from share/a5_grader/config/params.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('a5_grader'), 'config', 'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='a5_grader',
            executable='scenario_publisher',
            name='scenario_publisher',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='a5_grader',
            executable='grader',
            name='grader',
            output='screen',
            parameters=[params_file],
        ),
    ])
