"""Bring up the Neil side: cone-map + bicycle sim + grader.

Loads scenario parameters from share/a5_neil/config/params.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('a5_neil'), 'config', 'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='a5_neil',
            executable='sim_node',
            name='sim_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='a5_neil',
            executable='grader',
            name='grader',
            output='screen',
            parameters=[params_file],
        ),
    ])
