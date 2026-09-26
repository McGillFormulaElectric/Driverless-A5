"""Bring up Neil's side: reference signal publisher + grader.

Loads scenario parameters from share/a2_grader/config/params.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('a2_grader'), 'config', 'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='a2_grader',
            executable='signal_publisher',
            name='signal_publisher',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='a2_grader',
            executable='grader',
            name='grader',
            output='screen',
            parameters=[params_file],
        ),
    ])
