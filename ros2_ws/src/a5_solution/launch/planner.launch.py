"""Launch A5.1 centerline planner under the student's GitHub-username namespace.

Loads parameters from share/a5_solution/config/params.yaml.

Usage:
    ros2 launch a5_solution planner.launch.py github_user:=<your-handle>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    github_user = LaunchConfiguration('github_user')
    params_file = os.path.join(
        get_package_share_directory('a5_solution'), 'config', 'params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'github_user',
            description='Your GitHub username; used as the ROS namespace.',
        ),
        Node(
            package='a5_solution',
            executable='planner_node',
            name='planner_node',
            namespace=github_user,
            output='screen',
            parameters=[params_file],
        ),
    ])
