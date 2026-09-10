"""Launch A5.2 pure-pursuit controller under the student's GitHub-username namespace.

Usage:
    ros2 launch a5_student controller.launch.py github_user:=<your-handle>

Typically you'll want to run both the planner and the controller together;
launch this after `planner.launch.py` (or add a second `Node(...)` entry
below to bring them up in one shot).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    github_user = LaunchConfiguration('github_user')
    return LaunchDescription([
        DeclareLaunchArgument(
            'github_user',
            description='Your GitHub username; used as the ROS namespace.',
        ),
        Node(
            package='a5_student',
            executable='planner_node',
            name='planner_node',
            namespace=github_user,
            output='screen',
        ),
        Node(
            package='a5_student',
            executable='controller_node',
            name='controller_node',
            namespace=github_user,
            output='screen',
        ),
    ])
