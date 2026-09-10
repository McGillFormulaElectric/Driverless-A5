"""Launch A5.1 centerline planner under the student's GitHub-username namespace.

Usage:
    ros2 launch a5_student planner.launch.py github_user:=<your-handle>
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
    ])
