"""Launch A5.2 pure-pursuit controller under the student's GitHub-username namespace.

Loads controller_node's tunables (lookahead_m, v_target_min, v_target_max)
from share/a5_new_member/config/params.yaml.

Usage:
    ros2 launch a5_new_member controller.launch.py github_user:=<your-handle>

Brings up both the planner and the controller together, since the
controller needs the planner's centerline to have anything to follow.
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
        get_package_share_directory('a5_new_member'), 'config', 'params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'github_user',
            description='Your GitHub username; used as the ROS namespace.',
        ),
        Node(
            package='a5_new_member',
            executable='planner_node',
            name='planner_node',
            namespace=github_user,
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='a5_new_member',
            executable='controller_node',
            name='controller_node',
            namespace=github_user,
            output='screen',
            parameters=[params_file],
        ),
    ])
