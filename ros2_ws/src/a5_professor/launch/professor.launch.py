"""Bring up the professor side: cone-map + bicycle sim + grader."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a5_professor',
            executable='sim_node',
            name='sim_node',
            output='screen',
        ),
        Node(
            package='a5_professor',
            executable='grader',
            name='grader',
            output='screen',
        ),
    ])
