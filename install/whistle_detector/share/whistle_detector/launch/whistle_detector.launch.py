#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='whistle_detector',
            executable='whistle_detector_node',
            name='whistle_detector',
            output='screen',
            parameters=[{
                'detection_threshold': 0.7,
                'confirmation_time': 0.5,
                'cooldown_time': 3.0
            }]
        )
    ])