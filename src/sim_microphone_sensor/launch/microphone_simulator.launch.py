#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_microphone_sensor',
            executable='microphone_simulator',
            name='microphone_simulator',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,
                'base_noise_level': 0.1,
                'whistle_intensity': 0.0
            }]
        )
    ])