#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zenoh_bridge',
            executable='zenoh_bridge_node',
            name='zenoh_bridge',
            output='screen',
            parameters=[{
                'zenoh_mode': 'peer',
                'zenoh_topic': 'cloud/alerts/whistle',
                'connect_endpoint': ''
            }]
        )
    ])