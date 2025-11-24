#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='alert_manager',
            executable='alert_manager_node',
            name='alert_manager',
            output='screen'
        )
    ])