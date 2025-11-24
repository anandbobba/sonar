#!/usr/bin/env python3
"""
Launch RViz with whistle follower configuration
Can be used standalone or will be auto-launched with the main system
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    whistle_detector_pkg = get_package_share_directory('whistle_detector')
    rviz_config = os.path.join(whistle_detector_pkg, 'rviz', 'whistle_follower.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
