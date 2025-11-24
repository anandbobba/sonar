#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sim_microphone_sensor', executable='microphone_simulator', name='microphone_simulator', output='screen', parameters=[{'publish_rate': 10.0, 'base_noise_level': 0.1, 'whistle_intensity': 0.0}]),
        Node(package='whistle_detector', executable='whistle_detector_node', name='whistle_detector', output='screen', parameters=[{'detection_threshold': 0.7, 'confirmation_time': 0.5, 'cooldown_time': 3.0}]),
        Node(package='alert_manager', executable='alert_manager_node', name='alert_manager', output='screen'),
        Node(package='zenoh_bridge', executable='zenoh_bridge_node', name='zenoh_bridge', output='screen', parameters=[{'zenoh_mode': 'peer', 'zenoh_topic': 'cloud/alerts/whistle'}])
    ])
