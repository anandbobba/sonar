#!/usr/bin/env python3
"""
Launch Gazebo with the whistle system. Includes optional demo helper to enable the virtual whistle.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('whistle_gazebo')
    robot_desc_pkg = get_package_share_directory('whistle_robot_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'whistle_test.world')
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'whistle_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Launch arguments
    declare_autonomous = DeclareLaunchArgument('autonomous', default_value='true', description='Enable autonomous responder')
    declare_sound_x = DeclareLaunchArgument('sound_x', default_value='3.0', description='X coordinate of sound source')
    declare_sound_y = DeclareLaunchArgument('sound_y', default_value='3.0', description='Y coordinate of sound source')
    declare_demo = DeclareLaunchArgument('demo', default_value='false', description='Run demo helper to enable virtual whistle')

    autonomous_cfg = LaunchConfiguration('autonomous')
    sound_x_cfg = LaunchConfiguration('sound_x')
    sound_y_cfg = LaunchConfiguration('sound_y')
    demo_cfg = LaunchConfiguration('demo')

    ld = LaunchDescription([
        declare_autonomous,
        declare_sound_x,
        declare_sound_y,
        declare_demo,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'whistle_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        ),

        Node(
            package='sim_microphone_sensor',
            executable='microphone_simulator',
            name='microphone_simulator',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,
                'base_noise_level': 0.1,
                'whistle_intensity': 0.0,
                'use_sim_time': True
            }]
        ),

        Node(
            package='whistle_detector',
            executable='whistle_detector_node',
            output='screen',
            parameters=[{
                'detection_threshold': 0.7,
                'confirmation_time': 0.5,
                'cooldown_time': 3.0,
                'use_sim_time': True
            }]
        ),

        Node(
            package='whistle_detector',
            executable='autonomous_responder',
            output='screen',
            condition=IfCondition(autonomous_cfg),
            parameters=[{
                'activation_threshold': 0.15,
                'approach_speed': 0.3,
                'stop_threshold': 0.8,
                'sound_x': sound_x_cfg,
                'sound_y': sound_y_cfg,
                'use_sim_time': True
            }]
        ),

        Node(
            package='alert_manager',
            executable='alert_manager_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='zenoh_bridge',
            executable='zenoh_bridge_node',
            output='screen',
            parameters=[{
                'zenoh_mode': 'peer',
                'zenoh_topic': 'cloud/alerts/whistle',
                'use_sim_time': True
            }]
        ),

        # Demo helper: run script that sets microphone_simulator params (enabled via demo launch arg)
        ExecuteProcess(
            cmd=['python3', os.path.join(gazebo_pkg, 'scripts', 'demo_microphone.py'), sound_x_cfg, sound_y_cfg],
            output='screen',
            condition=IfCondition(demo_cfg)
        ),
    ])

    return ld