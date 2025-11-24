#!/usr/bin/env python3
"""
Adaptive Sound Follower Launch File
Robot behavior adapts based on sound strength:
- Very weak sound (0.15-0.3): Robot moves VERY SLOWLY
- Weak sound (0.3-0.6): Robot moves SLOWLY
- Strong sound (>0.6): Robot moves FAST
- Very strong sound (>0.8): Robot STOPS (arrived)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    declare_sound_x = DeclareLaunchArgument('sound_x', default_value='3.0', description='X coordinate of sound source')
    declare_sound_y = DeclareLaunchArgument('sound_y', default_value='3.0', description='Y coordinate of sound source')
    
    # Sound strength thresholds
    declare_activation = DeclareLaunchArgument('activation_threshold', default_value='0.15', description='Minimum to start moving')
    declare_weak = DeclareLaunchArgument('weak_threshold', default_value='0.3', description='Below this = very slow')
    declare_moderate = DeclareLaunchArgument('moderate_threshold', default_value='0.6', description='Above this = normal speed')
    declare_stop = DeclareLaunchArgument('stop_threshold', default_value='0.8', description='Stop when reached')
    
    # Speed settings
    declare_weak_speed = DeclareLaunchArgument('weak_speed', default_value='0.1', description='Speed for very weak sound')
    declare_moderate_speed = DeclareLaunchArgument('moderate_speed', default_value='0.3', description='Speed for moderate sound')
    declare_strong_speed = DeclareLaunchArgument('strong_speed', default_value='0.5', description='Speed for strong sound')

    sound_x_cfg = LaunchConfiguration('sound_x')
    sound_y_cfg = LaunchConfiguration('sound_y')
    activation_cfg = LaunchConfiguration('activation_threshold')
    weak_cfg = LaunchConfiguration('weak_threshold')
    moderate_cfg = LaunchConfiguration('moderate_threshold')
    stop_cfg = LaunchConfiguration('stop_threshold')
    weak_speed_cfg = LaunchConfiguration('weak_speed')
    moderate_speed_cfg = LaunchConfiguration('moderate_speed')
    strong_speed_cfg = LaunchConfiguration('strong_speed')

    ld = LaunchDescription([
        declare_sound_x,
        declare_sound_y,
        declare_activation,
        declare_weak,
        declare_moderate,
        declare_stop,
        declare_weak_speed,
        declare_moderate_speed,
        declare_strong_speed,

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file}.items()
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'whistle_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        ),

        # Microphone simulator
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

        # Whistle detector
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

        # Adaptive autonomous responder
        Node(
            package='whistle_detector',
            executable='autonomous_responder',
            output='screen',
            parameters=[{
                'activation_threshold': activation_cfg,
                'weak_threshold': weak_cfg,
                'moderate_threshold': moderate_cfg,
                'stop_threshold': stop_cfg,
                'weak_speed': weak_speed_cfg,
                'moderate_speed': moderate_speed_cfg,
                'strong_speed': strong_speed_cfg,
                'sound_x': sound_x_cfg,
                'sound_y': sound_y_cfg,
                'audio_alpha': 0.6,
                'stop_delay': 0.5,
                'use_sim_time': True
            }]
        ),

        # Alert manager
        Node(
            package='alert_manager',
            executable='alert_manager_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Zenoh bridge
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

        # Demo helper: automatically enables virtual whistle at specified location
        ExecuteProcess(
            cmd=['python3', os.path.join(gazebo_pkg, 'scripts', 'demo_microphone.py'), sound_x_cfg, sound_y_cfg],
            output='screen'
        ),
    ])

    return ld
