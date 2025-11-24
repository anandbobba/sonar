#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # Get package directories
    gazebo_pkg = get_package_share_directory('whistle_gazebo')
    robot_desc_pkg = get_package_share_directory('whistle_robot_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    # World file
    world_file = os.path.join(gazebo_pkg, 'worlds', 'whistle_test.world')
    
    # Robot URDF
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'whistle_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_whistle_robot',
        arguments=[
            '-entity', 'whistle_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot
    ])
