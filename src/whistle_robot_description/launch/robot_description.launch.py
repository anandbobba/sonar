#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('whistle_robot_description')
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_dir, 'urdf', 'whistle_robot.urdf.xacro')
    
    # Process xacro to URDF
    robot_description = xacro.process_file(xacro_file).toxml()
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Joint State Publisher node (for testing without Gazebo)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
