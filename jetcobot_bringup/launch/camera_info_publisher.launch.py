#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    camera_info_config_arg = DeclareLaunchArgument(
        'camera_info_config',
        default_value='camera_info_publisher.yaml',
        description='Camera info publisher configuration file'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='jetcocam',
        description='Camera frame ID'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='jetcocam',
        description='Camera name'
    )

    # Configuration file path
    config_file = PathJoinSubstitution([
        FindPackageShare('jetcobot_bringup'),
        'config',
        LaunchConfiguration('camera_info_config')
    ])

    # Camera info publisher node
    camera_info_node = Node(
        package='jetcobot_bringup',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        parameters=[
            config_file,
            {
                'frame_id': LaunchConfiguration('frame_id'),
                'camera_name': LaunchConfiguration('camera_name')
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_info_config_arg,
        frame_id_arg,
        camera_name_arg,
        camera_info_node,
    ])
