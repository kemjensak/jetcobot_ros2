#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/jetcocam0',
        description='Camera device path or index (e.g., "/dev/video0" or "0")'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='tags_36h11.yaml',
        description='Configuration file name'
    )

    # Configuration file path
    config_file = PathJoinSubstitution([
        FindPackageShare('apriltag_ros'),
        'cfg',
        LaunchConfiguration('config_file')
    ])

    # AprilTag node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        parameters=[
            config_file,
            {
                'camera_device': LaunchConfiguration('camera_device')
            }
        ],
        output='screen'
    )

    # Camera info publisher (you may need to adjust this based on your camera setup)
    camera_info_node = Node(
        package='camera_info_manager',
        executable='camera_info_manager',
        name='camera_info_publisher',
        namespace='camera',
        parameters=[
            {'camera_name': 'jetcocam'},
            {'camera_info_url': 'file:///path/to/your/camera_calibration.yaml'}  # Update this path
        ],
        remappings=[
            ('camera_info', '/camera_info')
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_device_arg,
        config_file_arg,
        apriltag_node,
        # camera_info_node,  # Uncomment if you need to publish camera_info
    ])
