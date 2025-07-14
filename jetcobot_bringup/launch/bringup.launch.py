import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    joint_control_node = Node(
        package="jetcobot_bringup",
        executable="joint_control",
        name="joint_control_node",
        output="screen"
    )

    # Include camera_info_publisher launch file
    camera_info_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetcobot_bringup'),
                'launch',
                'camera_info_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'jetcocam',
            'frame_id': 'jetcocam'
        }.items()
    )

    apriltag_ros_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[PathJoinSubstitution([
                FindPackageShare('jetcobot_bringup'), 'config', 'tags_36h11.yaml'])
        ],
        remappings=[
            ('camera_info', '/camera_info'),
        ],
    )

    return LaunchDescription(
        [
            joint_control_node,
            camera_info_launch,
            apriltag_ros_node,
        ]
    )
