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

    joint_state_switcher_node = Node(
        package="jetcobot_bringup",
        executable="joint_state_switcher",
        name="joint_state_switcher_node",
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
            'camera_name': 'ov3360',
            'frame_id': 'ov3360'
        }.items()
    )

    # Include jetcobot_moveit_config demo.launch.py with use_rviz=false
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetcobot_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': 'false'
        }.items()
    )

    apriltag_ros_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[PathJoinSubstitution([
                FindPackageShare('apriltag_ros'), 'cfg', 'tags_41h12.yaml'])
        ],
        remappings=[
            ('camera_info', '/camera_info'),
        ],
    )

    pinky_tf_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetcobot_bringup'),
                'launch',
                'pinky_tf_publisher.launch.py'
            ])
        ])
    )

    ground_tf_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetcobot_moveit_picker'),
                'launch',
                'loadpoint_publisher.launch.py'
            ])
        ])
    )

    return LaunchDescription(
        [
            joint_control_node,
            joint_state_switcher_node,
            camera_info_launch,
            moveit_demo_launch,
            apriltag_ros_node,
            pinky_tf_publisher,
            ground_tf_publisher
        ]
    )
