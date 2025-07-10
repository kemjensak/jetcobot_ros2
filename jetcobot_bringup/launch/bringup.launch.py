import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    res = []

    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mycobot_280_pi/mycobot_280_pi.urdf"
        )
    )
    res.append(model_launch_arg)
    
    joint_control_node = Node(
        package="jetcobot_bringup",
        executable="joint_control",
        name="joint_control_node",
        output="screen"
    )

    jetcocam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare('jetcobot_bringup'), 'config', 'jetcocam_1_param.yaml'])
        ],
        remappings=[
            ('image_raw', '/jetcocam/image_raw'),
            ('image_raw/compressed', 'jetcocam/image_compressed'),
            ('camera_info', '/jetcocam/camera_info'),
        ],
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
            ('image_rect', '/jetcocam/image_raw'),
            ('camera_info', '/jetcocam/camera_info'),
        ],
    )

    return LaunchDescription(
        [
            joint_control_node,
            jetcocam_node,
            apriltag_ros_node,
        ]
    )
