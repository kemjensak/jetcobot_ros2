#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Position constants for loadpoints relative to pinky_bag (in meters)
    LOADPOINT_X_OFFSET = 0.01835  # 18.25mm
    LOADPOINT_Y_OFFSET = 0.039     # 39mm
    LOADPOINT_Z_OFFSET = 0.0      # 0mm
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot (e.g., pinky1, pinky2)'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    
    # Create frame names with namespace prefix
    pinky_bag_frame = PythonExpression([
        "'", namespace, "/pinky_bag_projected' if '", namespace, "' != '' else 'pinky_bag_projected'"
    ])
    
    fl_loadpoint_frame = PythonExpression([
        "'", namespace, "/fl' if '", namespace, "' != '' else 'fl_loadpoint'"
    ])
    
    fr_loadpoint_frame = PythonExpression([
        "'", namespace, "/fr' if '", namespace, "' != '' else 'fr_loadpoint'"
    ])
    
    rr_loadpoint_frame = PythonExpression([
        "'", namespace, "/rr' if '", namespace, "' != '' else 'rr_loadpoint'"
    ])
    
    rl_loadpoint_frame = PythonExpression([
        "'", namespace, "/rl' if '", namespace, "' != '' else 'rl_loadpoint'"
    ])

    base_link_frame = PythonExpression([
        "'", namespace, "/base_link' if '", namespace, "' != '' else 'base_link'"
    ])
    
    # Get package share directory
    pkg_share = FindPackageShare('pinky_description')
    
    # Robot description from xacro file
    robot_description = ParameterValue(
        Command([
            'xacro ',
            pkg_share, '/urdf/robot_core.xacro',
            ' namespace:=', namespace
        ])
    )
    
    # Robot state publisher with conditional namespace
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher GUI with conditional namespace
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen'
    )
    
    # Static TF publishers for road points relative to pinky_bag
    # FL: Front Left (+x, +y)
    fl_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fl_loadpoint_tf_publisher',
        namespace=namespace,
        arguments=[str(LOADPOINT_X_OFFSET), str(LOADPOINT_Y_OFFSET), str(LOADPOINT_Z_OFFSET), 
                  '0', '0', '0', pinky_bag_frame, fl_loadpoint_frame]
    )
    
    # FR: Front Right (+x, -y)
    fr_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fr_loadpoint_tf_publisher',
        namespace=namespace,
        arguments=[str(LOADPOINT_X_OFFSET), str(-LOADPOINT_Y_OFFSET), str(LOADPOINT_Z_OFFSET), 
                  '0', '0', '0', pinky_bag_frame, fr_loadpoint_frame]
    )
    
    # RR: Rear Right (-x, -y)
    rr_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rr_loadpoint_tf_publisher',
        namespace=namespace,
        arguments=[str(-LOADPOINT_X_OFFSET), str(-LOADPOINT_Y_OFFSET), str(LOADPOINT_Z_OFFSET), 
                  '0', '0', '0', pinky_bag_frame, rr_loadpoint_frame]
    )
    
    # RL: Rear Left (-x, +y)
    rl_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rl_loadpoint_tf_publisher',
        namespace=namespace,
        arguments=[str(-LOADPOINT_X_OFFSET), str(LOADPOINT_Y_OFFSET), str(LOADPOINT_Z_OFFSET), 
                  '0', '0', '0', pinky_bag_frame, rl_loadpoint_frame]
    )

    pinky_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pinky_base_tf_publisher',
        namespace=namespace,
        arguments=['0.108', '0.000', '-0.018',
                  '0', '0', '0', pinky_bag_frame, base_link_frame]
    )
    
    return LaunchDescription([
        namespace_arg,
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        fl_loadpoint_tf,
        fr_loadpoint_tf,
        rr_loadpoint_tf,
        rl_loadpoint_tf,
        pinky_base_tf,
    ])