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
    
    # Define pinky namespaces
    pinky_namespaces = ['pinky1', 'pinky2', 'pinky3']
    
    # Get package share directory
    pkg_share = FindPackageShare('pinky_description')
    
    # List to store all nodes
    nodes = []
    
    # Create nodes for each pinky namespace
    for namespace in pinky_namespaces:
        # Create frame names with namespace prefix
        pinky_bag_frame = f"{namespace}/pinky_bag_projected"
        fl_loadpoint_frame = f"{namespace}/fl"
        fr_loadpoint_frame = f"{namespace}/fr"
        rr_loadpoint_frame = f"{namespace}/rr"
        rl_loadpoint_frame = f"{namespace}/rl"
        base_link_frame = f"{namespace}/base_footprint"
        
        # Robot description from xacro file
        robot_description = ParameterValue(
            Command([
                'xacro ',
                pkg_share, '/urdf/robot_core.xacro',
                f' namespace:={namespace}'
            ])
        )
        
        # Robot state publisher with namespace
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
        
        # Static TF publishers for road points relative to pinky_bag
        # FL: Front Left (+x, +y)
        fl_loadpoint_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fl_loadpoint_tf_publisher',
            namespace=namespace,
            arguments=['--x', str(LOADPOINT_X_OFFSET), '--y', str(LOADPOINT_Y_OFFSET), '--z', str(LOADPOINT_Z_OFFSET),
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', pinky_bag_frame, '--child-frame-id', fl_loadpoint_frame]
        )
        
        # FR: Front Right (+x, -y)
        fr_loadpoint_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fr_loadpoint_tf_publisher',
            namespace=namespace,
            arguments=['--x', str(LOADPOINT_X_OFFSET), '--y', str(-LOADPOINT_Y_OFFSET), '--z', str(LOADPOINT_Z_OFFSET),
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', pinky_bag_frame, '--child-frame-id', fr_loadpoint_frame]
        )
        
        # RR: Rear Right (-x, -y)
        rr_loadpoint_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rr_loadpoint_tf_publisher',
            namespace=namespace,
            arguments=['--x', str(-LOADPOINT_X_OFFSET), '--y', str(-LOADPOINT_Y_OFFSET), '--z', str(LOADPOINT_Z_OFFSET),
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', pinky_bag_frame, '--child-frame-id', rr_loadpoint_frame]
        )
        
        # RL: Rear Left (-x, +y)
        rl_loadpoint_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rl_loadpoint_tf_publisher',
            namespace=namespace,
            arguments=['--x', str(-LOADPOINT_X_OFFSET), '--y', str(LOADPOINT_Y_OFFSET), '--z', str(LOADPOINT_Z_OFFSET),
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', pinky_bag_frame, '--child-frame-id', rl_loadpoint_frame]
        )

        # Pinky base TF publisher
        pinky_base_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pinky_base_tf_publisher',
            namespace=namespace,
            arguments=['--x', '0.108', '--y', '0.000', '--z', '-0.048',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', pinky_bag_frame, '--child-frame-id', base_link_frame]
        )
        
        # Add all nodes for this namespace to the list
        nodes.extend([
            robot_state_publisher_node,
            fl_loadpoint_tf,
            fr_loadpoint_tf,
            rr_loadpoint_tf,
            rl_loadpoint_tf,
            pinky_base_tf,
        ])
    
    return LaunchDescription(nodes)