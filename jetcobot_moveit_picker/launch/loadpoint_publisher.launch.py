#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Loadpoint offset constants
    LOADPOINT_X_OFFSET = 0.039  # 39mm
    LOADPOINT_Y_OFFSET = 0.01835    # 18.35mm

    ground_right_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ground_right_loadpoint_tf_publisher',
        arguments=['--x', '0', '--y', '-0.205', '--z', '0', 
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'world', '--child-frame-id', 'ground_right']
    )

    ground_left_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ground_left_loadpoint_tf_publisher',
        arguments=['--x', '0', '--y', '0.205', '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'world', '--child-frame-id', 'ground_left']
    )

    # Left side loadpoints (4 total: rr, rl, fl, fr)
    left_fr_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_fr_loadpoint_tf_publisher',
        arguments=['--x', str(LOADPOINT_X_OFFSET), '--y', str(-LOADPOINT_Y_OFFSET), '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'ground_left', '--child-frame-id', 'ground_left/fr']
    )

    left_fl_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_fl_loadpoint_tf_publisher',
        arguments=['--x', str(LOADPOINT_X_OFFSET), '--y', str(LOADPOINT_Y_OFFSET), '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'ground_left', '--child-frame-id', 'ground_left/fl']
    )

    left_rl_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_rl_loadpoint_tf_publisher',
        arguments=['--x', str(-LOADPOINT_X_OFFSET), '--y', str(LOADPOINT_Y_OFFSET), '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'ground_left', '--child-frame-id', 'ground_left/rl']
    )

    left_rr_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_rr_loadpoint_tf_publisher',
        arguments=['--x', str(-LOADPOINT_X_OFFSET), '--y', str(-LOADPOINT_Y_OFFSET), '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'ground_left', '--child-frame-id', 'ground_left/rr']
    )

    # Right side loadpoints (4 total: rr, rl, fl, fr)
    right_fr_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_fr_loadpoint_tf_publisher',
        arguments=['--x', str(LOADPOINT_X_OFFSET), '--y', str(-LOADPOINT_Y_OFFSET), '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'ground_right', '--child-frame-id', 'ground_right/fr']
    )

    right_fl_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_fl_loadpoint_tf_publisher',
        arguments=['--x', str(LOADPOINT_X_OFFSET), '--y', str(LOADPOINT_Y_OFFSET), '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'ground_right', '--child-frame-id', 'ground_right/fl']
    )

    right_rl_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_rl_loadpoint_tf_publisher',
        arguments=['--x', str(-LOADPOINT_X_OFFSET), '--y', str(LOADPOINT_Y_OFFSET), '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'ground_right', '--child-frame-id', 'ground_right/rl']
    )

    right_rr_loadpoint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_rr_loadpoint_tf_publisher',
        arguments=['--x', str(-LOADPOINT_X_OFFSET), '--y', str(-LOADPOINT_Y_OFFSET), '--z', '0',
                  '--roll', '0', '--pitch', '0', '--yaw', '0',
                  '--frame-id', 'ground_right', '--child-frame-id', 'ground_right/rr']
    )

    return LaunchDescription([
        ground_right_loadpoint_tf,
        ground_left_loadpoint_tf,
        left_rr_loadpoint_tf,
        left_rl_loadpoint_tf,
        left_fl_loadpoint_tf,
        left_fr_loadpoint_tf,
        right_rr_loadpoint_tf,
        right_rl_loadpoint_tf,
        right_fl_loadpoint_tf,
        right_fr_loadpoint_tf
    ])