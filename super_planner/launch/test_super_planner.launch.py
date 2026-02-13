#!/usr/bin/env python3

"""
Standalone SUPER Planner Launch File
Launches only the fsm_node for testing with FAST-LIO + PX4.

Prerequisites (launched separately):
  1. PX4 SITL + Gazebo:  ros2 launch px4_offboard_sim sim.launch.py
  2. FAST-LIO SLAM:      ros2 launch fast_lio_ros2 slam_simulation.launch.py

Usage:
  ros2 launch super_planner test_super_planner.launch.py
  ros2 launch super_planner test_super_planner.launch.py config_file:=/path/to/config.yaml

Author: Kevin Medrano Ayala
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    super_planner_dir = get_package_share_directory('super_planner')

    default_config_file = os.path.join(
        super_planner_dir, 'config', 'px4_integration.yaml'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to SUPER configuration YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for SUPER visualization'
    )

    # SUPER FSM Node
    super_fsm_node = Node(
        package='super_planner',
        executable='fsm_node',
        name='super_fsm',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )

    # RViz Node (optional)
    rviz_config_file = os.path.join(super_planner_dir, 'rviz', 'super_integration.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_super',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        rviz_arg,
        super_fsm_node,
        rviz_node,
    ])
