#!/usr/bin/env python3

"""
Test Launch File for SUPER Planner Integration
Tests SUPER with FAST-LIO and PX4 system

Author: Kevin Medrano Ayala
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    super_planner_dir = get_package_share_directory('super_planner')

    # Default configuration file (installed location)
    default_config_file = os.path.join(
        super_planner_dir, 'config', 'px4_integration.yaml'
    )

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to SUPER configuration YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
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
        # Topic remappings (if needed)
        remappings=[
            # Uncomment if your odometry topic name is different
            # ('/Odometry', '/your_odom_topic'),
        ],
        emulate_tty=True,
        prefix='bash -c "sleep 2; $0 $@"'  # Wait 2s for other nodes to start
    )

    # RViz Node (optional) - Using super_integration.rviz (fixed for ROS2)
    rviz_config_file = os.path.join(
        super_planner_dir, 'rviz', 'super_integration.rviz'
    )

    # Create a proper launch condition
    from launch.conditions import IfCondition

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
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
