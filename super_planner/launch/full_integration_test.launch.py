#!/usr/bin/env python3

"""
Full Integration Test Launch File
Launches: FAST-LIO + px4_offboard_sim + SUPER + px4_super_bridge

Prerequisites:
  - Gazebo + PX4 SITL running separately
  - All packages built and sourced

Author: Kevin Medrano Ayala
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    super_planner_dir = get_package_share_directory('super_planner')

    # Try to get other package directories (may not exist)
    try:
        px4_offboard_dir = get_package_share_directory('px4_offboard_sim')
        px4_offboard_available = True
    except:
        px4_offboard_available = False
        print("[WARN] px4_offboard_sim package not found")

    try:
        px4_bridge_dir = get_package_share_directory('px4_super_bridge')
        px4_bridge_available = True
    except:
        px4_bridge_available = False
        print("[WARN] px4_super_bridge package not found")

    # Configuration file paths (installed location)
    super_config = os.path.join(
        super_planner_dir, 'config', 'px4_integration.yaml'
    )

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    launch_bridge_arg = DeclareLaunchArgument(
        'launch_bridge',
        default_value='true' if px4_bridge_available else 'false',
        description='Launch px4_super_bridge'
    )

    launch_offboard_arg = DeclareLaunchArgument(
        'launch_offboard',
        default_value='true' if px4_offboard_available else 'false',
        description='Launch px4_offboard_sim (FAST-LIO included)'
    )

    # SUPER FSM Node (delayed start)
    super_fsm_node = TimerAction(
        period=3.0,  # Wait 3 seconds for FAST-LIO to start
        actions=[
            Node(
                package='super_planner',
                executable='fsm_node',
                name='super_fsm',
                output='screen',
                parameters=[
                    super_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                emulate_tty=True,
            )
        ]
    )

    # px4_super_bridge Node (delayed start)
    bridge_node = TimerAction(
        period=5.0,  # Wait 5 seconds for SUPER to initialize
        actions=[
            Node(
                package='px4_super_bridge',
                executable='bridge_node',
                name='px4_super_bridge',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ]
            )
        ]
    ) if px4_bridge_available else None

    # px4_offboard_sim + FAST-LIO Launch (if available)
    offboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(px4_offboard_dir, 'launch', 'slam_simulation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    ) if px4_offboard_available else None

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Build launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_bridge_arg)
    ld.add_action(launch_offboard_arg)

    # Add nodes conditionally
    if px4_offboard_available:
        ld.add_action(offboard_launch)

    ld.add_action(super_fsm_node)

    if px4_bridge_available and bridge_node:
        ld.add_action(bridge_node)

    ld.add_action(rviz_node)

    return ld
