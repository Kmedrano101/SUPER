#!/usr/bin/env python3

"""
Full Gazebo Integration Launch File
Launches: SUPER FSM + px4_super_bridge + (optionally) mission_planner

Prerequisites (launched separately):
  1. PX4 SITL + Gazebo:  ros2 launch px4_offboard_sim sim.launch.py
  2. FAST-LIO SLAM:      ros2 launch fast_lio_ros2 slam_simulation.launch.py

This launch file handles the planning stack on top of the simulation.

Author: Kevin Medrano Ayala
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    super_planner_dir = get_package_share_directory('super_planner')

    try:
        bridge_dir = get_package_share_directory('px4_super_bridge')
        bridge_available = True
    except Exception:
        bridge_available = False
        print("[WARN] px4_super_bridge package not found - bridge will not be launched")

    try:
        mission_dir = get_package_share_directory('mission_planner')
        mission_available = True
    except Exception:
        mission_available = False

    # Configuration paths
    super_config = os.path.join(super_planner_dir, 'config', 'px4_integration.yaml')
    bridge_config = os.path.join(bridge_dir, 'config', 'bridge.yaml') if bridge_available else ''

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    launch_bridge_arg = DeclareLaunchArgument(
        'launch_bridge', default_value='true',
        description='Launch px4_super_bridge (set false if running separately)')

    launch_mission_arg = DeclareLaunchArgument(
        'launch_mission', default_value='false',
        description='Launch mission_planner for waypoint missions')

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=super_config,
        description='SUPER planner configuration file')

    # SUPER FSM Node (delayed 2s for FAST-LIO to initialize)
    super_fsm_node = TimerAction(
        period=2.0,
        actions=[
            Node(
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
        ]
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_bridge_arg)
    ld.add_action(launch_mission_arg)
    ld.add_action(config_arg)

    # SUPER planner
    ld.add_action(super_fsm_node)

    # px4_super_bridge (delayed 3s, after SUPER starts)
    if bridge_available:
        bridge_node = TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='px4_super_bridge',
                    executable='bridge_node',
                    name='px4_super_bridge',
                    output='screen',
                    parameters=[
                        bridge_config,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    condition=IfCondition(LaunchConfiguration('launch_bridge')),
                )
            ]
        )
        ld.add_action(bridge_node)

    # mission_planner (optional, delayed 4s)
    if mission_available:
        mission_node = TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='mission_planner',
                    executable='waypoint_mission',
                    name='waypoint_mission',
                    output='screen',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    condition=IfCondition(LaunchConfiguration('launch_mission')),
                )
            ]
        )
        ld.add_action(mission_node)

    return ld
