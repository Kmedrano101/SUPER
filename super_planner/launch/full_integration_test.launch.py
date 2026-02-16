#!/usr/bin/env python3

"""
Full Gazebo Integration Launch File
Launches: SUPER FSM + px4_super_bridge OR px4_mpc_controller + (optionally) mission_planner

Prerequisites (launched separately):
  1. PX4 SITL + Gazebo:  ros2 launch px4_offboard_sim sim.launch.py
  2. FAST-LIO SLAM:      ros2 launch fast_lio_ros2 slam_simulation.launch.py

This launch file handles the planning stack on top of the simulation.

Modes:
  use_mpc:=false (default) — launches px4_super_bridge (position-only control)
  use_mpc:=true            — launches px4_mpc_controller (pos+vel+acc MPC control)

Author: Kevin Medrano Ayala
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, NotSubstitution
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
        mpc_dir = get_package_share_directory('px4_mpc_controller')
        mpc_available = True
    except Exception:
        mpc_available = False
        print("[WARN] px4_mpc_controller package not found - MPC will not be available")

    try:
        mission_dir = get_package_share_directory('mission_planner')
        mission_available = True
    except Exception:
        mission_available = False

    # Configuration paths
    super_config = os.path.join(super_planner_dir, 'config', 'px4_integration.yaml')
    bridge_config = os.path.join(bridge_dir, 'config', 'bridge.yaml') if bridge_available else ''
    mpc_config = os.path.join(mpc_dir, 'config', 'mpc_params.yaml') if mpc_available else ''

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    launch_bridge_arg = DeclareLaunchArgument(
        'launch_bridge', default_value='true',
        description='Launch px4_super_bridge (set false if running separately)')

    use_mpc_arg = DeclareLaunchArgument(
        'use_mpc', default_value='false',
        description='Use MPC controller instead of bridge (pos+vel+acc feedforward)')

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
    ld.add_action(use_mpc_arg)
    ld.add_action(launch_mission_arg)
    ld.add_action(config_arg)

    # SUPER planner
    ld.add_action(super_fsm_node)

    # px4_super_bridge (delayed 3s, after SUPER starts)
    # Only launch if launch_bridge=true AND use_mpc=false
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
            ],
            condition=IfCondition(NotSubstitution(LaunchConfiguration('use_mpc'))),
        )
        ld.add_action(bridge_node)

    # px4_mpc_controller (delayed 3s, replaces bridge when use_mpc=true)
    if mpc_available:
        mpc_node = TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='px4_mpc_controller',
                    executable='mpc_trajectory_node',
                    name='px4_mpc_controller',
                    output='screen',
                    parameters=[
                        mpc_config,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                )
            ],
            condition=IfCondition(LaunchConfiguration('use_mpc')),
        )
        ld.add_action(mpc_node)

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
