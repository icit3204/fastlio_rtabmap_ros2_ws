#!/usr/bin/env python3
"""Live dual-LiDAR autonomy/operator profile with hard-locked mock actuation."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_bringup = FindPackageShare('robot_bringup')
    operator_gui = FindPackageShare('operator_gui')
    robot_visualization = FindPackageShare('robot_visualization')

    canonical_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_bringup, 'launch', 'bringup.launch.py'])),
        launch_arguments={
            'mode': 'navigation',
            'start_livox': 'true',
            'start_ydlidar': 'true',
            'use_fast_lio': 'true',
            'start_rtabmap': 'true',
            'enable_rviz': 'false',
            'stationary_mission_manager_mock_gate': 'true',
            'mission_manager_expected_topology_version':
                'sha256:f8bd2b688ce827446ea88f42f19777f97c918c2dc1832091fa9b379a85000d70',
            'stationary_real_localization_validity': 'true',
            'rtabmap_use_working_copy': 'true',
        }.items(),
    )

    live_operator_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([operator_gui, 'launch', 'operator_gui.launch.py'])),
        launch_arguments={'gate_state_topic': '/phase5/gate_test/state'}.items(),
    )

    live_navigation_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                robot_visualization, 'launch', 'navigation_visualization.launch.py'])),
        launch_arguments={'use_fixture': 'false', 'use_rviz': 'true'}.items(),
    )

    return LaunchDescription([
        canonical_stack,
        live_operator_gui,
        live_navigation_visualization,
    ])
