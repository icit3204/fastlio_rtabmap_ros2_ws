#!/usr/bin/env python3
"""Offline navigation with obstacle avoidance — no hardware needed.

Loads a 2D grid map (with obstacles baked in), displays the original
mapping trajectory, sends the robot along the path via Nav2, and
MPPI controller handles local obstacle avoidance automatically.

Usage — clean map (no obstacles):
  ros2 launch robot_bringup offline_avoidance.launch.py \
    map_yaml:=.../scripts/offline_nav_maps/clean_map.yaml

Usage — obstacle map:
  ros2 launch robot_bringup offline_avoidance.launch.py \
    map_yaml:=.../scripts/offline_nav_maps/obstacle_map.yaml \
    path_yaml:=.../path_waypoints.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    # ── Launch arguments ──
    map_yaml = LaunchConfiguration('map_yaml')
    path_yaml = LaunchConfiguration('path_yaml')
    database_path = LaunchConfiguration('database_path')
    enable_rviz = LaunchConfiguration('enable_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    obstacles_yaml = LaunchConfiguration('obstacles_yaml')
    show_obstacles = LaunchConfiguration('show_obstacles')
    enable_clicked_obstacles = LaunchConfiguration('enable_clicked_obstacles')
    robot_initial_x = LaunchConfiguration('robot_initial_x')
    robot_initial_y = LaunchConfiguration('robot_initial_y')
    robot_initial_yaw = LaunchConfiguration('robot_initial_yaw')

    declare_args = [
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([robot_bringup_share, '..', '..', '..', '..',
                                               'scripts', 'offline_nav_maps', 'obstacle_map.yaml']),
            description='Path to map YAML file for map_server'),
        DeclareLaunchArgument(
            'path_yaml',
            default_value=EnvironmentVariable('PARKING_ROBOT_WAYPOINTS_FILE', default_value=''),
            description='Path to waypoints YAML extracted from database. Explicit launch argument overrides PARKING_ROBOT_WAYPOINTS_FILE.'),
        DeclareLaunchArgument(
            'database_path',
            default_value=EnvironmentVariable('PARKING_ROBOT_OFFLINE_PATH_DATABASE', default_value=''),
            description='RTAB-Map database for extracting mapping trajectory. Explicit launch argument overrides PARKING_ROBOT_OFFLINE_PATH_DATABASE.'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_offline.yaml']),
            description='Nav2 params file for offline navigation'),
        DeclareLaunchArgument(
            'obstacles_yaml',
            default_value='/tmp/obstacles.yaml',
            description='Obstacle YAML for RViz red marker visualization'),
        DeclareLaunchArgument('show_obstacles', default_value='true'),
        DeclareLaunchArgument('enable_clicked_obstacles', default_value='true'),
        DeclareLaunchArgument('robot_initial_x', default_value='0.0',
                              description='Robot initial X (map frame) — overridden by waypoint sender'),
        DeclareLaunchArgument('robot_initial_y', default_value='0.0'),
        DeclareLaunchArgument('robot_initial_yaw', default_value='0.0'),
    ]

    # ── 1. map_server — loads 2D occupancy grid ──
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
            'bond_timeout': 20.0,
        }],
    )

    # ── 2. Static TFs ──
    # map → odom: identity (fixed, since there's no localization)
    map_to_odom_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='offline_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
    )
    # base_footprint → base_link: identity
    base_fp_to_base_link_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='offline_base_footprint_to_base_link',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
    )

    # ── 3. Fake odometry (integrates /cmd_vel_nav → /Odometry + odom→base_footprint TF) ──
    odom_sim = Node(
        package='robot_bringup',
        executable='odom_from_cmd_vel.py',
        name='odom_from_cmd_vel',
        output='screen',
        parameters=[{
            'initial_x': robot_initial_x,
            'initial_y': robot_initial_y,
            'initial_yaw': robot_initial_yaw,
        }],
    )

    # ── 4. Path publisher (publishes /mapping_path for RViz trajectory display) ──
    path_pub = Node(
        package='robot_bringup',
        executable='path_publisher.py',
        name='path_publisher',
        output='screen',
        parameters=[{
            'database_path': database_path,
            'path_yaml': path_yaml,
            'frame_id': 'map',
            'publish_rate': 1.0,
        }],
    )

    obstacle_markers = Node(
        package='robot_bringup',
        executable='obstacle_marker_publisher.py',
        name='obstacle_marker_publisher',
        output='screen',
        condition=IfCondition(show_obstacles),
        parameters=[{
            'input': obstacles_yaml,
            'frame_id': 'map',
            'publish_rate': 1.0,
        }],
    )

    robot_marker = Node(
        package='robot_bringup',
        executable='robot_marker_publisher.py',
        name='robot_marker_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
        }],
    )

    clicked_obstacles = Node(
        package='robot_bringup',
        executable='clicked_obstacle_publisher.py',
        name='clicked_obstacle_publisher',
        output='screen',
        condition=IfCondition(enable_clicked_obstacles),
        parameters=[{
            'frame_id': 'map',
            'obstacle_width': 1.6,
            'obstacle_height': 1.6,
            'obstacle_z': 0.5,
        }],
    )

    # ── 5. Nav2 navigation stack ──
    nav2_launch = GroupAction(
        actions=[
            SetRemap('/cmd_vel', '/cmd_vel_nav'),
            SetRemap('/goal_pose', '/nav2_direct_goal_pose_disabled'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': nav2_params_file,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items(),
            ),
        ],
    )

    # ── 6. Waypoint sender (sets initial pose + sends path to Nav2) ──
    # Delayed to let Nav2 nodes fully start
    waypoint_sender = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='robot_bringup',
                executable='path_waypoint_sender.py',
                name='path_waypoint_sender',
                output='screen',
                parameters=[{
                    'input': path_yaml,
                    'frame_id': 'map',
                    'map_yaml': map_yaml,
                    'auto_goal_distance': 8.0,
                    'auto_replan_on_clicked_obstacle': True,
                    'clicked_obstacle_topic': '/clicked_point',
                    'auto_replan_delay': 0.8,
                    'clicked_obstacle_width': 1.6,
                    'clicked_obstacle_height': 1.6,
                    'dynamic_goal_clearance': 0.8,
                }],
            ),
        ],
    )

    # ── 7. RViz ──
    rviz_config = PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_navigation.rviz'])
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(enable_rviz),
        arguments=['-d', rviz_config],
    )

    # ── Assemble ──
    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(map_server_node)
    ld.add_action(map_server_lifecycle)
    ld.add_action(map_to_odom_tf)
    ld.add_action(base_fp_to_base_link_tf)
    ld.add_action(odom_sim)
    ld.add_action(path_pub)
    ld.add_action(obstacle_markers)
    ld.add_action(robot_marker)
    ld.add_action(clicked_obstacles)
    ld.add_action(TimerAction(period=8.0, actions=[nav2_launch]))
    ld.add_action(waypoint_sender)
    ld.add_action(rviz_node)
    return ld
