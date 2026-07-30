#!/usr/bin/env python3
"""
Sensor-only recording launch file for bag capture.

Only starts:
  1. Livox MID360 driver
  2. RealSense D435i driver (based on sensor_profile)
  3. Static TFs (must be recorded into the bag)

Does NOT start FAST-LIO, RTAB-Map, or any processing nodes.

Usage:
  ros2 launch robot_bringup record_bag.launch.py sensor_profile:=lidar_stereo
  ros2 launch robot_bringup record_bag.launch.py sensor_profile:=lidar_rgbd start_livox:=true
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    start_livox = LaunchConfiguration('start_livox')
    start_realsense = LaunchConfiguration('start_realsense')
    sensor_profile = LaunchConfiguration('sensor_profile')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')

    livox_share = FindPackageShare('livox_ros_driver2')

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('start_livox', default_value='true',
                              description='Start Livox MID360 driver'),
        DeclareLaunchArgument('start_realsense', default_value='true',
                              description='Start RealSense D435i driver'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_stereo',
                              description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint -> base_link'),
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(start_livox),
    )

    # ── 2. RealSense D435i driver ──

    # --- RGBD mode ---
    realsense_rgbd_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[{
            'enable_color': False,
            'enable_depth': True,
            'enable_infra1': True,
            'enable_infra2': False,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': True,
            'align_depth.enable': False,
            'initial_reset': False,
        }],
        condition=IfCondition(PythonExpression([
            "'", start_realsense, "' == 'true' and '",
            sensor_profile, "' == 'lidar_rgbd'"
        ])),
    )

    # --- Stereo mode (infrared, emitter OFF) ---
    realsense_stereo_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[{
            'enable_color': False,
            'enable_depth': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_module.emitter_enabled': 0,  # 关闭红外投影器
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': True,
            'align_depth.enable': False,
            'initial_reset': False,
        }],
        condition=IfCondition(PythonExpression([
            "'", start_realsense, "' == 'true' and '",
            sensor_profile, "' == 'lidar_stereo'"
        ])),
    )

    # --- Mono mode (color only) ---
    realsense_mono_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': True,
            'align_depth.enable': False,
            'initial_reset': False,
        }],
        condition=IfCondition(PythonExpression([
            "'", start_realsense, "' == 'true' and '",
            sensor_profile, "' == 'lidar_mono'"
        ])),
    )

    # ── 3. Static TFs (必须录进 bag，回放时需要) ──
    body_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_footprint',
        arguments=['0', '0', '-1.5', '0', '0', '0', 'body', 'base_footprint'],
        condition=IfCondition(publish_base_link_tf),
    )

    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '1.5', '0', '0', '0', 'base_footprint', 'base_link'],
        condition=IfCondition(publish_base_link_tf),
    )

    livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=['-0.03', '0.0', '-0.10', '0.0', '-0.12', '0.0', 'base_link', 'camera_link'],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' != 'lidar_only'"
        ])),
    )

    # ── Assemble ──
    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(body_tf)
    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    ld.add_action(camera_tf)
    ld.add_action(livox_launch)
    ld.add_action(realsense_rgbd_node)
    ld.add_action(realsense_stereo_node)
    ld.add_action(realsense_mono_node)

    return ld
