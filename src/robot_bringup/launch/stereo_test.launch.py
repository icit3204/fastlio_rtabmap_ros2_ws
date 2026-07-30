#!/usr/bin/env python3
"""
Minimal D435i stereo SLAM test — no IMU required.

Usage:
  ros2 launch robot_bringup stereo_test.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    parameters = {
        'frame_id': 'camera_link',
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'wait_imu_to_init': False,
    }

    remappings = [
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info'),
    ]

    return LaunchDescription([

        # Disable IR emitter for clean stereo matching
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # D435i camera driver — infrared stereo only, no color/depth/IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
            launch_arguments={
                'camera_namespace': '',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_color': 'false',
                'enable_depth': 'false',
                'enable_sync': 'true',
                'depth_module.infra_profile': '848x480x30',
            }.items(),
        ),

        # Stereo visual odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings,
        ),

        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d'],
        ),

        # RTAB-Map viz
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters,
                        {'odometry_node_name': 'stereo_odometry'}],
            remappings=remappings,
        ),
    ])
