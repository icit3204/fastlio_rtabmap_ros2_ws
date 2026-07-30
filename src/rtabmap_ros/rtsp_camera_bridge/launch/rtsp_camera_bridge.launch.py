#!/usr/bin/env python3
"""Launch only the RTSP camera bridge node.

Usage:
  ros2 launch rtsp_camera_bridge rtsp_camera_bridge.launch.py \
    rtsp_url:='rtsp://user:pass%40@host:554/stream' \
    image_topic:=/sensors/camera/rgb/image_rect \
    camera_info_topic:=/sensors/camera/rgb/camera_info
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('rtsp_url', default_value='', description='RTSP URL (password @ -> %40)'),
        DeclareLaunchArgument('gstreamer_pipeline', default_value=''),
        DeclareLaunchArgument('backend', default_value='ffmpeg'),
        DeclareLaunchArgument('rtsp_transport', default_value='tcp'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('target_fps', default_value='0.0'),
        DeclareLaunchArgument('width', default_value='0'),
        DeclareLaunchArgument('height', default_value='0'),
        DeclareLaunchArgument('force_mono', default_value='false'),
        DeclareLaunchArgument('focal_length_px', default_value='0.0'),
        DeclareLaunchArgument('image_topic', default_value='image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='camera_info'),
    ]
    node = Node(
        package='rtsp_camera_bridge',
        executable='rtsp_camera_bridge.py',
        name='rtsp_camera_bridge',
        output='screen',
        parameters=[{
            'rtsp_url': LaunchConfiguration('rtsp_url'),
            'gstreamer_pipeline': LaunchConfiguration('gstreamer_pipeline'),
            'backend': LaunchConfiguration('backend'),
            'rtsp_transport': LaunchConfiguration('rtsp_transport'),
            'frame_id': LaunchConfiguration('frame_id'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'target_fps': LaunchConfiguration('target_fps'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'force_mono': LaunchConfiguration('force_mono'),
            'focal_length_px': LaunchConfiguration('focal_length_px'),
        }],
        remappings=[
            ('image_raw', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
        ],
    )
    ld = LaunchDescription(args)
    ld.add_action(node)
    return ld
