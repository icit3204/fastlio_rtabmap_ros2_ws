#!/usr/bin/env python3
"""RTAB-Map bridge starter.

Suggested location:
  robot_bringup/launch/rtabmap_bridge.launch.py

This wrapper keeps RTAB-Map focused on global SLAM, loop closure, mapping, and
localization, while local continuous odometry comes from FAST-LIO.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_only', description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('enable_gps', default_value='false'),
        DeclareLaunchArgument('localization', default_value='true', description='false=mapping, true=localization/navigation'),
        DeclareLaunchArgument(
            'database_path',
            default_value=EnvironmentVariable('PARKING_ROBOT_RTABMAP_DATABASE', default_value=''),
            description='RTAB-Map database path. Explicit launch argument overrides PARKING_ROBOT_RTABMAP_DATABASE.',
        ),
        DeclareLaunchArgument('rtabmap_args', default_value=''),
        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('imu_topic', default_value='/unused_imu'),
        DeclareLaunchArgument('gps_topic', default_value='/sensors/gps/fix'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('rgb_topic', default_value='/sensors/camera/rgb/image_rect'),
        DeclareLaunchArgument('depth_topic', default_value='/sensors/camera/depth/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/sensors/camera/rgb/camera_info'),
        DeclareLaunchArgument('left_image_topic', default_value='/sensors/camera/left/image_rect'),
        DeclareLaunchArgument('right_image_topic', default_value='/sensors/camera/right/image_rect'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/sensors/camera/left/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/sensors/camera/right/camera_info'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false'),
        DeclareLaunchArgument('delete_db_on_start', default_value='false'),
    ]

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'localization': LaunchConfiguration('localization'),
            'database_path': LaunchConfiguration('database_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'publish_tf_map': 'true',
            'odom_topic': LaunchConfiguration('odom_topic'),
            'publish_tf_odom': 'false',
            'imu_topic': LaunchConfiguration('imu_topic'),
            'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
            'gps_topic': LaunchConfiguration('gps_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'subscribe_scan_cloud': 'true',
            'subscribe_scan': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'approx_sync': 'true',
            'qos': '2',
            'latch': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
            'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
            # === 禁用 rgbd_sync，绕过合成节点 ===
            # 作用：控制是否启动 rtabmap_sync/rgbd_sync（或 stereo_sync）节点，把分开的 RGB + Depth 图像在内部同步成 rgbd_image 格式。
            # true：启动内部同步节点，由 RTAB-Map 自己把 rgb_topic + depth_topic + camera_info 打包成 rgbd_image
            # false：不启动内部同步节点
            # 'rgbd_sync': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' else 'false'"]),
            'rgbd_sync': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            # subscribe_rgbd — 是否订阅已同步的 rgbd_image
            # 作用：控制 RTAB-Map 主节点（slam/viz/odom）订阅的是分开的图像话题，还是已经打包好的 rgbd_image 话题。
            # true：rtabmap 订阅 rgbd_image（统一格式的同步数据），不再单独订阅 rgb/depth
            # false：rtabmap 分别订阅 rgb/image 和 depth/image
            # 'subscribe_rgbd': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' else 'false'"]),
            'subscribe_rgbd': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            # 作用：仅当 rgbd_sync=true 时生效，控制 rtabmap_sync/rgbd_sync 节点使用近似时间同步还是精确时间同步。
            # true（默认）：approx_sync=true，允许时间戳有一定偏差（适合真实传感器，不同话题到达时间有抖动）
            # false：approx_sync=false，要求时间戳严格一致（适合 rosbag 或仿真环境）
            # 'approx_rgbd_sync': 'false',  #内部合成节点是否近似同步
           
            'stereo': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            'depth': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' else 'false'"]),
            'subscribe_rgb': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_mono' else 'false'"]),
            'args': [
                PythonExpression(["'-d ' if '", LaunchConfiguration('delete_db_on_start'), "' == 'true' else ''"]),
                LaunchConfiguration('rtabmap_args'),
            ],
        }.items(),
    )

    ld = LaunchDescription()

    _colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    if _colcon_prefix:
        _ws_root = os.path.dirname(_colcon_prefix.split(':')[0])
        _rtabmap_lib = os.path.join(_ws_root, 'third_party', 'rtabmap-0.23.4', 'install', 'lib')
        _existing_ldpath = os.environ.get('LD_LIBRARY_PATH', '')
        _extra_libs_list = [_rtabmap_lib]
        _torch_lib = os.environ.get('RTABMAP_TORCH_LIB_DIR', '')
        if _torch_lib:
            if os.path.isdir(_torch_lib):
                if _torch_lib not in _extra_libs_list and _torch_lib not in _existing_ldpath.split(':'):
                    _extra_libs_list.append(_torch_lib)
            else:
                print(f'[robot_bringup] Ignoring invalid RTABMAP_TORCH_LIB_DIR: {_torch_lib}')
        _extra_libs = ':'.join(_extra_libs_list)
        ld.add_action(SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            _extra_libs + ':' + _existing_ldpath if _existing_ldpath else _extra_libs,
        ))

    for action in declare_args:
        ld.add_action(action)
    ld.add_action(rtabmap_launch)
    return ld
