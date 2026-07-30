#!/usr/bin/env python3
"""
Main mapping bringup launch file using FAST-LIO.

Starts:
  1. Livox MID360 lidar driver
  2. FAST-LIO odometry -> /Odometry (and odom->base_footprint TF)
  3. RTAB-Map SLAM (consumes /Odometry)
  4. Static TFs

Usage:
  ros2 launch robot_bringup fastlio_mapping.launch.py
  ros2 launch robot_bringup fastlio_mapping.launch.py start_livox:=false rviz:=true
"""

import math
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox = LaunchConfiguration('start_livox')
    sensor_profile = LaunchConfiguration('sensor_profile')
    start_realsense = LaunchConfiguration('start_realsense')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')

    livox_share = FindPackageShare('livox_ros_driver2')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')
    fast_lio_share = FindPackageShare('fast_lio')

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_stereo', description='lidar_only | lidar_rgbd  | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('start_livox', default_value='false',
                              description='Start Livox MID360 driver'),
        DeclareLaunchArgument('start_realsense', default_value='false',
                              description='Start RealSense D435i driver'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint -> base_link'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true',
                              description='Delete old RTAB-Map database on startup for a clean mapping session'),
        
        # We change the default scan_cloud_topic from /livox/lidar to /cloud_registered_body.
        # This is the motion-deskewed pointcloud exported by FAST-LIO, meaning 
        # RTAB-Map's loop closure and map generation will be much cleaner/sharper.
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body',
                              description='Input point cloud topic for RTAB-Map'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/livox/imu',
            description='Optional RTAB-Map IMU topic. FAST-LIO still uses /livox/imu from its own config.',
        ),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/infra1/camera_info'),
        DeclareLaunchArgument('left_image_topic', default_value='/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('right_image_topic', default_value='/camera/infra2/image_rect_raw'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/camera/infra1/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/camera/infra2/camera_info'),
        DeclareLaunchArgument('database_path', default_value='./map/rtabmap_2d__.db',
                              description='Path to save/load the RTAB-Map database'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Robot base frame'),
        DeclareLaunchArgument('odom_frame_id', default_value='',
                              description='RTAB-Map odometry TF frame. Keep empty to use odom_topic'),
        # Kp/DetectorStrategy and Vis/FeatureType
        # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector 16=SuperPoint-Rpautrat")
        DeclareLaunchArgument('rtabmap_args', default_value=
            (                
                # === 基础注册与视觉参数 ===
                "--Reg/Strategy 1 "
                # "--Reg/Force3DoF true "
                # "--RGBD/ForceOdom3DoF true "
                "--RGBD/NeighborLinkRefining true "
                "--RGBD/ProximityBySpace true "
                # "--RGBD/OptimizeMaxError 5 "
                "--Stereo/Gpu false "
                "--Stereo/DenseStrategy 1 "
                "--Mem/NotLinkedNodesKept false "

                "--Kp/DetectorStrategy 11 "
                "--FAST/Gpu true "
                "--GFTT/Gpu true "
                "--ORB/Gpu true "
                "--SIFT/Gpu true "
                "--SURF/GpuVersion true "
                "--SuperPoint/Cuda true "
                "--SuperPoint/ModelPath ./superpoint_superglue/SuperPointPretrainedNetwork/superpoint_v1.pt "
                "--Vis/FeatureType 11 "
                "--Vis/MaxFeatures 800 "
                "--Vis/MinInliers 80 "
                "--Vis/CorNNType 6 "
                "--PyMatcher/Path ./superpoint_superglue/SuperGluePretrainedNetwork/rtabmap_superglue.py "
                "--PyMatcher/Cuda true "
                "--PyMatcher/Model outdoor "
                "--Icp/VoxelSize 0.05 "
                "--Icp/MaxTranslation 1.0 "             
                "--Icp/MaxRotation 0.7 "
                "--Icp/MaxCorrespondenceDistance 0.3 "  # 放宽匹配距离
                "--Icp/CorrespondenceRatio 0.1 "
                "--Icp/PointToPlane true "
                "--Icp/PointToPlaneK 10 "
                "--Icp/PointToPlaneMinComplexity 0.04 "
                # === 栅格地图核心参数（空旷停车场 2D 地图优化）===
                "--Grid/3D false "                 # 使用 2D 占用栅格地图
                "--Grid/Sensor 0 "
                "--Grid/RayTracing true "
                "--Grid/NormalsSegmentation false " # 当设为 false 时：使用快速直通滤波（fast passthrough）代替法线分割 此时需要设置 Grid/MaxGroundHeight 来定义地面高度阈值 所有高于 MaxGroundHeight 的点被视为障碍物
                # "--Grid/MinGroundHeight -1.7 "
                # "--Grid/MaxGroundHeight -1.2 "
                # "--Grid/MaxObstacleHeight 0.2 "
                # "--Grid/MinGroundHeight -0.2 "
                "--Grid/MaxGroundHeight 0.3 "
                "--Grid/MaxObstacleHeight 1.0 "
                "--Grid/CellSize 0.15 "
                "--Grid/RangeMin 0.0 "
                "--Grid/RangeMax 20.0 "
            )
        ),
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(PythonExpression([
            "'", start_livox, "' == 'true'"
        ])),
    )
    
    # ── 1.2. RealSense D435i driver ──
    realsense_stereo_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[{
            'enable_color': False,
            'enable_depth': False,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': True, # 内部硬件同步
            'align_depth.enable': False, # 深度对齐到彩色
            'initial_reset': False, # 启动时不重置设备，避免丢帧
        }],
        condition=IfCondition(PythonExpression([
            "'", start_realsense, "' == 'true' and '",
            sensor_profile, "' == 'lidar_stereo'"
        ])),
    )

    # ── 2. Static TFs ──
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
        # arguments=[
        #     '-0.25', '0.18', '-0.019',
        #     '0.0', '0.20845989984609956', '0.0', '0.9780309147241483',
        #     'base_link', 'camera_link'
        # ],
        arguments=['-0.03', '0.0', '-0.10', '0.0', '-0.12', '0.0', 'base_link', 'camera_link'],
    )

    # ── Stereo camera static TFs from bag ──
    camera_infra1_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_infra1_frame',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'camera_link', 'camera_infra1_frame'
        ],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' == 'lidar_stereo' "
        ])),
    )

    camera_aligned_depth_to_infra1_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_aligned_depth_to_infra1_frame',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'camera_link', 'camera_aligned_depth_to_infra1_frame'
        ],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' == 'lidar_stereo' "
        ])),
    )

    camera_infra1_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_aligned_depth_to_infra1_frame_to_camera_infra1_optical_frame',
        arguments=[
            '0.0', '0.0', '0.0',
            '-0.5', '0.4999999999999999', '-0.5', '0.5000000000000001',
            'camera_aligned_depth_to_infra1_frame', 'camera_infra1_optical_frame'
        ],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' == 'lidar_stereo' "
        ])),
    )

    camera_infra2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_infra2_frame',
        arguments=[
            '0.0', '-0.04991656169295311', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'camera_link', 'camera_infra2_frame'
        ],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' == 'lidar_stereo' "
        ])),
    )

    camera_infra2_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_infra2_frame_to_camera_infra2_optical_frame',
        arguments=[
            '0.0', '0.0', '0.0',
            '-0.5', '0.4999999999999999', '-0.5', '0.5000000000000001',
            'camera_infra2_frame', 'camera_infra2_optical_frame'
        ],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' == 'lidar_stereo' "
        ])),
    )

    # ── 3. FAST-LIO (Replaces icp_odometry & ekf_node) ──
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': 'mid360.yaml',
            'rviz': 'false', # Disable FAST-LIO's separate RViz to prevent conflict
        }.items(),
    )

    # ── 4. RTAB-Map SLAM ──
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': 'false',
            'database_path': LaunchConfiguration('database_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'publish_tf_map': 'true',
            'publish_tf_odom': 'false',
            'odom_topic': '/Odometry',   # <--- Link to FAST-LIO's odometry topic
            'imu_topic': LaunchConfiguration('imu_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
            'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
            'subscribe_scan_cloud': 'true',
            'subscribe_scan': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'rviz': 'true',
            'rtabmap_viz': rtabmap_viz,
            'rgbd_sync': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            # 'rgbd_sync': 'false',
            'subscribe_rgbd': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            # 'subscribe_rgbd': 'false',
            'subscribe_rgb': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_mono' else 'false'"]),
            'depth': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' else 'false'"]),
            'stereo': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            'approx_sync': 'true',
            'qos': '2',
            'namespace': 'rtabmap',
            'args':[
                PythonExpression(["'-d ' if '", delete_db_on_start, "' == 'true' else ''"]),
                LaunchConfiguration('rtabmap_args'),
            ],
        }.items(),
    )

    # ── Assemble ──
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    
    # RTAB-Map library workaround: dynamically locate from workspace root
    _colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    if _colcon_prefix:
        _ws_root = os.path.dirname(_colcon_prefix.split(':')[0])
        _rtabmap_lib = os.path.join(_ws_root, 'third_party', 'rtabmap-0.23.4', 'install', 'lib')
        _torch_lib = '/home/dog/.local/lib/python3.10/site-packages/torch/lib'
        _existing_ldpath = os.environ.get('LD_LIBRARY_PATH', '')
        _extra_libs = ':'.join([_rtabmap_lib, _torch_lib])
        ld.add_action(SetEnvironmentVariable('LD_LIBRARY_PATH',
            _extra_libs + ':' + _existing_ldpath if _existing_ldpath else _extra_libs
        ))
    
    for arg in declare_args:
        ld.add_action(arg)
        
    ld.add_action(body_tf)
    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    ld.add_action(camera_tf)
    ld.add_action(camera_infra1_tf)
    ld.add_action(camera_aligned_depth_to_infra1_tf)
    ld.add_action(camera_infra1_optical_tf)
    ld.add_action(camera_infra2_tf)
    ld.add_action(camera_infra2_optical_tf)



    ld.add_action(livox_launch)
    ld.add_action(realsense_stereo_node)
    ld.add_action(fast_lio_launch)
    ld.add_action(rtabmap_launch)
    
    return ld
