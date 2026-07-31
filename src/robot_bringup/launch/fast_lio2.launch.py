#!/usr/bin/env python3
"""
FAST-LIO + RTAB-Map mapping bringup with correct RViz visualization.

Starts:
  1. Livox MID360 lidar driver
  2. FAST-LIO odometry -> /Odometry (and odom->base_footprint TF)
  3. RTAB-Map SLAM (consumes /Odometry and /cloud_registered_body)
  4. Static TFs
  5. RViz2 with LiDAR + map + pose visualization

Usage:
  ros2 launch robot_bringup fast_lio2.launch.py
  ros2 launch robot_bringup fast_lio2.launch.py rviz:=false start_livox:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox = LaunchConfiguration('start_livox')
    start_ydlidar = LaunchConfiguration('start_ydlidar')   # <修改 version3 YDLIDAR 2D雷达>
    use_fast_lio = LaunchConfiguration('use_fast_lio')     # <修改 version3 ICP里程计切换>
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    rviz = LaunchConfiguration('rviz')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    scan_topic = LaunchConfiguration('scan_topic')          # <修改 version3 LaserScan话题>

    livox_share = FindPackageShare('livox_ros_driver2')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')
    fast_lio_share = FindPackageShare('fast_lio')

    # ── Rviz config (resolved at parse time to avoid substitution issues) ──
    rviz_config = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'fast_lio2.rviz')

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping | localization'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_livox', default_value='true',
                              description='Start Livox MID360 driver'),
        # <修改 version3 YDLIDAR 2D雷达支持>
        DeclareLaunchArgument('start_ydlidar', default_value='false',
                              description='Start YDLIDAR T-mini Plus driver (2D LiDAR)'),
        DeclareLaunchArgument('use_fast_lio', default_value='true',
                              description='Use FAST-LIO for odometry (3D). Set false for 2D LiDAR + RTAB-Map ICP odometry'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='LaserScan topic from 2D LiDAR'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint -> base_link'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true',
                              description='Delete old RTAB-Map database on startup for a clean mapping session'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body',
                              description='Input point cloud topic for RTAB-Map'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/unused_imu',
            description='Optional IMU topic. FAST-LIO uses /livox/imu from its own config.',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value=EnvironmentVariable('PARKING_ROBOT_RTABMAP_DATABASE', default_value=''),
            description='RTAB-Map database path. Explicit launch argument overrides PARKING_ROBOT_RTABMAP_DATABASE.',
        ),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Robot base frame'),
        DeclareLaunchArgument('odom_frame_id', default_value='',
                              description='RTAB-Map odometry TF frame. Keep empty to use odom_topic'),
        DeclareLaunchArgument('initial_pose', default_value='',
                              description='Initial pose for localization: "x y z roll pitch yaw" (space-separated, radians) or empty'),
    ]

    # ── 1a. YDLIDAR T-mini Plus driver ──  # <修改 version3>
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        condition=IfCondition(start_ydlidar),
        parameters=[PathJoinSubstitution([FindPackageShare('ydlidar_ros2_driver'), 'params', 'TminiPro.yaml'])],
    )

    # <修改 version3 新式参数: 老式参数在ROS2 Humble中已不发布/tf_static>
    ydlidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        arguments=['--x', '0', '--y', '0', '--z', '0.02', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
        condition=IfCondition(start_ydlidar),
    )

    # ── 1c. LaserScan -> PointCloud2 converter ──  # <修改 version3>
    scan_to_pc = Node(
        package='robot_bringup',
        executable='scan_to_pointcloud.py',
        name='scan_to_pointcloud',
        output='screen',
        condition=IfCondition(PythonExpression(["'", start_ydlidar, "' == 'true' and '", use_fast_lio, "' != 'true'"])),
        parameters=[{
            'target_topic': '/cloud_registered_body',
            'scan_topic': scan_topic,
        }],
    )

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(start_livox),
    )

    # ── 2. Static TFs ──
    # <修改 version3 新式参数: 老式参数在ROS2 Humble中已不发布/tf_static>
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
        condition=IfCondition(publish_base_link_tf),
    )

    livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
    )

    # ── 3. FAST-LIO ──  # <修改 version3 YDLIDAR: use_fast_lio=false时跳过>
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])),
        condition=IfCondition(use_fast_lio),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': 'mid360.yaml',
            'rviz': 'false',  # We handle rviz ourselves
        }.items(),
    )

    # ── 4. RTAB-Map SLAM / Localization ──
    # LiDAR-only common args (ICP + Grid, shared by mapping & localization)
    # <原版-机器狗参数> ICP参数为Livox MID360 3D雷达优化
    # <修改 version3 YDLIDAR 2D雷达: 放宽MaxCorrespondenceDistance=1.0 (原0.5对稀疏2D点云太严)
    #   PointToPlaneK/PointToPlaneMinComplexity=0 (PointToPlane关闭时无需设置)>
    _lidar_args = (
        "'--Reg/Strategy 1 --RGBD/ProximityBySpace true --RGBD/ProximityOdomGuess true "
        "--RGBD/ProximityPathMaxNeighbors 10 "
        "--Icp/VoxelSize 0.05 --Icp/DownsamplingStep 1 --Icp/MaxTranslation 1.5 "
        "--Icp/MaxRotation 0.7 --Icp/MaxCorrespondenceDistance 1.0 --Icp/CorrespondenceRatio 0.05 "
        "--Icp/PointToPlaneK 0 --Icp/PointToPlaneMinComplexity 0 "
        "--Grid/3D false --Grid/NormalsSegmentation true --Grid/MaxGroundAngle 15 "
        "--Grid/MinGroundHeight -1.2 --Grid/MaxGroundHeight -0.50 --Grid/MaxObstacleHeight 0.20 "
        "--Grid/RayTracing true'"
    )
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': PythonExpression(["'true' if '", mode, "' != 'mapping' else 'false'"]),
            'database_path': LaunchConfiguration('database_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'publish_tf_map': 'true',
            # <原版> publish_tf_odom: false (FAST-LIO publishes odom TF)
            # <修改 version3 YDLIDAR: use_fast_lio=false时 ICP里程计发布odom TF>
            'publish_tf_odom': PythonExpression(["'false' if '", use_fast_lio, "' == 'true' else 'true'"]),
            'odom_topic': '/Odometry',
            'imu_topic': LaunchConfiguration('imu_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'scan_topic': scan_topic,
            'initial_pose': LaunchConfiguration('initial_pose'),
            # <原版> subscribe_scan_cloud: true, subscribe_scan: false, icp_odometry: false
            # <修改 version3 YDLIDAR: use_fast_lio=false时用scan+ICP里程计>
            'subscribe_scan_cloud': 'true',
            'subscribe_scan': PythonExpression(["'true' if '", use_fast_lio, "' != 'true' else 'false'"]),
            'visual_odometry': 'false',
            'icp_odometry': PythonExpression(["'true' if '", use_fast_lio, "' != 'true' else 'false'"]),
            'vo_frame_id': PythonExpression(["'/Odometry' if '", use_fast_lio, "' == 'true' else 'odom'"]),
            'scan_voxel_size': '0.0',
            'rviz': 'false',  # We handle rviz ourselves
            'rtabmap_viz': 'false',
            'rgbd_sync': 'false',
            'subscribe_rgbd': 'false',
            'subscribe_rgb': 'false',
            'depth': 'false',
            'stereo': 'false',
            # <修改 version3 wait_for_transform=2.0: 确保静态TF(/tf_static)在RTAB-Map处理首帧前已缓存>
            'wait_for_transform': '2.0',
            'approx_sync': 'true',
            'qos': '2',
            'namespace': 'rtabmap',
            'args': PythonExpression([
                # <修改 version3 YDLIDAR 2D: PointToPlane与PCL 2D不兼容, 仅3D时启用>
                "'--Icp/PointToPlane true ' if '", use_fast_lio, "' == 'true' else '--Icp/PointToPlane false '",
                # Common LiDAR args
                _lidar_args,
                # delete_db_on_start (only in mapping mode to avoid wiping DB during localization)
                " + (' --delete_db_on_start ' if '", delete_db_on_start, "' == 'true' and '", mode, "' == 'mapping' else '')",
                # Mapping mode: prune unlinked nodes
                " + (' --Mem/NotLinkedNodesKept false' if '", mode, "' == 'mapping' else '')",
                # Localization mode: load all nodes, don't create new ones
                # <原版-重定位参数> 原版包含 --RGBD/OptimizeFromGraphEnd true
                # <修改 version1 删除/OptimizeFromGraphEnd参数> 原因是 OptimizeFromGraphEnd=true 时
                #   map→odom TF 始终为 identity，导致 initial_pose 参数无效，机器人每次都在原点启动。
                #   设为 false（默认值）后，map→odom = initialPose * odom⁻¹，机器人正确出现在指定初始位姿。
                " + (' --Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true' if '", mode, "' == 'localization' else '')",
            ]),
        }.items(),
    )

    # ── 5. RViz2 ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        # condition=IfCondition(rviz),
    )

    # ── Assemble ──
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # RTAB-Map library workaround: dynamically locate from workspace root
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
        ld.add_action(SetEnvironmentVariable('LD_LIBRARY_PATH',
            _extra_libs + ':' + _existing_ldpath if _existing_ldpath else _extra_libs
        ))

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    # <修改 version3 YDLIDAR节点>
    ld.add_action(ydlidar_node)
    ld.add_action(ydlidar_tf)
    ld.add_action(scan_to_pc)
    ld.add_action(livox_launch)
    ld.add_action(fast_lio_launch)
    ld.add_action(rtabmap_launch)
    ld.add_action(rviz_node)

    return ld
