#!/usr/bin/env python3
"""RTAB-Map-led AMR bringup starter.

Suggested location:
  robot_bringup/launch/bringup.launch.py

Assumptions:
- RTAB-Map is the only publisher of map -> odom.
- FAST-LIO publishes odom -> base_footprint and /Odometry.
- Nav2 consumes /map and /Odometry.
- Nav2 outputs /cmd_vel_nav → collision_monitor filters → /cmd_vel → wheeltec hardware.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration('namespace')
    mode = LaunchConfiguration('mode')
    sensor_profile = LaunchConfiguration('sensor_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start_livox = LaunchConfiguration('start_livox')
    start_realsense = LaunchConfiguration('start_realsense')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_rviz = LaunchConfiguration('enable_rviz')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    database_path = LaunchConfiguration('database_path')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    enable_laser_avoidance = LaunchConfiguration('enable_laser_avoidance')
    scan_topic = LaunchConfiguration('scan_topic')

    robot_bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio')
    realsense_share = FindPackageShare('realsense2_camera')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('mode', default_value='navigation', description='mapping | localization | navigation'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_stereo', description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_livox', default_value='false', description='Start Livox MID360 launch'),
        DeclareLaunchArgument('start_realsense', default_value='false', description='Start RealSense D435i launch'),
        DeclareLaunchArgument('enable_gps', default_value='false', description='Enable navsat_transform and pass GPS fix to RTAB-Map'),
        DeclareLaunchArgument('enable_rviz', default_value='true', description='Launch RViz with Nav2 navigation config'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true', description='Publish a zero static TF from base_footprint to base_link if URDF is not ready'),
        DeclareLaunchArgument('enable_laser_avoidance', default_value='true', description='Use low 2D lidar to generate /active_plan and safety-filter wheel commands'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='Low 2D lidar LaserScan topic for avoidance'),
        DeclareLaunchArgument('database_path', default_value='./map/rtabmap_2d.db'),
        DeclareLaunchArgument('rtabmap_args', default_value=
            (                
                # === 基础注册与视觉参数 ===
                "--Reg/Strategy 1 "
                "--RGBD/NeighborLinkRefining true "
                "--RGBD/ProximityBySpace true "
                "--RGBD/OptimizeMaxError 5 "
                "--RGBD/ProximityAngle 90 "
                "--RGBD/NeighborLinkRefining true "
                "--RGBD/ProximityPathMaxNeighbors 1 "
                # "--RGBD/CreateOccupancyGrid true "
                "--RGBD/StartAtOrigin true "
                "--Stereo/Gpu false "
                
                "--Mem/NotLinkedNodesKept false "
                "--Mem/IncrementalMemory false "

                "--FAST/Gpu true "
                "--GFTT/Gpu true "
                "--ORB/Gpu true "
                "--SIFT/Gpu true "
                "--SURF/GpuVersion true "
                "--Vis/MaxFeatures 800 "
                "--Vis/MinInliers 40 "
                # === ICP 配准参数（空旷场景优化）===
                "--Icp/VoxelSize 0.05 "
                "--Icp/DownsamplingStep 1 "
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
                "--Grid/MinGroundHeight -0.2 "
                "--Grid/MaxGroundHeight 0.3 "
                "--Grid/MaxObstacleHeight 1.0 "
                "--Grid/CellSize 0.15 "
                "--Grid/RangeMin 0.0 "
                "--Grid/RangeMax 20.0 "
            )
        ),
        DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_common.yaml'])),
        DeclareLaunchArgument('rtabmap_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('rtabmap_map_frame', default_value='map'),
        DeclareLaunchArgument('rtabmap_odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu', description='Optional IMU topic for RTAB-Map and navsat_transform when GPS is enabled.'),
        DeclareLaunchArgument('gps_fix_topic', default_value='/sensors/gps/fix'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('left_image_topic', default_value='/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('right_image_topic', default_value='/camera/infra2/image_rect_raw'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/camera/infra1/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/camera/infra2/camera_info'),
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(
            PythonExpression(["'", start_livox, "' == 'true'"])
        ),
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
             # === 关键修复：关闭所有不需要的流，减少USB带宽 ===
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_module.emitter_enabled': 0,  # 确保红外发射器开启以获得更好的深度质量
            # === 关键修复：同步降低红外流硬件配置，解决帧超时 ===
            'depth_module.depth_profile': '640x480x15',
            'depth_module.infra_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            # === 功能配置 ===
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
    #   相机 TF (根据实际安装位置修改平移和旋转参数，单位为米和弧度)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=['-0.03', '0.0', '-0.10', '0.0', '-0.12', '0.0', 'base_link', 'camera_link'],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' != 'lidar_only' "
        ])), # 仅在启动相机时发布此 TF
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


    # ── 3. FAST-LIO (replaces icp_odometry + EKF) ──
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': 'mid360.yaml',
            'rviz': 'false',
        }.items(),
    )

    # ── 4. GPS (optional, requires robot_localization installed separately) ──
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        condition=IfCondition(enable_gps),
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 20.0,
            'delay': 1.0,
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'broadcast_utm_transform': False,
            'publish_filtered_gps': False,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
        }],
        remappings=[
            ('imu/data', LaunchConfiguration('imu_topic')),
            ('gps/fix', LaunchConfiguration('gps_fix_topic')),
            ('odometry/filtered', '/Odometry'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    # ── 5. RTAB-Map (SLAM / Localization) ──
    rtabmap_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([robot_bringup_share, 'launch', 'rtabmap_bridge.launch.py'])),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'sensor_profile': sensor_profile,
            'enable_gps': enable_gps,
            'localization': PythonExpression(["'true' if '", mode, "' != 'mapping' else 'false'"]),
            'database_path': database_path,
            'rtabmap_args': LaunchConfiguration('rtabmap_args'),
            'frame_id': LaunchConfiguration('rtabmap_frame_id'),
            'map_frame_id': LaunchConfiguration('rtabmap_map_frame'),
            'odom_topic': LaunchConfiguration('rtabmap_odom_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'gps_topic': LaunchConfiguration('gps_fix_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
            'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
            'rviz': 'false',
            'rtabmap_viz': 'true'
        }.items(),
    )

    # ── 5b. RViz with Nav2 navigation config ──
    nav2_rviz_config = PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_navigation.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(enable_rviz),
        arguments=['-d', nav2_rviz_config],
    )

    # ── 6. Nav2 ──
    # Remap /cmd_vel → /cmd_vel_nav so collision_monitor can intercept before the hardware.
    nav2_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        actions=[
            SetRemap('/cmd_vel', '/cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': nav2_params_file,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'log_level': 'warn',
                }.items(),
            ),
        ],
    )

    pure_pursuit_node = Node(
    package='wheelchair_controller',
        executable='pure_pursuit_controller_node',
        condition=IfCondition(enable_laser_avoidance),
        parameters=[{
            'lookahead_distance': 1.65,      # 前瞻距离：越大转弯越平滑，但过弯容易切内角
            'avoidance_lookahead_distance': 0.8,
            'min_turning_radius': 1.5,       # 轮椅物理极限
            'linear_velocity': 4.5,           # 巡航速度，原 3.0 的 1.5 倍
            'rotate_in_place_speed': 1.6,
            'goal_tolerance': 0.9,            # 到达终点的距离误差容限
            'goal_yaw_tolerance': 3.14,        # 到达终点的角度误差容限
            'stop_on_position_reached': True,  # 进入距离容差后直接停车，不再终点原地调朝向
            # 'path_topic_name': "/mapPath",
            # 'path_topic_name': "/plan",
            'path_topic_name': "/active_plan",
            'control_topic_name': "/wheelchair_control_command_raw",
            'use_sim_time': use_sim_time
        }]
    )

    pure_pursuit_node_no_avoidance = Node(
    package='wheelchair_controller',
        executable='pure_pursuit_controller_node',
        condition=IfCondition(PythonExpression(["'", enable_laser_avoidance, "' != 'true'"])),
        parameters=[{
            'lookahead_distance': 1.65,      # 前瞻距离：越大转弯越平滑，但过弯容易切内角
            'avoidance_lookahead_distance': 0.8,
            'min_turning_radius': 1.5,       # 轮椅物理极限
            'linear_velocity': 4.5,           # 巡航速度，原 3.0 的 1.5 倍
            'rotate_in_place_speed': 1.6,
            'goal_tolerance': 0.9,            # 到达终点的距离误差容限
            'goal_yaw_tolerance': 3.14,        # 到达终点的角度误差容限
            'stop_on_position_reached': True,  # 进入距离容差后直接停车，不再终点原地调朝向
            'path_topic_name': "/plan_nav",
            'control_topic_name': "/wheelchair_control_command",
            'use_sim_time': use_sim_time
        }]
    )

    plan_nav_laser_avoidance = Node(
        package='robot_bringup',
        executable='plan_nav_laser_avoidance.py',
        name='plan_nav_laser_avoidance',
        output='screen',
        condition=IfCondition(enable_laser_avoidance),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_path_topic': '/plan_nav',
            'output_path_topic': '/active_plan',
            'scan_topic': scan_topic,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'obstacle_enter_distance': 1.5,
            'obstacle_exit_distance': 1.6,
            'emergency_stop_distance': 1.0,
            'rear_self_filter_distance': 1.0,
            'front_sector_deg': 50.0,
            'front_channel_half_width': 0.5,
            'side_inner_deg': 25.0,
            'side_outer_deg': 50.0,
            'side_clearance_min': 0.8,
            'side_clearance_margin': 0.3,
            'emergency_stop_duration': 0.8,
            'avoidance_depart_angle_deg': 45.0,
            'avoidance_depart_distance': 1.414,
            'return_margin': 2.0,
            'min_return_distance': 3.0,
            'max_return_distance': 7.0,
            'clear_duration': 1.0,
            'return_tolerance': 1.0,
            'path_lateral_tolerance': 1.0,
            'heading_tolerance_deg': 60.0,
            'obstacle_confirm_frames': 3,
        }],
    )

    laser_command_safety_filter = Node(
        package='robot_bringup',
        executable='laser_command_safety_filter.py',
        name='laser_command_safety_filter',
        output='screen',
        condition=IfCondition(enable_laser_avoidance),
        parameters=[{
            'input_command_topic': '/wheelchair_control_command_raw',
            'output_command_topic': '/wheelchair_control_command',
            'scan_topic': scan_topic,
            'avoidance_state_topic': '/laser_avoidance_state',
            'enabled': True,
            'require_scan': False,
            'front_sector_deg': 50.0,
            'front_channel_half_width': 0.5,
            'rear_self_filter_distance': 1.0,
            'slowdown_distance': 1.5,
            'emergency_stop_distance': 1.0,
            'hard_stop_distance': 0.30,
            'min_slowdown_scale': 0.45,
            'avoidance_turn_radius_max_mm': 20000.0,
            'avoidance_max_speed_mm_s': 1400.0,
        }],
    )

    # ── 7. Collision Monitor ──
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'cmd_vel_in_topic': '/cmd_vel_nav',
            'cmd_vel_out_topic': '/cmd_vel',
            'transform_tolerance': 0.3,
            'source_timeout': 1.0,
            'base_shift_correction': True,
            'stop_pub_timeout': 1.0,
            'polygons': ['StopZone', 'SlowZone'],
            'observation_sources': ['pointcloud'],
            'StopZone.type': 'polygon',
            'StopZone.points': [0.35, 0.30, 0.35, -0.30, -0.10, -0.30, -0.10, 0.30],
            'StopZone.action_type': 'stop',
            'StopZone.max_points': 3,
            'StopZone.visualize': True,
            'StopZone.polygon_pub_topic': 'collision_monitor/stop_zone',
            'StopZone.enabled': True,
            'SlowZone.type': 'polygon',
            'SlowZone.points': [0.55, 0.40, 0.55, -0.40, -0.25, -0.40, -0.25, 0.40],
            'SlowZone.action_type': 'slowdown',
            'SlowZone.max_points': 3,
            'SlowZone.slowdown_ratio': 0.35,
            'SlowZone.visualize': True,
            'SlowZone.polygon_pub_topic': 'collision_monitor/slow_zone',
            'SlowZone.enabled': True,
            'pointcloud.type': 'pointcloud',
            'pointcloud.topic': '/cloud_registered_body',
            'pointcloud.min_height': -1.1,
            'pointcloud.max_height': 0.1,
            'pointcloud.enabled': True,
        }],
    )

    collision_monitor_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['collision_monitor'],
        }],
    )

    # ── Assemble ──
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

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

    for action in declare_args:
        ld.add_action(action)
        
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
    ld.add_action(navsat_transform)
    ld.add_action(rtabmap_bridge)
    ld.add_action(rviz_node)
    ld.add_action(nav2_launch)
    ld.add_action(plan_nav_laser_avoidance)
    ld.add_action(pure_pursuit_node)
    ld.add_action(pure_pursuit_node_no_avoidance)
    ld.add_action(laser_command_safety_filter)
    # ld.add_action(collision_monitor)
    # ld.add_action(collision_monitor_lifecycle)
    return ld
