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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    start_ydlidar = LaunchConfiguration('start_ydlidar')  # <修改 version3 YDLIDAR 2D雷达支持>
    use_fast_lio = LaunchConfiguration('use_fast_lio')    # <修改 version3 控制是否使用FAST-LIO里程计>
    use_fake_odom = LaunchConfiguration('use_fake_odom')   # <修改 version3 假里程计模拟模式>
    enable_gps = LaunchConfiguration('enable_gps')
    enable_rviz = LaunchConfiguration('enable_rviz')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    database_path = LaunchConfiguration('database_path')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    lookahead_distance = LaunchConfiguration('lookahead_distance')  # <修改 version2 预瞄点距离>
    scan_topic = LaunchConfiguration('scan_topic')  # <修改 version3 YDLIDAR 2D雷达scan话题>

    robot_bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('mode', default_value='navigation', description='mapping | localization | navigation'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_only', description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_livox', default_value='true', description='Start Livox MID360 launch'),
        # <修改 version3 YDLIDAR 2D雷达支持>
        DeclareLaunchArgument('start_ydlidar', default_value='false', description='Start YDLIDAR T-mini Plus driver'),
        DeclareLaunchArgument('use_fast_lio', default_value='true', description='Use FAST-LIO for odometry (3D LiDAR). Set false for 2D LiDAR + RTAB-Map ICP odometry'),
        # <修改 version3 假里程计模式: 无机器人时模拟运动>
        DeclareLaunchArgument('use_fake_odom', default_value='false', description='Use fake odometry for simulation without a robot'),
        DeclareLaunchArgument('ydlidar_params_file', default_value=PathJoinSubstitution([FindPackageShare('ydlidar_ros2_driver'), 'params', 'TminiPro.yaml']), description='YDLIDAR parameter file'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='LaserScan topic from 2D LiDAR'),
        DeclareLaunchArgument('enable_gps', default_value='false', description='Enable navsat_transform and pass GPS fix to RTAB-Map'),
        DeclareLaunchArgument('enable_rviz', default_value='false', description='Launch RViz with Nav2 navigation config'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true', description='Publish a zero static TF from base_footprint to base_link if URDF is not ready'),
        DeclareLaunchArgument(
            'database_path',
            default_value=EnvironmentVariable('PARKING_ROBOT_RTABMAP_DATABASE', default_value=''),
            description='RTAB-Map database path. Explicit launch argument overrides PARKING_ROBOT_RTABMAP_DATABASE.',
        ),
        DeclareLaunchArgument('rtabmap_args', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_common.yaml'])),
        DeclareLaunchArgument('rtabmap_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('rtabmap_map_frame', default_value='map'),
        DeclareLaunchArgument('rtabmap_odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/unused_imu',
            description='Optional IMU topic for RTAB-Map and navsat_transform when GPS is enabled.',
        ),
        DeclareLaunchArgument('gps_fix_topic', default_value='/sensors/gps/fix'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('lookahead_distance', default_value='1.0',
                              description='Lookahead distance (meters) for the preview point on the plan'),  # <修改 version2 预瞄点>
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='Launch RTAB-Map standalone visualization GUI'),
    ]

    # ── 1a. YDLIDAR T-mini Plus driver ──  # <修改 version3 YDLIDAR 2D雷达>
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        condition=IfCondition(start_ydlidar),
        parameters=[LaunchConfiguration('ydlidar_params_file')],
    )

    # <修改 version3 新式参数: 老式参数在ROS2 Humble中已不发布/tf_static>
    ydlidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        arguments=['--x', '0', '--y', '0', '--z', '0.02', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
        condition=IfCondition(start_ydlidar),
    )

    # ── 1c. LaserScan -> PointCloud2 converter (for costmaps when FAST-LIO is off) ──  # <修改 version3>
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
        PythonLaunchDescriptionSource(PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
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

    # ── 3. FAST-LIO (replaces icp_odometry + EKF) ──
    # <修改 version3 YDLIDAR 2D雷达: use_fast_lio=false时跳过FAST-LIO，改用RTAB-Map ICP里程计>
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])),
        condition=IfCondition(use_fast_lio),
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
            # <修改 version3 YDLIDAR 2D雷达支持>
            'use_fast_lio': use_fast_lio,
            'use_fake_odom': use_fake_odom,
            'scan_topic': scan_topic,
            'rviz': 'false',
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
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
                    'log_level': 'info',
                }.items(),
            ),
        ],
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
            'pointcloud.min_height': 0.05,
            'pointcloud.max_height': 1.80,
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

    # ── 8. Preview Point Publisher (navigation mode only) ──  # <修改 version2 增加预瞄点节点>
    preview_point = Node(
        package='robot_bringup',
        executable='preview_point_publisher.py',
        name='preview_point_publisher',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'lookahead_distance': lookahead_distance,
            'plan_topic': '/plan',
        }],
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

    for action in declare_args:
        ld.add_action(action)
    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    ld.add_action(livox_launch)
    # <修改 version3 YDLIDAR 2D雷达节点>
    ld.add_action(ydlidar_node)
    ld.add_action(ydlidar_tf)
    ld.add_action(scan_to_pc)
    ld.add_action(fast_lio_launch)
    ld.add_action(navsat_transform)
    ld.add_action(rtabmap_bridge)
    ld.add_action(rviz_node)
    ld.add_action(nav2_launch)
    ld.add_action(collision_monitor)
    ld.add_action(collision_monitor_lifecycle)
    ld.add_action(preview_point)  # <修改 version2 预瞄点>
    return ld
