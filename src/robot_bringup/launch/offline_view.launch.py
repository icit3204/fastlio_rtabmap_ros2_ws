#!/usr/bin/env python3
"""Offline map viewer - loads RTAB-Map database and displays map in RViz without any hardware (LiDAR, GPS, etc.).

Usage:
  ros2 launch robot_bringup offline_view.launch.py database_path:=/data/maps/site_a/2_version_0514.db

Optional:
  enable_rviz:=true    (default true)
  rtabmap_viz:=true    (default false, opens RTAB-Map standalone GUI with MapGraph/trajectory)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_bringup_share = FindPackageShare('robot_bringup')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')

    database_path = LaunchConfiguration('database_path')
    enable_rviz = LaunchConfiguration('enable_rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sensor_profile = LaunchConfiguration('sensor_profile')

    declare_args = [
        DeclareLaunchArgument('database_path', default_value='/data/maps/site_a/rtabmap.db',
                              description='Path to RTAB-Map .db file'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
                              description='Launch RViz with Nav2 navigation config'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='Launch RTAB-Map standalone visualization GUI (shows MapGraph / trajectory)'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_only'),
    ]

    # ── Static TFs (identity) — the full tree exists without localization ──
    map_to_odom_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='offline_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    odom_to_base_footprint_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='offline_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )
    base_footprint_to_base_link_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='offline_base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )
    base_link_to_livox_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='offline_base_link_to_livox',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
    )

    # ── RTAB-Map params (same Grid/ICP as production for consistent map display) ──
    grid_args = (
        '--Reg/Strategy 1 '
        '--RGBD/ProximityBySpace true '
        '--RGBD/ProximityOdomGuess true '
        '--RGBD/ProximityPathMaxNeighbors 10 '
        '--Icp/VoxelSize 0.05 '
        '--Icp/DownsamplingStep 1 '
        '--Icp/MaxTranslation 1.5 '
        '--Icp/MaxRotation 0.7 '
        '--Icp/MaxCorrespondenceDistance 0.5 '
        '--Icp/CorrespondenceRatio 0.05 '
        '--Icp/PointToPlane true '
        '--Icp/PointToPlaneK 15 '
        '--Icp/PointToPlaneMinComplexity 0.04 '
        '--Grid/3D false '
        '--Grid/NormalsSegmentation true '
        '--Grid/MaxGroundAngle 15 '
        '--Grid/MinGroundHeight -0.85 '
        '--Grid/MaxGroundHeight -0.50 '
        '--Grid/MaxObstacleHeight 0.20 '
        '--Grid/RayTracing true'
    )

    # ── RTAB-Map in localization mode, no sensor subscriptions ──
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'localization': 'true',
            'database_path': database_path,
            'frame_id': 'base_footprint',
            'map_frame_id': 'map',
            'map_topic': 'map',
            'publish_tf_map': 'false',       # static TF instead
            'publish_tf_odom': 'false',      # static TF instead
            'odom_topic': '/Odometry',
            'imu_topic': '/unused_imu',
            'subscribe_scan': 'false',
            'subscribe_scan_cloud': 'false',
            'subscribe_rgb': 'false',
            'subscribe_rgbd': 'false',
            'depth': 'false',
            'stereo': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'approx_sync': 'true',
            'qos': '2',
            'rviz': 'false',
            'rtabmap_viz': rtabmap_viz,
            'rtabmap_args': grid_args,
            'wait_imu_to_init': 'false',
        }.items(),
    )

    # ── Fake Odometry (keeps RTAB-Map's main loop alive in offline mode) ──
    fake_odom_node = Node(
        package='robot_bringup', executable='fake_odom_publisher.py',
        name='fake_odom_publisher', output='screen',
    )

    # ── RViz ──
    nav2_rviz_config = PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_navigation.rviz'])
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(enable_rviz),
        arguments=['-d', nav2_rviz_config],
    )

    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(map_to_odom_tf)
    ld.add_action(odom_to_base_footprint_tf)
    ld.add_action(base_footprint_to_base_link_tf)
    ld.add_action(base_link_to_livox_tf)
    ld.add_action(rtabmap_launch)
    ld.add_action(fake_odom_node)
    ld.add_action(rviz_node)
    return ld
