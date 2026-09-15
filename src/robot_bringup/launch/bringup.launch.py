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

import hashlib
import json
import os
import shutil
import tempfile
import time
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
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
    start_rtabmap = LaunchConfiguration('start_rtabmap')
    start_ydlidar = LaunchConfiguration('start_ydlidar')  # <修改 version3 YDLIDAR 2D雷达支持>
    use_fast_lio = LaunchConfiguration('use_fast_lio')    # <修改 version3 控制是否使用FAST-LIO里程计>
    use_fake_odom = LaunchConfiguration('use_fake_odom')   # <修改 version3 假里程计模拟模式>
    enable_gps = LaunchConfiguration('enable_gps')
    enable_rviz = LaunchConfiguration('enable_rviz')
    stationary_integration_gate = LaunchConfiguration('stationary_integration_gate')
    stationary_collision_monitor_gate = LaunchConfiguration('stationary_collision_monitor_gate')
    stationary_fail_close_gate = LaunchConfiguration('stationary_fail_close_gate')
    stationary_command_chain_dry_run = LaunchConfiguration('stationary_command_chain_dry_run')
    stationary_mppi_mock_gate = LaunchConfiguration('stationary_mppi_mock_gate')
    stationary_planner_mock_gate = LaunchConfiguration('stationary_planner_mock_gate')
    stationary_bt_navigator_mock_gate = LaunchConfiguration('stationary_bt_navigator_mock_gate')
    stationary_mission_manager_mock_gate = LaunchConfiguration('stationary_mission_manager_mock_gate')
    mission_manager_expected_topology_version = LaunchConfiguration('mission_manager_expected_topology_version')
    stationary_real_localization_validity = LaunchConfiguration('stationary_real_localization_validity')
    database_path = LaunchConfiguration('database_path')
    rtabmap_use_working_copy = LaunchConfiguration('rtabmap_use_working_copy')
    rtabmap_runtime_dir = LaunchConfiguration('rtabmap_runtime_dir')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    lookahead_distance = LaunchConfiguration('lookahead_distance')  # <修改 version2 预瞄点距离>
    scan_topic = LaunchConfiguration('scan_topic')  # <修改 version3 YDLIDAR 2D雷达scan话题>

    robot_bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('mode', default_value='navigation', description='mapping | localization | navigation'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_only', description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_livox', default_value='true', description='Start Livox MID360 launch'),
        DeclareLaunchArgument('start_rtabmap', default_value='true', description='Start the canonical RTAB-Map bridge when that localization/map owner is required'),
        # <修改 version3 YDLIDAR 2D雷达支持>
        DeclareLaunchArgument('start_ydlidar', default_value='true', description='Start qualified YDLIDAR T-mini Plus driver'),
        DeclareLaunchArgument('use_fast_lio', default_value='true', description='Use FAST-LIO for odometry (3D LiDAR). Set false for 2D LiDAR + RTAB-Map ICP odometry'),
        # <修改 version3 假里程计模式: 无机器人时模拟运动>
        DeclareLaunchArgument('use_fake_odom', default_value='false', description='Use fake odometry for simulation without a robot'),
        DeclareLaunchArgument('ydlidar_params_file', default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'tmini_mk2c.yaml']), description='Qualified YDLIDAR parameter file'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='LaserScan topic from 2D LiDAR'),
        DeclareLaunchArgument('enable_gps', default_value='false', description='Enable navsat_transform and pass GPS fix to RTAB-Map'),
        DeclareLaunchArgument('enable_rviz', default_value='false', description='Launch RViz with Nav2 navigation config'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='false', description='Deprecated compatibility argument; calibrated robot_state_publisher owns this TF'),
        DeclareLaunchArgument('stationary_integration_gate', default_value='false', description='Start the production local costmap while suppressing Nav2 control, BT, Collision Monitor and command producers'),
        DeclareLaunchArgument('stationary_collision_monitor_gate', default_value='false', description='Add the production Collision Monitor with isolated Phase-5 command topics while suppressing all motion-capable nodes'),
        DeclareLaunchArgument('stationary_fail_close_gate', default_value='false', description='Add dual-source freshness validity and the Generic Safety Gate on isolated Phase-5 topics'),
        DeclareLaunchArgument('stationary_command_chain_dry_run', default_value='false', description='Run CM -> Gate -> labmate-derived MK-mini backend with hard-locked MockTransport and no control nodes'),
        DeclareLaunchArgument('stationary_mppi_mock_gate', default_value='false', description='Add real controller_server/MPPI above the canonical safety chain; hard-locked MockTransport, no BT/planner/CAN'),
        DeclareLaunchArgument('stationary_planner_mock_gate', default_value='false', description='Add real planner_server plus MPPI above the canonical safety chain; hard-locked MockTransport, no BT/CAN'),
        DeclareLaunchArgument('stationary_bt_navigator_mock_gate', default_value='false', description='Add real BT Navigator/NavigateToPose above planner and MPPI; Ackermann-safe recovery tree, no behavior_server, hard-locked MockTransport'),
        DeclareLaunchArgument('stationary_mission_manager_mock_gate', default_value='false', description='Add the typed Mission Manager above real NavigateToPose; hard-locked MockTransport and no CAN'),
        DeclareLaunchArgument(
            'mission_manager_expected_topology_version', default_value='v1',
            description='Exact topology version accepted by Mission Manager; defaults to the qualified synthetic-test value'),
        DeclareLaunchArgument(
            'stationary_real_localization_validity', default_value='false',
            description='Replace only the stationary localization permission fixture with the qualified live localization-validity monitor; controller permission remains mock-only'),
        DeclareLaunchArgument('collision_monitor_params_file', default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'collision_monitor_dual_sensor.yaml']), description='Canonical dual-sensor Collision Monitor parameters'),
        DeclareLaunchArgument(
            'database_path',
            default_value=EnvironmentVariable(
                'PARKING_ROBOT_RTABMAP_DATABASE',
                default_value='/home/dog/fastlio_rtabmap_ros2_ws/map/rtabmap_2d.db'),
            description='Reference RTAB-Map database. Localization copies this file before RTAB-Map opens it.',
        ),
        DeclareLaunchArgument(
            'rtabmap_use_working_copy', default_value='true',
            description='For localization/navigation, copy database_path into rtabmap_runtime_dir before RTAB-Map starts.'),
        DeclareLaunchArgument(
            'rtabmap_runtime_dir', default_value='/home/dog/phase5_runtime/rtabmap',
            description='Persistent per-run RTAB-Map localization working-copy directory.'),
        # The current calibrated body/base transform defines a planar
        # ``odom_chassis`` authority for RTAB-Map.  RTAB's native TF odometry
        # mode must use it instead of interpreting FAST-LIO's sensor-oriented
        # ``odom`` basis as a horizontal chassis frame.  Ray tracing is needed
        # to persist observed free space from the 3-D scan cloud.
        DeclareLaunchArgument('rtabmap_args', default_value='--Grid/RayTracing true'),
        DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_common.yaml'])),
        DeclareLaunchArgument('rtabmap_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('rtabmap_map_frame', default_value='map'),
        DeclareLaunchArgument('rtabmap_odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('rtabmap_odom_frame_id', default_value='odom_chassis'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/unused_imu',
            description='Optional IMU topic for navsat_transform when GPS is enabled.',
        ),
        # Keep RTAB-Map's optional gravity input independent of FAST-LIO's
        # ``imu_topic`` launch argument. Nested launch descriptions share
        # launch-configuration names; reusing ``imu_topic`` let FAST-LIO's
        # sensor-frame IMU silently become RTAB-Map's map-correction input.
        DeclareLaunchArgument(
            'rtabmap_imu_topic',
            default_value='/unused_imu',
            description='Optional RTAB-Map IMU topic; disabled by default for the calibrated chassis mapping path.',
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
        name='main_base_link_to_laser_frame',
        arguments=['--x', '0.703', '--y', '0', '--z', '0.1923', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
        condition=IfCondition(start_ydlidar),
    )

    # Qualified R3 sensor/localization authority owns MID-360, freshness,
    # FAST-LIO, robot description and dependent body/chassis TFs.
    calibrated_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_bringup_share, 'launch', 'mkmini_calibrated_localization.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_livox': start_livox,
            'start_fast_lio': use_fast_lio,
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
    rtabmap_bridge_launch = IncludeLaunchDescription(
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
            'odom_frame_id': LaunchConfiguration('rtabmap_odom_frame_id'),
            'rtabmap_imu_topic': LaunchConfiguration('rtabmap_imu_topic'),
            'gps_topic': LaunchConfiguration('gps_fix_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            # TEMPORARY_MKMINI_SELF_BODY_CROP: opt-in only for the current
            # measured MK-mini platform. The generic bridge defaults false.
            'enable_rtabmap_self_filter': 'true',
            # <修改 version3 YDLIDAR 2D雷达支持>
            'use_fast_lio': use_fast_lio,
            'use_fake_odom': use_fake_odom,
            'scan_topic': scan_topic,
            'rviz': 'false',
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
        }.items(),
    )
    # RTAB-Map remains default-on for the normal production launch.  The
    # stationary command-chain dry run can explicitly suppress this optional
    # map/localization bridge while it exercises the authoritative FAST-LIO
    # TF, costmap, Collision Monitor and command safety chain.
    rtabmap_bridge = GroupAction(
        condition=IfCondition(LaunchConfiguration('start_rtabmap')),
        actions=[rtabmap_bridge_launch],
    )

    def prepare_rtabmap_working_copy(context):
        """Keep the map reference immutable when RTAB-Map localizes.

        RTAB-Map may repair dictionaries or otherwise write a localization DB.
        Mapping is intentionally excluded: its selected path is the new map
        being authored. Localization/navigation always receives a fresh,
        hash-verified copy and leaves the reference untouched.
        """
        if start_rtabmap.perform(context).lower() not in ('true', '1', 'yes'):
            return []
        if mode.perform(context) == 'mapping':
            return []
        if rtabmap_use_working_copy.perform(context).lower() not in ('true', '1', 'yes'):
            return []
        def sha256_file(path):
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(block)
            return digest.hexdigest()

        reference = Path(database_path.perform(context)).expanduser().resolve()
        if not reference.is_file():
            raise RuntimeError(f'RTAB-Map reference database is not a file: {reference}')
        reference_hash = sha256_file(reference)
        runtime_dir = Path(rtabmap_runtime_dir.perform(context)).expanduser()
        runtime_dir.mkdir(parents=True, exist_ok=True)
        run_id = time.strftime('%Y%m%dT%H%M%S') + f'_pid{os.getpid()}'
        runtime_db = runtime_dir / f'{reference.stem}_localization_{run_id}{reference.suffix}'
        shutil.copy2(reference, runtime_db)
        runtime_hash = sha256_file(runtime_db)
        if runtime_hash != reference_hash:
            raise RuntimeError('RTAB-Map working-copy hash mismatch; refusing localization startup')
        provenance = runtime_db.with_suffix(runtime_db.suffix + '.provenance.json')
        provenance.write_text(json.dumps({
            'reference_db': str(reference),
            'reference_sha256': reference_hash,
            'runtime_db': str(runtime_db),
            'runtime_sha256_before_rtabmap': runtime_hash,
            'policy': 'preserve runtime DB after shutdown; never delete or modify reference automatically',
        }, indent=2) + '\\n')
        context.launch_configurations['database_path'] = str(runtime_db)
        print(f'[robot_bringup] RTAB-Map localization working copy: {runtime_db}')
        return []

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
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation' and '", stationary_integration_gate, "' != 'true' and '", stationary_collision_monitor_gate, "' != 'true' and '", stationary_fail_close_gate, "' != 'true' and '", stationary_command_chain_dry_run, "' != 'true' and '", stationary_mppi_mock_gate, "' != 'true' and '", stationary_planner_mock_gate, "' != 'true' and '", stationary_bt_navigator_mock_gate, "' != 'true' and '", stationary_mission_manager_mock_gate, "' != 'true'"])),
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

    def stationary_local_costmap_actions(context):
        local_only = stationary_integration_gate.perform(context).lower() in ('true', '1', 'yes')
        with_cm = stationary_collision_monitor_gate.perform(context).lower() in ('true', '1', 'yes')
        with_gate = stationary_fail_close_gate.perform(context).lower() in ('true', '1', 'yes')
        with_chain = stationary_command_chain_dry_run.perform(context).lower() in ('true', '1', 'yes')
        with_mppi = stationary_mppi_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_planner = stationary_planner_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_bt = (stationary_bt_navigator_mock_gate.perform(context).lower() in ('true', '1', 'yes')
                   or stationary_mission_manager_mock_gate.perform(context).lower() in ('true', '1', 'yes'))
        if mode.perform(context) != 'navigation' or not (local_only or with_cm or with_gate or with_chain or with_mppi or with_planner or with_bt):
            return []
        # ControllerServer owns and spins its own local_costmap instance.
        if with_mppi or with_planner or with_bt:
            return []
        production = Path(nav2_params_file.perform(context))
        source = yaml.safe_load(production.read_text(encoding='utf-8'))
        params = source['local_costmap']['local_costmap']['ros__parameters']
        generated = Path(tempfile.gettempdir()) / 'p5a_mk2c3_main_local_costmap_effective.yaml'
        generated.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}, sort_keys=False), encoding='utf-8')
        return [
            Node(package='nav2_costmap_2d', executable='nav2_costmap_2d',
                 name='main_pipeline_costmap_host', output='screen', parameters=[str(generated)]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_main_pipeline_local_costmap', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'bond_timeout': 0.0, 'node_names': ['costmap/costmap']}]),
        ]

    def stationary_collision_monitor_actions(context):
        cm_only = stationary_collision_monitor_gate.perform(context).lower() in ('true', '1', 'yes')
        with_gate = stationary_fail_close_gate.perform(context).lower() in ('true', '1', 'yes')
        with_chain = stationary_command_chain_dry_run.perform(context).lower() in ('true', '1', 'yes')
        with_mppi = stationary_mppi_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_planner = stationary_planner_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_bt = (stationary_bt_navigator_mock_gate.perform(context).lower() in ('true', '1', 'yes')
                   or stationary_mission_manager_mock_gate.perform(context).lower() in ('true', '1', 'yes'))
        if not (cm_only or with_gate or with_chain or with_mppi or with_planner or with_bt):
            return []
        production = Path(LaunchConfiguration('collision_monitor_params_file').perform(context))
        source = yaml.safe_load(production.read_text(encoding='utf-8'))
        params = source['collision_monitor']['ros__parameters']
        if with_mppi or with_planner or with_bt:
            params['cmd_vel_in_topic'] = '/cmd_vel_nav'
            params['cmd_vel_out_topic'] = '/cmd_vel'
        else:
            params['cmd_vel_in_topic'] = '/phase5/cm_test/cmd_vel_in'
            params['cmd_vel_out_topic'] = '/phase5/cm_test/cmd_vel_out'
        generated = Path(tempfile.gettempdir()) / 'p5a_mk2d1_collision_monitor_isolated.yaml'
        generated.write_text(
            yaml.safe_dump({'/**': {'ros__parameters': params}}, sort_keys=False),
            encoding='utf-8')
        return [
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', output='screen', parameters=[str(generated)]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_stationary_collision_monitor', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'service_call_timeout': 10.0,
                              'node_names': ['collision_monitor']}]),
        ]

    def stationary_fail_close_actions(context):
        with_gate = stationary_fail_close_gate.perform(context).lower() in ('true', '1', 'yes')
        with_chain = stationary_command_chain_dry_run.perform(context).lower() in ('true', '1', 'yes')
        with_mppi = stationary_mppi_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_planner = stationary_planner_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_bt = (stationary_bt_navigator_mock_gate.perform(context).lower() in ('true', '1', 'yes')
                   or stationary_mission_manager_mock_gate.perform(context).lower() in ('true', '1', 'yes'))
        if not (with_gate or with_chain or with_mppi or with_planner or with_bt):
            return []
        real_localization = stationary_real_localization_validity.perform(context).lower() in ('true', '1', 'yes')
        safety_share = Path(os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0]) / 'share' / 'vehicle_cmd_safety'
        if not safety_share.exists():
            safety_share = Path('/home/dog/fastlio_rtabmap_ros2_ws/install/vehicle_cmd_safety/share/vehicle_cmd_safety')
        config = safety_share / 'config'
        actions = [
            Node(package='vehicle_cmd_safety', executable='collision_monitor_validity_monitor',
                 name='mid360_collision_validity', output='screen',
                 parameters=[str(config / 'phase5_dual_mid360_validity.yaml')]),
            Node(package='vehicle_cmd_safety', executable='collision_monitor_validity_monitor',
                 name='tmini_collision_validity', output='screen',
                 parameters=[str(config / 'phase5_dual_tmini_validity.yaml')]),
            Node(package='vehicle_cmd_safety', executable='required_perception_validity',
                 name='required_perception_validity', output='screen',
                 parameters=[{'input_timeout_sec': 0.25, 'recovery_consecutive_ticks': 3, 'heartbeat_hz': 20.0}]),
            Node(package='vehicle_cmd_safety', executable='phase4_p4c_permission_fixture',
                 name='stationary_gate_permission_fixture', output='screen',
                 parameters=[{'publish_localization': not real_localization, 'publish_controller': True,
                              'publish_collision': False, 'publish_rate_hz': 20.0}]),
            Node(package='vehicle_cmd_safety', executable='guarded_vehicle_cmd_gate',
                 name='guarded_vehicle_cmd_gate', output='screen',
                 parameters=[str(config / 'phase5_dual_fail_close_gate.yaml'),
                             ({'safe_input_topic': '/cmd_vel',
                               'output_topic': '/vehicle_cmd_safe'} if (with_mppi or with_planner or with_bt) else
                              {'output_topic': '/vehicle_cmd_safe'} if with_chain else {})]),
        ]
        if real_localization:
            actions.insert(3, Node(
                package='vehicle_cmd_safety', executable='localization_validity_monitor',
                name='localization_validity_monitor', output='screen',
                parameters=[str(config / 'phase5_localization_validity.yaml')]))
        return actions

    def stationary_command_chain_actions(context):
        with_chain = stationary_command_chain_dry_run.perform(context).lower() in ('true', '1', 'yes')
        with_mppi = stationary_mppi_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_planner = stationary_planner_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_bt = (stationary_bt_navigator_mock_gate.perform(context).lower() in ('true', '1', 'yes')
                   or stationary_mission_manager_mock_gate.perform(context).lower() in ('true', '1', 'yes'))
        if not (with_chain or with_mppi or with_planner or with_bt):
            return []
        return [
            Node(package='wheelchair_cmd_adapter', executable='gate_to_labmate_bridge',
                 name='gate_to_labmate_bridge', output='screen',
                 parameters=[PathJoinSubstitution([
                     FindPackageShare('wheelchair_cmd_adapter'), 'config',
                     'gate_to_labmate_bridge.yaml'])]),
            # Hard lock: there is no launch substitution or argument capable
            # of selecting physical CAN in this stationary qualification mode.
            Node(package='wheelchair_controller', executable='wheelchair_controller_node',
                 name='wheelchair_controller_node', output='screen',
                 parameters=[PathJoinSubstitution([
                     FindPackageShare('wheelchair_controller'), 'config',
                     'wheelchair_controller_param.yaml']),
                     {'output_transport': 'mock', 'auto_start': True,
                      'can_interface': 'CANONICAL_MOCK_FORBIDDEN'}]),
        ]

    def stationary_mppi_controller_actions(context):
        with_mppi = stationary_mppi_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_planner = stationary_planner_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_bt = (stationary_bt_navigator_mock_gate.perform(context).lower() in ('true', '1', 'yes')
                   or stationary_mission_manager_mock_gate.perform(context).lower() in ('true', '1', 'yes'))
        if not (with_mppi or with_planner or with_bt):
            return []
        if mode.perform(context) != 'navigation':
            return []
        # ControllerServer configures its internal local costmap immediately.
        # Give the real sensor/localization branch time to establish the
        # odom_chassis -> base_footprint TF before lifecycle configuration;
        # the safety chain is already running fail-closed during this delay.
        return [TimerAction(period=10.0, actions=[
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server', output='screen',
                 parameters=[nav2_params_file],
                 remappings=[('/cmd_vel', '/cmd_vel_nav')]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_stationary_mppi', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'bond_timeout': 0.0,
                              'service_call_timeout': 10.0,
                              'node_names': ['controller_server']}]),
        ])]

    def stationary_planner_actions(context):
        with_planner = stationary_planner_mock_gate.perform(context).lower() in ('true', '1', 'yes')
        with_bt = (stationary_bt_navigator_mock_gate.perform(context).lower() in ('true', '1', 'yes')
                   or stationary_mission_manager_mock_gate.perform(context).lower() in ('true', '1', 'yes'))
        if not (with_planner or with_bt):
            return []
        if mode.perform(context) != 'navigation':
            return []
        # RTAB-Map is the canonical map -> odom and /map authority. Delay
        # planner lifecycle activation until it has established map/TF and the
        # static-layer global costmap can consume the actual map.
        return [TimerAction(period=15.0, actions=[
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', output='screen', parameters=[nav2_params_file]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_stationary_planner', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'bond_timeout': 0.0,
                              'service_call_timeout': 10.0,
                              'node_names': ['planner_server']}]),
        ])]

    def stationary_bt_navigator_actions(context):
        if (stationary_bt_navigator_mock_gate.perform(context).lower() not in ('true', '1', 'yes')
                and stationary_mission_manager_mock_gate.perform(context).lower() not in ('true', '1', 'yes')):
            return []
        if mode.perform(context) != 'navigation':
            return []
        # The current chassis is forward-only Ackermann.  This BT uses only
        # planner/controller actions and costmap-clearing service recoveries.
        # behavior_server is intentionally absent: Humble's Spin/BackUp/
        # DriveOnHeading plugins publish cmd_vel below Collision Monitor.
        safe_bt = PathJoinSubstitution([
            robot_bringup_share, 'config', 'ackermann_navigate_to_pose.xml'])
        return [TimerAction(period=20.0, actions=[
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen',
                 parameters=[nav2_params_file,
                             {'default_nav_to_pose_bt_xml': safe_bt}]),
            # Do not race the lifecycle Configure request with creation of the
            # BT Navigator action servers.  Humble can leave a successfully
            # configured navigator INACTIVE when both processes are spawned in
            # the same launch tick and the first change-state response races
            # DDS service discovery.  The delayed manager remains the sole
            # lifecycle authority; it merely starts after the node is ready.
            TimerAction(period=2.0, actions=[
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_stationary_bt_navigator', output='screen',
                     parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                                  'bond_timeout': 0.0,
                                  'service_call_timeout': 10.0,
                                  'node_names': ['bt_navigator']}]),
            ]),
        ])]

    def stationary_mission_manager_actions(context):
        if stationary_mission_manager_mock_gate.perform(context).lower() not in ('true', '1', 'yes'):
            return []
        if mode.perform(context) != 'navigation':
            return []
        # The manager owns mission-driven NavigateToPose goals only. Its
        # progress observer watches the canonical Phase-5 chain; it publishes
        # no velocity command and has no transport below MockTransport.
        return [TimerAction(period=23.0, actions=[
            Node(package='parking_robot_mission_manager', executable='mission_manager_node',
                 name='mission_manager', output='screen', parameters=[{
                     'expected_topology_version': mission_manager_expected_topology_version,
                     'navigate_to_pose_action': '/navigate_to_pose',
                     'odometry_topic': '/Odometry',
                     'raw_command_topic': '/cmd_vel_nav',
                     'safe_command_topic': '/cmd_vel',
                     'gate_state_topic': '/phase5/gate_test/state',
                     'collision_valid_topic': '/system/collision_monitor_valid',
                     'localization_valid_topic': '/system/localization_valid',
                     'controller_valid_topic': '/system/controller_valid',
                     'adapter_diagnostics_topic': '/gate_to_labmate_bridge/diagnostics',
                     'progress_tf_frame': 'odom_chassis',
                     'progress_base_frame': 'base_footprint',
                 }]),
        ])]

    # ── 7. Collision Monitor ──
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation' and '", stationary_integration_gate, "' != 'true' and '", stationary_collision_monitor_gate, "' != 'true' and '", stationary_fail_close_gate, "' != 'true' and '", stationary_command_chain_dry_run, "' != 'true' and '", stationary_mppi_mock_gate, "' != 'true' and '", stationary_planner_mock_gate, "' != 'true' and '", stationary_bt_navigator_mock_gate, "' != 'true' and '", stationary_mission_manager_mock_gate, "' != 'true'"])),
        parameters=[LaunchConfiguration('collision_monitor_params_file'), {'use_sim_time': use_sim_time}],
    )

    collision_monitor_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation' and '", stationary_integration_gate, "' != 'true' and '", stationary_collision_monitor_gate, "' != 'true' and '", stationary_fail_close_gate, "' != 'true' and '", stationary_command_chain_dry_run, "' != 'true' and '", stationary_mppi_mock_gate, "' != 'true' and '", stationary_planner_mock_gate, "' != 'true' and '", stationary_bt_navigator_mock_gate, "' != 'true' and '", stationary_mission_manager_mock_gate, "' != 'true'"])),
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
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation' and '", stationary_integration_gate, "' != 'true' and '", stationary_collision_monitor_gate, "' != 'true' and '", stationary_fail_close_gate, "' != 'true' and '", stationary_command_chain_dry_run, "' != 'true' and '", stationary_mppi_mock_gate, "' != 'true' and '", stationary_planner_mock_gate, "' != 'true' and '", stationary_bt_navigator_mock_gate, "' != 'true' and '", stationary_mission_manager_mock_gate, "' != 'true'"])),
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
    ld.add_action(calibrated_localization)
    # <修改 version3 YDLIDAR 2D雷达节点>
    ld.add_action(ydlidar_node)
    ld.add_action(ydlidar_tf)
    ld.add_action(navsat_transform)
    ld.add_action(OpaqueFunction(function=prepare_rtabmap_working_copy))
    ld.add_action(rtabmap_bridge)
    ld.add_action(rviz_node)
    ld.add_action(nav2_launch)
    ld.add_action(OpaqueFunction(function=stationary_local_costmap_actions))
    ld.add_action(collision_monitor)
    ld.add_action(collision_monitor_lifecycle)
    ld.add_action(OpaqueFunction(function=stationary_collision_monitor_actions))
    ld.add_action(OpaqueFunction(function=stationary_fail_close_actions))
    ld.add_action(OpaqueFunction(function=stationary_command_chain_actions))
    ld.add_action(OpaqueFunction(function=stationary_mppi_controller_actions))
    ld.add_action(OpaqueFunction(function=stationary_planner_actions))
    ld.add_action(OpaqueFunction(function=stationary_bt_navigator_actions))
    ld.add_action(OpaqueFunction(function=stationary_mission_manager_actions))
    ld.add_action(preview_point)  # <修改 version2 预瞄点>
    return ld
