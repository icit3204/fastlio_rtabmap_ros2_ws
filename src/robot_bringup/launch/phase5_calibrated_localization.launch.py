#!/usr/bin/env python3
"""Phase-5 calibrated MID-360, FAST-LIO and RTAB-Map localization authority.

This launch owns only the real sensor/localization side of the no-motion
composition. It does not launch Nav2 command safety or any chassis adapter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Values are operational copies of P5-0C0 calibration authority. Angles passed
# to static_transform_publisher are radians.
CALIBRATED_STATIC_TRANSFORMS = (
    {
        "name": "phase5_body_to_base_footprint",
        "parent": "body",
        "child": "base_footprint",
        "translation": (0.452466, -0.183290, -1.396285),
        "rpy_rad": (0.0, -0.49741883681838395, 0.0),
    },
    {
        "name": "phase5_base_footprint_to_base_link",
        "parent": "base_footprint",
        "child": "base_link",
        "translation": (0.0, 0.0, 0.125),
        "rpy_rad": (0.0, 0.0, 0.0),
    },
    {
        "name": "phase5_base_link_to_livox_frame",
        "parent": "base_link",
        "child": "livox_frame",
        "translation": (0.280, 0.160, 1.362),
        "rpy_rad": (0.0, 0.49741883681838395, 0.0),
    },
)


def _static_transform_node(spec: dict) -> Node:
    x, y, z = spec["translation"]
    roll, pitch, yaw = spec["rpy_rad"]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=spec["name"],
        output="screen",
        arguments=[
            "--x", str(x), "--y", str(y), "--z", str(z),
            "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
            "--frame-id", spec["parent"], "--child-frame-id", spec["child"],
        ],
    )


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    database_path = LaunchConfiguration("database_path")
    boundary_enabled = LaunchConfiguration("startup_freshness_boundary_enabled")
    boundary_lidar_age = LaunchConfiguration("startup_lidar_max_age_sec")
    boundary_imu_age = LaunchConfiguration("startup_imu_max_age_sec")
    boundary_window = LaunchConfiguration("startup_stable_window_sec")

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("livox_ros_driver2"), "launch", "msg_MID360_launch.py"])
        ),
        condition=IfCondition(start_livox),
    )
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("fast_lio"), "launch", "mapping.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config_file": "mid360.yaml",
            "startup_freshness_guard_enabled": "true",
            "startup_max_lidar_age_sec": "0.5",
            "startup_fresh_consecutive_groups": "10",
            "lidar_topic": "/phase5/livox/lidar_fresh",
            "imu_topic": "/phase5/livox/imu_fresh",
            "rviz": "false",
        }.items(),
    )
    freshness_boundary = Node(
        package="phase5_sensor_freshness_boundary",
        executable="sensor_freshness_boundary",
        name="phase5_sensor_freshness_boundary",
        output="screen",
        parameters=[{
            "startup_freshness_boundary_enabled": boundary_enabled,
            "startup_lidar_max_age_sec": boundary_lidar_age,
            "startup_imu_max_age_sec": boundary_imu_age,
            "startup_stable_window_sec": boundary_window,
            "lidar_input_topic": "/livox/lidar",
            "imu_input_topic": "/livox/imu",
            "lidar_output_topic": "/phase5/livox/lidar_fresh",
            "imu_output_topic": "/phase5/livox/imu_fresh",
        }],
    )
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("robot_bringup"), "launch", "rtabmap_bridge.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "sensor_profile": "lidar_only",
            "localization": "true",
            "database_path": database_path,
            "frame_id": "base_footprint",
            "map_frame_id": "map",
            "odom_topic": "/Odometry",
            "imu_topic": "/unused_imu",
            "scan_cloud_topic": "/cloud_registered_body",
            # Explicitly opt in for the measured current MK-mini platform.
            # The generic bridge remains disabled by default for future robots.
            "enable_rtabmap_self_filter": "true",
            "rviz": "false",
            "rtabmap_viz": "false",
            "delete_db_on_start": "false",
        }.items(),
    )

    actions = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "start_livox",
            default_value="true",
            description="Future live admission only; P5-0C1 does not execute this launch.",
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value=EnvironmentVariable("PARKING_ROBOT_RTABMAP_DATABASE", default_value=""),
            description="Explicit accepted RTAB-Map localization database path.",
        ),
        DeclareLaunchArgument("startup_freshness_boundary_enabled", default_value="true"),
        DeclareLaunchArgument("startup_lidar_max_age_sec", default_value="0.5"),
        DeclareLaunchArgument("startup_imu_max_age_sec", default_value="0.05"),
        DeclareLaunchArgument("startup_stable_window_sec", default_value="2.0"),
    ]
    actions.extend(_static_transform_node(spec) for spec in CALIBRATED_STATIC_TRANSFORMS)
    # Establish the boundary subscription before the driver can expose its
    # startup prefix; FAST-LIO may initialize concurrently and remains input-starved
    # until the boundary opens.
    actions.extend([freshness_boundary, fast_lio, livox, rtabmap])
    return LaunchDescription(actions)
