#!/usr/bin/env python3
"""MK-mini MID-360/FAST-LIO localization with one calibration authority.

``mkmini_platform_profile.yaml`` is the only numeric MID-360 mounting
authority. This launch renders the robot description and derives both
``body -> base_footprint`` and ``odom -> odom_chassis`` from that profile.
It excludes RTAB-Map, Nav2, Collision Monitor, command adapters and CAN.
"""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_vec(rotation, vector):
    return [sum(rotation[i][j] * vector[j] for j in range(3)) for i in range(3)]


def _transpose(rotation):
    return [[rotation[j][i] for j in range(3)] for i in range(3)]


def _matrix_rpy(rotation):
    return (
        math.atan2(rotation[2][1], rotation[2][2]),
        math.asin(-rotation[2][0]),
        math.atan2(rotation[1][0], rotation[0][0]),
    )


def load_current_frame_authority(mkmini_share):
    """Load the single profile and derive every current MID-360 TF value."""
    share = Path(mkmini_share)
    profile_path = share / "config" / "mkmini_platform_profile.yaml"
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if profile["status"] != "MKMINI_MID360_CURRENT_YAW_AUTHORITY_MK2C1R3":
        raise RuntimeError(f"Unexpected MID-360 authority status in {profile_path}")

    native = profile["livox_native_transform"]
    xyz = [native[f"{axis}_m"]["value"] for axis in "xyz"]
    rpy = [math.radians(native[f"{axis}_deg"]["value"])
           for axis in ("roll", "pitch", "yaw")]
    rotation = _rpy_matrix(*rpy)

    contract = profile["fast_lio_body_contract"]
    lidar_to_body = contract["lidar_to_imu_translation_m"]
    if contract["lidar_to_imu_rotation"] != [1.0, 0.0, 0.0,
                                               0.0, 1.0, 0.0,
                                               0.0, 0.0, 1.0]:
        raise RuntimeError("Launch derivation currently requires the qualified identity R_L_I")

    # p_base_link = R_base_livox p_livox + t_base_livox
    # p_body = p_livox + t_lidar_to_body
    # therefore t_base_link_body = t_base_link_livox - R_base_livox*t_lidar_to_body.
    rotated_internal_translation = _mat_vec(rotation, lidar_to_body)
    base_link_to_body_t = [xyz[i] - rotated_internal_translation[i] for i in range(3)]
    base_link_z = profile["transforms"]["base_footprint_to_base_link"]["translation_m"][2]
    base_footprint_to_body_t = [base_link_to_body_t[0], base_link_to_body_t[1],
                                base_link_to_body_t[2] + base_link_z]
    body_to_base_rotation = _transpose(rotation)
    body_to_base_t = [-value for value in _mat_vec(body_to_base_rotation,
                                                    base_footprint_to_body_t)]
    body_to_base_rpy = _matrix_rpy(body_to_base_rotation)

    template = (share / "urdf" / "mkmini_frames.urdf.template").read_text(encoding="utf-8")
    robot_description = (template
        .replace("@BASE_LINK_Z@", f"{base_link_z:.12g}")
        .replace("@FRONT_AXLE_X@", f"{profile['geometry']['manufacturer_nominal']['wheelbase_m']['value']:.12g}")
        .replace("@LIVOX_XYZ@", " ".join(f"{v:.12g}" for v in xyz))
        .replace("@LIVOX_RPY_RAD@", " ".join(f"{v:.12g}" for v in rpy)))
    if "@" in robot_description:
        raise RuntimeError("Unresolved token in MK-mini URDF template")

    return {
        "profile_path": str(profile_path),
        "robot_description": robot_description,
        "base_link_to_livox": {"translation": tuple(xyz), "rpy_rad": tuple(rpy)},
        "base_footprint_to_body": {
            "translation": tuple(base_footprint_to_body_t),
            "rotation": tuple(tuple(value for value in row) for row in rotation),
            "rpy_rad": tuple(rpy)},
        "body_to_base_footprint": {
            "translation": tuple(body_to_base_t), "rpy_rad": tuple(body_to_base_rpy)},
        }


def _static_node(name, parent, child, transform, condition=None):
    x, y, z = transform["translation"]
    roll, pitch, yaw = transform["rpy_rad"]
    return Node(
        package="tf2_ros", executable="static_transform_publisher", name=name,
        output="screen",
        condition=IfCondition(condition) if condition is not None else None,
        arguments=[
            "--x", str(x), "--y", str(y), "--z", str(z),
            "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
            "--frame-id", parent, "--child-frame-id", child,
        ],
    )


def _conditional_static_node(name, parent, child, transform, condition):
    return _static_node(name, parent, child, transform, condition)


def generate_launch_description() -> LaunchDescription:
    mkmini_share = get_package_share_directory("mkmini_description")
    authority = load_current_frame_authority(mkmini_share)
    robot_description = ParameterValue(authority["robot_description"], value_type=str)

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    boundary_enabled = LaunchConfiguration("startup_freshness_boundary_enabled")
    lidar_age = LaunchConfiguration("startup_lidar_max_age_sec")
    imu_age = LaunchConfiguration("startup_imu_max_age_sec")
    stable_window = LaunchConfiguration("startup_stable_window_sec")

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("livox_ros_driver2"), "launch", "msg_MID360_launch.py"])),
        condition=IfCondition(start_livox),
    )
    freshness = Node(
        package="phase5_sensor_freshness_boundary",
        executable="sensor_freshness_boundary", name="mkmini_sensor_freshness_boundary",
        output="screen",
        parameters=[{
            "startup_freshness_boundary_enabled": boundary_enabled,
            "startup_lidar_max_age_sec": lidar_age,
            "startup_imu_max_age_sec": imu_age,
            "startup_stable_window_sec": stable_window,
            "lidar_input_topic": "/livox/lidar", "imu_input_topic": "/livox/imu",
            "lidar_output_topic": "/phase5/livox/lidar_fresh",
            "imu_output_topic": "/phase5/livox/imu_fresh",
            "diagnostics_topic": "/phase5/mkmini/livox_freshness_status",
        }],
    )
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("fast_lio"), "launch", "mapping.launch.py"])),
        condition=IfCondition(start_fast_lio),
        launch_arguments={
            "use_sim_time": use_sim_time, "config_file": "mid360.yaml",
            "startup_freshness_guard_enabled": "true",
            "startup_max_lidar_age_sec": "0.5",
            "startup_fresh_consecutive_groups": "10",
            "lidar_topic": "/phase5/livox/lidar_fresh",
            "imu_topic": "/phase5/livox/imu_fresh", "rviz": "false",
        }.items(),
    )
    robot_state = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="mkmini_robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )
    body_to_base = authority["body_to_base_footprint"]
    base_to_body = authority["base_footprint_to_body"]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_livox", default_value="true"),
        DeclareLaunchArgument("start_fast_lio", default_value="true"),
        DeclareLaunchArgument("startup_freshness_boundary_enabled", default_value="true"),
        DeclareLaunchArgument("startup_lidar_max_age_sec", default_value="0.5"),
        DeclareLaunchArgument("startup_imu_max_age_sec", default_value="0.05"),
        DeclareLaunchArgument("startup_stable_window_sec", default_value="2.0"),
        robot_state,
        _conditional_static_node("mkmini_body_to_base_footprint", "body", "base_footprint", body_to_base, start_fast_lio),
        # RTAB-Map publishes map->odom_chassis when it consumes the native
        # chassis TF odometry.  Making odom_chassis the parent of raw FAST-LIO
        # odom keeps one parent per frame and preserves the qualified planar
        # composition: odom_chassis->base_footprint = B^-1 * odom->body * B.
        _conditional_static_node("mkmini_odom_chassis_to_odom", "odom_chassis", "odom", base_to_body, start_fast_lio),
        freshness, fast_lio, livox,
    ])
