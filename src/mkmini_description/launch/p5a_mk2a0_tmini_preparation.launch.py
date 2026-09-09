#!/usr/bin/env python3
"""P5A-MK2A0 T-MINI TF preparation, intentionally inactive.

This launch file contains only the future static-TF hook.  It does not start
the T-MINI driver, does not open a serial device, and does not publish a TF
by default.  The frozen nominal values are present for future qualification;
activation remains a separate operator-controlled decision.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _optional_tmini_tf(context):
    if LaunchConfiguration("enable_tmini_tf").perform(context).lower() != "true":
        return []

    z_text = LaunchConfiguration("scan_plane_z_m").perform(context)
    yaw_text = LaunchConfiguration("yaw_deg").perform(context)
    try:
        z = float(z_text)
        yaw_rad = math.radians(float(yaw_text))
    except ValueError as exc:
        raise RuntimeError(
            "T-MINI TF remains unresolved: provide numeric scan_plane_z_m "
            "and yaw_deg after physical qualification."
        ) from exc

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="p5a_mk2a0_base_link_to_laser_frame",
            output="screen",
            arguments=[
                "--x", LaunchConfiguration("x_m"),
                "--y", LaunchConfiguration("y_m"),
                "--z", str(z),
                "--roll", "0",
                "--pitch", "0",
                "--yaw", str(yaw_rad),
                "--frame-id", "base_link",
                "--child-frame-id", "laser_frame",
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_tmini_tf",
            default_value="false",
            description="Must remain false until physical T-MINI qualification is complete.",
        ),
        DeclareLaunchArgument("x_m", default_value="0.703"),
        DeclareLaunchArgument("y_m", default_value="0.000"),
        DeclareLaunchArgument("scan_plane_z_m", default_value="0.1923"),
        DeclareLaunchArgument("yaw_deg", default_value="0.0"),
        OpaqueFunction(function=_optional_tmini_tf),
    ])
