"""Isolated launch_ros execute-mode init seam; never constructs the campaign."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package="parking_robot_bringup", executable="phase4_p4e6b2a2a_init_seam",
        name="phase4_p4e6b_health_failure_runner",
        arguments=["--case-id","B-H01","--output-dir","/tmp/p4e6b2a2a_launch_init",
                   "--execute-authorized-case"],
    )])
