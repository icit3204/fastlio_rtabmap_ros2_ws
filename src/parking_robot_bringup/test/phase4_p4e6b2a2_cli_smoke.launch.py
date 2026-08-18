"""Test-only isolated launch of the installed runner in no-ROS-traffic dry mode."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package="parking_robot_bringup",
        executable="phase4_p4e6b_health_failure_runner",
        name="p4e6b2a2_cli_smoke",
        arguments=["--case-id", "B-H01", "--output-dir", "/tmp/p4e6b2a2_launch_dry",
                   "--dry-plan-only"],
    )])
