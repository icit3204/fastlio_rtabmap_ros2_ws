"""Standalone launch for only the P4-E.1A generic fake base."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    params = os.path.join(get_package_share_directory("parking_robot_bringup"), "config",
                          "phase4_p4e1a_fake_base.yaml")
    return LaunchDescription([Node(package="parking_robot_bringup",
        executable="phase4_vehicle_cmd_fake_base", name="phase4_vehicle_cmd_fake_base",
        output="screen", parameters=[params])])
