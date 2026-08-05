"""Phase 4 P4-C gate fixture-only preflight launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("vehicle_cmd_safety")
    gate_params = PathJoinSubstitution([pkg_share, "config", "phase4_p4c_gate_mock.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="vehicle_cmd_safety",
                executable="guarded_vehicle_cmd_gate",
                name="guarded_vehicle_cmd_gate",
                output="screen",
                parameters=[gate_params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="vehicle_cmd_safety",
                executable="phase4_p4c_safe_twist_fixture",
                name="phase4_p4c_safe_twist_fixture",
                output="screen",
            ),
            Node(
                package="vehicle_cmd_safety",
                executable="phase4_p4c_permission_fixture",
                name="phase4_p4c_permission_fixture",
                output="screen",
            ),
        ]
    )
