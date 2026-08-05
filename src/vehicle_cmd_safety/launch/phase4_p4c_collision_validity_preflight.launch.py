"""Phase 4 P4-C Collision Monitor validity preflight launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    safety_share = FindPackageShare("vehicle_cmd_safety")
    bringup_share = FindPackageShare("parking_robot_bringup")
    observation_source = LaunchConfiguration("observation_source")
    scan_validity = PathJoinSubstitution([safety_share, "config", "phase4_p4c_collision_validity_scan.yaml"])
    points_validity = PathJoinSubstitution([safety_share, "config", "phase4_p4c_collision_validity_points.yaml"])
    scan_cm = PathJoinSubstitution([bringup_share, "config", "phase4_p4b_collision_monitor_scan.yaml"])
    points_cm = PathJoinSubstitution([bringup_share, "config", "phase4_p4b_collision_monitor_points.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("observation_source", default_value="scan", choices=["scan", "points"]),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="phase4_p4c_odom_to_base_static_tf",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
            ),
            Node(
                package="parking_robot_bringup",
                executable="phase4_p4b_raw_twist_fixture",
                name="phase4_p4b_raw_twist_fixture",
                output="screen",
            ),
            Node(
                package="parking_robot_bringup",
                executable="phase4_p4b_synthetic_obstacles",
                name="phase4_p4b_synthetic_obstacles",
                output="screen",
                parameters=[
                    {
                        "mode": "CLEAR",
                        "frame_id": "base_footprint",
                        "publish_rate_hz": 20.0,
                        "publish_scan": PythonExpression(["'", observation_source, "' == 'scan'"]),
                        "publish_pointcloud": PythonExpression(["'", observation_source, "' == 'points'"]),
                    }
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[
                    IfElseSubstitution(
                        PythonExpression(["'", observation_source, "' == 'points'"]),
                        points_cm,
                        scan_cm,
                    )
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_collision_monitor",
                output="screen",
                parameters=[{"autostart": True, "node_names": ["collision_monitor"]}],
            ),
            Node(
                package="vehicle_cmd_safety",
                executable="collision_monitor_validity_monitor",
                name="collision_monitor_validity_monitor",
                output="screen",
                parameters=[
                    IfElseSubstitution(
                        PythonExpression(["'", observation_source, "' == 'points'"]),
                        points_validity,
                        scan_validity,
                    )
                ],
            ),
        ]
    )
