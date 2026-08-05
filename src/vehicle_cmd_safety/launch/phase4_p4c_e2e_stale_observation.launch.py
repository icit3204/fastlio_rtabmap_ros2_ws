"""Phase 4 P4-C end-to-end stale observation gate launch."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    safety_share = FindPackageShare("vehicle_cmd_safety")
    bringup_share = FindPackageShare("parking_robot_bringup")
    gate_params = PathJoinSubstitution([safety_share, "config", "phase4_p4c_gate_mock.yaml"])
    validity_params = PathJoinSubstitution([safety_share, "config", "phase4_p4c_collision_validity_scan.yaml"])
    collision_params = PathJoinSubstitution([bringup_share, "config", "phase4_p4b_collision_monitor_scan.yaml"])
    return LaunchDescription(
        [
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
                parameters=[{"publish_rate_hz": 20.0, "linear_x": 0.20, "angular_z": 0.20}],
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
                        "publish_scan": True,
                        "publish_pointcloud": False,
                    }
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[collision_params],
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
                parameters=[validity_params],
            ),
            Node(
                package="vehicle_cmd_safety",
                executable="guarded_vehicle_cmd_gate",
                name="guarded_vehicle_cmd_gate",
                output="screen",
                parameters=[gate_params],
            ),
            Node(
                package="vehicle_cmd_safety",
                executable="phase4_p4c_permission_fixture",
                name="phase4_p4c_permission_fixture",
                output="screen",
                parameters=[{"publish_collision": False}],
            ),
        ]
    )
