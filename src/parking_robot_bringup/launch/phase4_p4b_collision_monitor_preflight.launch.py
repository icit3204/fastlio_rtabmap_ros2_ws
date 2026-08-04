"""Phase 4 P4-B Collision Monitor fixture-only preflight launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("parking_robot_bringup")
    observation_source = LaunchConfiguration("observation_source")
    use_sim_time = LaunchConfiguration("use_sim_time")
    source_timeout = LaunchConfiguration("source_timeout")
    visualize = LaunchConfiguration("visualize")

    scan_params = PathJoinSubstitution(
        [pkg_share, "config", "phase4_p4b_collision_monitor_scan.yaml"]
    )
    points_params = PathJoinSubstitution(
        [pkg_share, "config", "phase4_p4b_collision_monitor_points.yaml"]
    )
    collision_params = IfElseSubstitution(
        PythonExpression(["'", observation_source, "' == 'points'"]),
        points_params,
        scan_params,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("observation_source", default_value="scan", choices=["scan", "points"]),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("source_timeout", default_value="0.5"),
            DeclareLaunchArgument("visualize", default_value="false"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="phase4_p4b_odom_to_base_static_tf",
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
                    collision_params,
                    {
                        "use_sim_time": use_sim_time,
                        "source_timeout": source_timeout,
                        "PolygonStop.visualize": visualize,
                        "PolygonSlow.visualize": visualize,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_collision_monitor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["collision_monitor"],
                    }
                ],
            ),
        ]
    )
