"""Explicit isolated Phase 2 Nav2 fake-base launch.

This launch file constructs only the Phase 2-safe nodes required for static
P2-B inspection. It must not be used in this task to start runtime nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("parking_robot_bringup")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    declare_args = [
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([pkg_share, "maps", "phase2_clean_map.yaml"]),
            description="Installed Phase 2 static map YAML.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([pkg_share, "config", "phase2_nav2_params.yaml"]),
            description="Installed Phase 2 Nav2 params YAML.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_phase2_mock"),
        DeclareLaunchArgument("initial_x", default_value="5.425"),
        DeclareLaunchArgument("initial_y", default_value="-53.725"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"yaml_filename": map_file, "use_sim_time": use_sim_time}],
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="phase2_map_to_odom_static_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    fake_base = Node(
        package="parking_robot_bringup",
        executable="phase2_fake_base",
        name="phase2_fake_base",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": cmd_vel_topic,
                "odom_topic": "/Odometry",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "publish_rate_hz": 50.0,
                "cmd_timeout_sec": 0.5,
                "max_integration_dt_sec": 0.1,
                "initial_x": initial_x,
                "initial_y": initial_y,
                "initial_yaw": initial_yaw,
            }
        ],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", cmd_vel_topic)],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", cmd_vel_topic)],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": [
                    "map_server",
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }
        ],
    )

    ld = LaunchDescription()
    for arg in declare_args:
        ld.add_action(arg)
    for node in (
        map_server,
        map_to_odom,
        fake_base,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ):
        ld.add_action(node)
    return ld

