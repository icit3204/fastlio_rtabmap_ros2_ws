"""Phase 4 P4-B isolated Nav2 Collision Monitor command chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("parking_robot_bringup")

    observation_source = LaunchConfiguration("observation_source")
    start_synthetic_fixture = LaunchConfiguration("start_synthetic_fixture")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")
    source_timeout = LaunchConfiguration("source_timeout")
    visualize = LaunchConfiguration("visualize")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

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

    declare_args = [
        DeclareLaunchArgument(
            "observation_source",
            default_value="scan",
            choices=["scan", "points"],
            description="Synthetic Collision Monitor source selection.",
        ),
        DeclareLaunchArgument("start_synthetic_fixture", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("source_timeout", default_value="0.5"),
        DeclareLaunchArgument("visualize", default_value="false"),
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([pkg_share, "maps", "phase2_clean_map.yaml"]),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([pkg_share, "config", "phase2_nav2_params.yaml"]),
        ),
        DeclareLaunchArgument("initial_x", default_value="5.425"),
        DeclareLaunchArgument("initial_y", default_value="-53.725"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params_file, {"yaml_filename": map_file, "use_sim_time": use_sim_time}],
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="phase4_p4b_map_to_odom_static_tf",
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
                "cmd_vel_topic": "/cmd_vel_nav_safe",
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
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", "/cmd_vel_nav_raw")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            nav2_params_file,
            {
                "use_sim_time": use_sim_time,
                "default_nav_to_pose_bt_xml": "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_time.xml",
                "default_nav_through_poses_bt_xml": "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_time.xml",
            },
        ],
    )

    collision_monitor = Node(
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
    )

    navigation_lifecycle_manager = Node(
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
                    "bt_navigator",
                ],
            }
        ],
    )

    collision_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["collision_monitor"],
            }
        ],
    )

    synthetic_fixture = Node(
        package="parking_robot_bringup",
        executable="phase4_p4b_synthetic_obstacles",
        name="phase4_p4b_synthetic_obstacles",
        output="screen",
        condition=IfCondition(start_synthetic_fixture),
        parameters=[
            {
                "mode": "CLEAR",
                "frame_id": "base_footprint",
                "publish_rate_hz": 20.0,
                "publish_scan": PythonExpression(["'", observation_source, "' == 'scan'"]),
                "publish_pointcloud": PythonExpression(["'", observation_source, "' == 'points'"]),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
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
        bt_navigator,
        collision_monitor,
        navigation_lifecycle_manager,
        collision_lifecycle_manager,
        synthetic_fixture,
        rviz,
    ):
        ld.add_action(node)
    return ld
