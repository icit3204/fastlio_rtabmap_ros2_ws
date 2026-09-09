#!/usr/bin/env python3
"""P5A-MK2C stationary, no-motion local-costmap observation stack.

The only Nav2 server launched here is the real ``nav2_costmap_2d`` local
costmap, parameterized directly from the current production
``nav2_common.yaml``.  It deliberately starts no controller, planner, BT
navigator, velocity smoother, Collision Monitor, vehicle adapter, CAN writer,
or Twist publisher.
"""

from launch import LaunchDescription
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_stack = LaunchConfiguration("start_stack")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_tmini = LaunchConfiguration("start_tmini")
    start_mid360 = LaunchConfiguration("start_mid360")
    start_fast_lio = LaunchConfiguration("start_fast_lio")

    sensor_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("robot_bringup"), "launch",
            "mkmini_calibrated_localization.launch.py",
        ])),
        condition=IfCondition(start_stack),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_livox": start_mid360,
            "start_fast_lio": start_fast_lio,
            "startup_freshness_boundary_enabled": "true",
            "startup_lidar_max_age_sec": "0.5",
            "startup_imu_max_age_sec": "0.05",
            "startup_stable_window_sec": "2.0",
        }.items(),
    )

    tmini = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        condition=IfCondition(start_tmini),
        parameters=[PathJoinSubstitution([
            FindPackageShare("robot_bringup"), "config", "tmini_mk2c.yaml"])]
    )
    tmini_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mk2c_base_link_to_laser_frame",
        output="screen",
        condition=IfCondition(start_tmini),
        arguments=[
            "--x", "0.703", "--y", "0.0", "--z", "0.1923",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "laser_frame",
        ],
    )
    # In T-mini-only stationary isolation there is no FAST-LIO odom->body
    # edge. Bridge the costmap's fixed frame to the stationary chassis; this
    # is disabled for the dual-sensor path where FAST-LIO supplies the edge.
    tmini_stationary_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mk2c_tmini_odom_chassis_to_base_footprint",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", start_tmini, "' == 'true' and '", start_fast_lio, "' != 'true'"
        ])),
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "odom_chassis", "--child-frame-id", "base_footprint",
        ],
    )

    def local_costmap_actions(context):
        if start_stack.perform(context).lower() not in ("true", "1", "yes"):
            return []
        # nav2_costmap_2d's standalone executable creates its own lifecycle
        # child (``costmap``). Its child does not inherit a node-keyed block
        # addressed to the launch host. Extract—not copy—the current production
        # local block and place it under ROS's wildcard key so that exact block
        # reaches that child. No costmap algorithm or parameter value is
        # reimplemented here.
        production = Path(get_package_share_directory("robot_bringup")) / "config" / "nav2_common.yaml"
        source = yaml.safe_load(production.read_text(encoding="utf-8"))
        params = source["local_costmap"]["local_costmap"]["ros__parameters"]
        generated = Path(tempfile.gettempdir()) / "p5a_mk2c_local_costmap_effective.yaml"
        generated.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}, sort_keys=False), encoding="utf-8")
        local_costmap = Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="mk2c_costmap_host",
            output="screen",
            parameters=[str(generated)],
        )
        lifecycle = Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_mk2c_local_costmap",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                # Humble's standalone costmap child does not establish the
                # lifecycle bond expected by the manager. Lifecycle services
                # still configure/activate it; disabling bond monitoring avoids
                # a false task-local bringup failure after successful activation.
                "bond_timeout": 0.0,
                # nav2_costmap_2d Humble standalone hard-codes this child
                # name/namespace; it is the real VoxelLayer node.
                "node_names": ["costmap/costmap"],
            }],
        )
        return [local_costmap, lifecycle]

    return LaunchDescription([
        # Safe by default: merely invoking the launch file cannot contact a
        # sensor or start a ROS node.  Hardware use remains an explicit action
        # after the operator declares it ready.
        DeclareLaunchArgument("start_stack", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_tmini", default_value="false"),
        DeclareLaunchArgument("start_mid360", default_value="true"),
        DeclareLaunchArgument("start_fast_lio", default_value="true"),
        sensor_pipeline,
        tmini,
        tmini_tf,
        tmini_stationary_tf,
        OpaqueFunction(function=local_costmap_actions),
    ])
