"""R3H production localization/Nav2 composition with one physical endpoint.

No mock adapter is present.  The physical backend starts disabled and is the
only subscriber below the Generic Gate output.
"""

import hashlib
import json
import os
from pathlib import Path
import shutil
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


R3H_DB = ("/home/dog/fastlio_rtabmap_ros2_ws/map/current_room_sessions/"
          "20260915_155828_current_room_final_production/rtabmap_2d.db")
R3H_DB_SHA256 = "2f517e07e2cf788f0695ea421a816fc030467075a79c75457ff2840529f2f1a8"
R3H_NAVIGATION_CPUS = {0, 1, 2, 3}
R3H_SAFETY_CPU = {7}


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def isolate_r3h_navigation_processes(context):
    """Keep full-stack compute away from the physical feedback/sender CPUs.

    ROS launch children inherit the launch process affinity. The physical
    backend and native sender explicitly widen back to their dedicated CPUs
    (4/5/7 and 6 respectively); localization, RTAB-Map, and Nav2 remain on
    0-3 so CPU-heavy map processing cannot starve SocketCAN feedback workers.
    """
    del context
    required_cpus = R3H_NAVIGATION_CPUS | {4, 5, 6, 7}
    available_cpus = os.sched_getaffinity(0)
    if not required_cpus.issubset(available_cpus):
        raise RuntimeError(
            "R3H physical composition requires CPUs 0-7 for isolated native "
            f"backend scheduling; available CPUs are {sorted(available_cpus)}"
        )
    os.sched_setaffinity(0, R3H_NAVIGATION_CPUS)
    return []


def generate_launch_description():
    robot = FindPackageShare("robot_bringup")
    safety = FindPackageShare("vehicle_cmd_safety")
    mkmini = FindPackageShare("mkmini_cmd_adapter")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    database_path = LaunchConfiguration("database_path")
    runtime_dir = LaunchConfiguration("rtabmap_runtime_dir")
    nav = PathJoinSubstitution([robot, "config", "nav2_common.yaml"])
    collision = PathJoinSubstitution([robot, "config", "collision_monitor_dual_sensor.yaml"])
    gate = PathJoinSubstitution([safety, "config", "r3h_physical_gate.yaml"])
    localization_validity = PathJoinSubstitution([safety, "config", "phase5_localization_validity.yaml"])
    tmini_validity = PathJoinSubstitution([safety, "config", "phase5_dual_tmini_validity.yaml"])
    physical = PathJoinSubstitution([mkmini, "config", "r11_physical_backend_commissioning.yaml"])

    # This process owns an OS file lock for the lifetime of the composition.
    # Its delayed dependent actions ensure a conflicting second launch exits
    # before it can create duplicate sensor, Nav2, Gate, or CAN authorities.
    instance_guard = Node(
        package="parking_robot_bringup", executable="r3h_instance_guard",
        name="r3h_instance_guard", output="screen",
    )

    def prepare_rtabmap_working_copy(context):
        """Hash-gate the R3H authority and give RTAB a disposable copy."""
        reference = Path(database_path.perform(context)).expanduser().resolve()
        if not reference.is_file():
            raise RuntimeError(f"R3H reference database is not a file: {reference}")
        reference_hash = _sha256(reference)
        if reference_hash != R3H_DB_SHA256:
            raise RuntimeError(
                "R3H reference database hash mismatch; refusing localization: "
                f"expected {R3H_DB_SHA256}, got {reference_hash}"
            )
        destination_dir = Path(runtime_dir.perform(context)).expanduser()
        destination_dir.mkdir(parents=True, exist_ok=True)
        run_id = time.strftime("%Y%m%dT%H%M%S") + f"_pid{os.getpid()}"
        runtime_db = destination_dir / f"rtabmap_2d_localization_{run_id}.db"
        shutil.copy2(reference, runtime_db)
        runtime_hash = _sha256(runtime_db)
        if runtime_hash != reference_hash:
            raise RuntimeError("R3H RTAB working-copy hash mismatch")
        runtime_db.with_suffix(".db.provenance.json").write_text(json.dumps({
            "reference_db": str(reference),
            "reference_sha256": reference_hash,
            "runtime_db": str(runtime_db),
            "runtime_sha256_before_rtabmap": runtime_hash,
            "policy": "RTAB may modify only this retained runtime copy",
        }, indent=2) + "\n", encoding="utf-8")
        context.launch_configurations["database_path"] = str(runtime_db)
        print(f"[r3h] RTAB localization working copy: {runtime_db}")
        return []

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([robot, "launch", "mkmini_calibrated_localization.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time, "start_livox": start_livox,
                          "start_fast_lio": "true"}.items())
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([robot, "launch", "rtabmap_bridge.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time, "localization": "true",
                          "database_path": database_path, "frame_id": "base_footprint",
                          "map_frame_id": "map", "odom_topic": "/Odometry",
                          "odom_frame_id": "odom_chassis", "rtabmap_imu_topic": "/unused_imu",
                          "scan_cloud_topic": "/cloud_registered_body",
                          "enable_rtabmap_self_filter": "true", "delete_db_on_start": "false"}.items())
    tmini = Node(package="ydlidar_ros2_driver", executable="ydlidar_ros2_driver_node",
                 name="ydlidar_ros2_driver_node", output="screen",
                 parameters=[PathJoinSubstitution([robot, "config", "tmini_mk2c.yaml"])],
                 remappings=[("/scan", "/scan_raw")])
    tmini_scan_normalizer = Node(
        package="robot_bringup", executable="tmini_scan_normalizer",
        name="tmini_scan_normalizer", output="screen",
        parameters=[{"input_topic": "/scan_raw", "output_topic": "/scan"}])
    tmini_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    name="r3h_base_link_to_laser_frame", output="screen",
                    arguments=["--x", "0.703", "--y", "0.0", "--z", "0.1923",
                               "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                               "--frame-id", "base_link", "--child-frame-id", "laser_frame"])
    nav_nodes = [
        # R22: consume the already self-cropped MID branch, classify points in
        # base_footprint, and preserve the source ``body`` frame so costmap
        # ray tracing starts at the real MID/FAST-LIO sensor origin.
        Node(package="robot_bringup", executable="mid360_nav_obstacle_filter",
             name="mid360_nav_obstacle_filter", output="screen",
             parameters=[PathJoinSubstitution([
                 robot, "config", "mkmini_mid360_nav_filter.yaml"])]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen", parameters=[nav]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", output="screen", parameters=[nav], remappings=[("/cmd_vel", "/cmd_vel_nav")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen", parameters=[nav]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation", output="screen", parameters=[{"autostart": True, "bond_timeout": 0.0, "node_names": ["planner_server", "controller_server", "bt_navigator"]}]),
    ]
    safety_nodes = [
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output="screen", parameters=[collision], prefix="taskset -c 7"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_collision_monitor", output="screen", parameters=[{"autostart": True, "bond_timeout": 0.0, "node_names": ["collision_monitor"]}]),
        Node(package="vehicle_cmd_safety", executable="localization_validity_monitor", name="localization_validity_monitor", output="screen", parameters=[localization_validity], prefix="taskset -c 7"),
        Node(package="vehicle_cmd_safety", executable="collision_monitor_validity_monitor", name="collision_monitor_validity_monitor", output="screen", parameters=[tmini_validity, {"validity_output_topic": "/system/collision_monitor_valid"}], prefix="taskset -c 7"),
        Node(package="vehicle_cmd_safety", executable="nav2_controller_validity_monitor", name="nav2_controller_validity_monitor", output="screen", parameters=[{
            "controller_node_name": "controller_server",
            "validity_output_topic": "/system/controller_valid",
            "publish_rate_hz": 20.0,
            "query_rate_hz": 5.0,
            "response_timeout_sec": 0.50,
        }], prefix="taskset -c 7"),
        Node(package="vehicle_cmd_safety", executable="guarded_vehicle_cmd_gate", name="guarded_vehicle_cmd_gate", output="screen", parameters=[gate], prefix="taskset -c 7"),
        Node(package="mkmini_cmd_adapter", executable="mkmini_physical_ros_backend", name="mkmini_physical_ros_backend", output="screen", parameters=[physical],
             # Native TX and feedback workers own CPU 4, executor owns CPU 5,
             # and supervision owns CPU 7. This keeps desktop workload off
             # the timing-critical writer without moving user processes.
             prefix="taskset -c 5,7"),
    ]
    guarded_composition = TimerAction(period=0.75, actions=[
        OpaqueFunction(function=prepare_rtabmap_working_copy),
        localization, rtabmap, tmini, tmini_scan_normalizer, tmini_tf,
        *nav_nodes, *safety_nodes,
    ])
    guard_exit_shutdown = RegisterEventHandler(OnProcessExit(
        target_action=instance_guard,
        on_exit=[
            LogInfo(msg="R3H singleton guard exited; stopping the physical composition"),
            EmitEvent(event=Shutdown(reason="R3H singleton guard lost")),
        ],
    ))
    return LaunchDescription([
        OpaqueFunction(function=isolate_r3h_navigation_processes),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_livox", default_value="true"),
        DeclareLaunchArgument("database_path", default_value=R3H_DB),
        DeclareLaunchArgument(
            "rtabmap_runtime_dir", default_value="/home/dog/phase5_runtime/r3h_rtabmap"),
        instance_guard, guard_exit_shutdown, guarded_composition,
    ])
