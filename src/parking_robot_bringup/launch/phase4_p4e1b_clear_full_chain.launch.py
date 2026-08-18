"""Allowlisted software-only P4-E.1B CLEAR full command chain."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare("parking_robot_bringup")
    safety = FindPackageShare("vehicle_cmd_safety")
    adapter = FindPackageShare("wheelchair_cmd_adapter")
    nav = PathJoinSubstitution([bringup, "config", "phase2_nav2_params.yaml"])
    map_yaml = PathJoinSubstitution([bringup, "maps", "phase2_clean_map.yaml"])
    collision = PathJoinSubstitution([bringup, "config", "phase4_p4e_collision_monitor_scan.yaml"])
    validity = PathJoinSubstitution([safety, "config", "phase4_p4c_collision_validity_scan.yaml"])
    gate = PathJoinSubstitution([safety, "config", "phase4_p4c_gate_mock.yaml"])
    fake = PathJoinSubstitution([bringup, "config", "phase4_p4e1a_fake_base.yaml"])
    adapter_params = PathJoinSubstitution([adapter, "config", "mock_wheelchair_cmd_adapter.yaml"])

    nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[nav, {"yaml_filename": map_yaml, "use_sim_time": False}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="phase2_map_to_odom_static_tf", output="screen",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[nav, {"use_sim_time": False}]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             output="screen", parameters=[nav, {"use_sim_time": False}],
             remappings=[("/cmd_vel", "/cmd_vel_nav_raw")]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=[nav, {"use_sim_time": False}],
             remappings=[("/cmd_vel", "/cmd_vel_nav_raw")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen", parameters=[nav, {"use_sim_time": False}]),
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", output="screen", parameters=[collision, {"use_sim_time": False}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": False, "autostart": True,
                          "node_names": ["map_server", "planner_server", "controller_server",
                                         "behavior_server", "bt_navigator"]}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_collision_monitor", output="screen",
             parameters=[{"use_sim_time": False, "autostart": True,
                          "node_names": ["collision_monitor"]}]),
        Node(package="parking_robot_bringup", executable="phase4_p4b_synthetic_obstacles",
             name="phase4_p4b_synthetic_obstacles", output="screen",
             parameters=[{"mode": "CLEAR", "frame_id": "base_footprint",
                          "publish_rate_hz": 20.0, "publish_scan": True,
                          "publish_pointcloud": False}]),
        Node(package="vehicle_cmd_safety", executable="collision_monitor_validity_monitor",
             name="collision_monitor_validity_monitor", output="screen", parameters=[validity]),
        Node(package="vehicle_cmd_safety", executable="phase4_p4c_permission_fixture",
             name="phase4_p4e1b_permission_fixture", output="screen",
             parameters=[{"publish_collision": False, "publish_localization": True,
                          "publish_controller": True, "localization_valid": True,
                          "controller_valid": True, "publish_rate_hz": 20.0}]),
        Node(package="vehicle_cmd_safety", executable="guarded_vehicle_cmd_gate",
             name="guarded_vehicle_cmd_gate", output="screen", parameters=[gate]),
        Node(package="wheelchair_cmd_adapter", executable="mock_wheelchair_cmd_adapter",
             name="mock_wheelchair_cmd_adapter", output="screen", parameters=[adapter_params]),
        Node(package="parking_robot_bringup", executable="phase4_vehicle_cmd_fake_base",
             name="phase4_vehicle_cmd_fake_base", output="screen", parameters=[fake]),
    ]
    return LaunchDescription(nodes)
