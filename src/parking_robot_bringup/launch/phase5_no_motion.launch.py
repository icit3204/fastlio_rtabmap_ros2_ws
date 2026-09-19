"""Authoritative Phase-5 real-localization, mock-output, no-CAN composition.

This launch has no automatic mission, no arm request, no fake dynamic TF and no
physical chassis publisher. The Generic Gate starts in its accepted DISARMED
state, and the only chassis adapter publishes the explicitly mock topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare("parking_robot_bringup")
    safety = FindPackageShare("vehicle_cmd_safety")
    adapter = FindPackageShare("wheelchair_cmd_adapter")
    mkmini = FindPackageShare("mkmini_cmd_adapter")
    robot = FindPackageShare("robot_bringup")

    nav = PathJoinSubstitution([bringup, "config", "phase5_no_motion_nav2.yaml"])
    nav_bt = PathJoinSubstitution([bringup, "behavior_trees", "phase4_p4e_chassis_navigate_to_pose.xml"])
    nav_through_bt = PathJoinSubstitution([bringup, "behavior_trees", "phase4_p4e_chassis_navigate_through_poses.xml"])
    collision = PathJoinSubstitution([bringup, "config", "collision_monitor_phase5_raw_mid360.yaml"])
    collision_validity = PathJoinSubstitution([bringup, "config", "phase5_collision_monitor_validity.yaml"])
    localization_validity = PathJoinSubstitution([safety, "config", "phase5_localization_validity.yaml"])
    gate = PathJoinSubstitution([bringup, "config", "phase5_gate_mock.yaml"])
    adapter_params = PathJoinSubstitution([adapter, "config", "mock_wheelchair_cmd_adapter.yaml"])
    physical_params = PathJoinSubstitution([mkmini, "config", "r11_physical_backend_commissioning.yaml"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    database_path = LaunchConfiguration("database_path")
    backend_mode = LaunchConfiguration("backend_mode")

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([robot, "launch", "phase5_calibrated_localization.launch.py"])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "start_livox": start_livox,
                "database_path": database_path,
            }.items(),
        ),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[nav, {"use_sim_time": use_sim_time}]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             output="screen", parameters=[nav, {"use_sim_time": use_sim_time}],
             remappings=[("/cmd_vel", "/cmd_vel_nav_raw")]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=[nav, {"use_sim_time": use_sim_time}],
             remappings=[("/cmd_vel", "/cmd_vel_nav_raw")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen", parameters=[nav, {"use_sim_time": use_sim_time,
                                                  "default_nav_to_pose_bt_xml": nav_bt,
                                                  "default_nav_through_poses_bt_xml": nav_through_bt}]),
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", output="screen",
             parameters=[collision, {"use_sim_time": use_sim_time}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                          "node_names": ["planner_server", "controller_server",
                                         "behavior_server", "bt_navigator"]}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_collision_monitor", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                          "node_names": ["collision_monitor"]}]),
        Node(package="vehicle_cmd_safety", executable="localization_validity_monitor",
             name="localization_validity_monitor", output="screen",
             parameters=[localization_validity, {"use_sim_time": use_sim_time}]),
        Node(package="vehicle_cmd_safety", executable="collision_monitor_validity_monitor",
             name="collision_monitor_validity_monitor", output="screen",
             parameters=[collision_validity, {"use_sim_time": use_sim_time}]),
        Node(package="vehicle_cmd_safety", executable="guarded_vehicle_cmd_gate",
             name="guarded_vehicle_cmd_gate", output="screen", parameters=[gate]),
        Node(package="wheelchair_cmd_adapter", executable="mock_wheelchair_cmd_adapter",
             name="mock_wheelchair_cmd_adapter", output="screen", parameters=[adapter_params],
             condition=IfCondition(PythonExpression(["'", backend_mode, "' == 'mock'"]))),
        Node(package="mkmini_cmd_adapter", executable="mkmini_physical_ros_backend",
             name="mkmini_physical_ros_backend", output="screen", parameters=[physical_params],
             condition=IfCondition(PythonExpression(["'", backend_mode, "' == 'physical_mkmini'"]))),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("backend_mode", default_value="mock",
                              description="Exactly one endpoint: mock or physical_mkmini."),
        DeclareLaunchArgument(
            "start_livox",
            default_value="true",
            description="Future live admission only; P5-0C1 does not execute this launch.",
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value=EnvironmentVariable("PARKING_ROBOT_RTABMAP_DATABASE", default_value=""),
            description="Explicit accepted RTAB-Map localization database path.",
        ),
        *nodes,
    ])
