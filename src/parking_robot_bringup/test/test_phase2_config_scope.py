from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch" / "phase2_core_nav2.launch.py"
PARAMS = PKG / "config" / "phase2_nav2_params.yaml"
PACKAGE_XML = PKG / "package.xml"

EXCLUDED = [
    "semantic_grid_tools",
    "/ground_cloud",
    "/obstacle_cloud",
    "/semantic_grid",
    "Collision Monitor",
    "collision_monitor",
    "Generic Command Safety Gate",
    "/vehicle_cmd_safe",
    "Wheelchair Command Adapter",
    "/wheelchair_control_command",
    "wheelchair_controller_node",
    "can0",
    "SocketCAN",
    "plan_nav",
    "Pure Pursuit",
    "pure_pursuit",
    "plan_nav_laser_avoidance",
    "laser_command_safety_filter",
    "Livox",
    "MID-360",
    "FAST-LIO",
    "fast_lio",
    "RTAB-Map live",
    "rtabmap",
    "camera",
    "YDLIDAR",
    "ydlidar",
    "ultrasonic",
    "keepout",
    "speed_filter",
    "Gazebo",
    "Mission Manager",
]


def test_package_xml_parses():
    ET.parse(PACKAGE_XML)


def test_no_excluded_scope_terms_in_launch_or_params():
    text = LAUNCH.read_text() + "\n" + PARAMS.read_text()
    for term in EXCLUDED:
        assert term not in text


def test_only_mock_velocity_topic_is_used():
    text = LAUNCH.read_text() + "\n" + PARAMS.read_text()
    assert "/cmd_vel_phase2_mock" in text
    for forbidden in ["/wheelchair_control_command", "/vehicle_cmd_safe", "/cmd_vel_nav"]:
        assert forbidden not in text


def test_costmaps_have_static_and_inflation_without_observation_sources():
    params = yaml.safe_load(PARAMS.read_text())
    for key in ("global_costmap", "local_costmap"):
        ros_params = params[key][key]["ros__parameters"]
        assert "static_layer" in ros_params["plugins"]
        assert "inflation_layer" in ros_params["plugins"]
        assert "obstacle_layer" not in ros_params["plugins"]
        assert "voxel_layer" not in ros_params["plugins"]
        assert "observation_sources" not in yaml.dump(ros_params)


def test_use_sim_time_defaults_false_and_package_share_defaults():
    params = yaml.safe_load(PARAMS.read_text())
    assert params["map_server"]["ros__parameters"]["use_sim_time"] is False
    assert params["planner_server"]["ros__parameters"]["use_sim_time"] is False
    assert params["controller_server"]["ros__parameters"]["use_sim_time"] is False
    launch_text = LAUNCH.read_text()
    assert 'DeclareLaunchArgument("use_sim_time", default_value="false")' in launch_text
    assert 'FindPackageShare("parking_robot_bringup")' in launch_text
    assert "PathJoinSubstitution" in launch_text


def test_plugin_identifiers_are_expected_phase2_set():
    params = yaml.safe_load(PARAMS.read_text())
    dumped = yaml.dump(params)
    expected = [
        "nav2_navfn_planner/NavfnPlanner",
        "nav2_mppi_controller::MPPIController",
        "nav2_costmap_2d::StaticLayer",
        "nav2_costmap_2d::InflationLayer",
        "nav2_behaviors/Spin",
        "nav2_behaviors/BackUp",
        "nav2_behaviors/Wait",
    ]
    for plugin in expected:
        assert plugin in dumped
    forbidden_plugins = ["ObstacleLayer", "VoxelLayer", "KeepoutFilter", "SpeedFilter"]
    for plugin in forbidden_plugins:
        assert plugin not in dumped


def test_launch_node_packages_are_explicit_phase2_set():
    text = LAUNCH.read_text()
    packages = set(re.findall(r'package="([^"]+)"', text))
    assert packages == {
        "nav2_map_server",
        "tf2_ros",
        "parking_robot_bringup",
        "nav2_planner",
        "nav2_controller",
        "nav2_behaviors",
        "nav2_bt_navigator",
        "nav2_lifecycle_manager",
    }

