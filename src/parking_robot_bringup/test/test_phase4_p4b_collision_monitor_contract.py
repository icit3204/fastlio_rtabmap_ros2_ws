from pathlib import Path
import ast
import math
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from builtin_interfaces.msg import Time

from parking_robot_bringup.phase4_p4b_synthetic_obstacles import (
    CLEAR_POINTS,
    SLOW_POINTS,
    STOP_POINTS,
    points_for_mode,
    points_to_cloud,
    points_to_scan,
)


PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch" / "phase4_p4b_collision_monitor.launch.py"
PREFLIGHT_LAUNCH = PKG / "launch" / "phase4_p4b_collision_monitor_preflight.launch.py"
SCAN_YAML = PKG / "config" / "phase4_p4b_collision_monitor_scan.yaml"
POINTS_YAML = PKG / "config" / "phase4_p4b_collision_monitor_points.yaml"
FIXTURE = PKG / "parking_robot_bringup" / "phase4_p4b_synthetic_obstacles.py"
RAW_FIXTURE = PKG / "parking_robot_bringup" / "phase4_p4b_raw_twist_fixture.py"
PREFLIGHT_MONITOR = PKG / "parking_robot_bringup" / "phase4_p4b_preflight_monitor.py"
SCENARIO_RUNNER = PKG / "parking_robot_bringup" / "phase4_p4b_nav2_scenario_runner.py"
DOC = PKG.parents[1] / "docs" / "phases" / "phase_04" / "PHASE_04_P4B_COLLISION_MONITOR_CHAIN.md"
PACKAGE_XML = PKG / "package.xml"
SETUP = PKG / "setup.py"

FORBIDDEN_TOPICS = [
    "/vehicle_cmd_safe",
    "/system/collision_monitor_valid",
    "/wheelchair_control_command_mock",
    "/wheelchair_control_command",
    "/wheelchair_control_command_raw",
]

FORBIDDEN_IMPLEMENTATION_TERMS = [
    "SocketCAN",
    "UdpSender",
    "laser_command_safety_filter",
    "wheelchair_controller_node",
]


def _params(path: Path):
    return yaml.safe_load(path.read_text())["collision_monitor"]["ros__parameters"]


def _points(path: Path, key: str):
    values = _params(path)[key]["points"]
    return [(values[index], values[index + 1]) for index in range(0, len(values), 2)]


def _inside_rect(point, rect):
    xs = [p[0] for p in rect]
    ys = [p[1] for p in rect]
    return min(xs) <= point.x <= max(xs) and min(ys) <= point.y <= max(ys)


def test_package_xml_parses_and_declares_phase4_dependencies():
    ET.parse(PACKAGE_XML)
    text = PACKAGE_XML.read_text()
    assert "<exec_depend>sensor_msgs</exec_depend>" in text
    assert "<exec_depend>rcl_interfaces</exec_depend>" in text
    assert "<exec_depend>tf2_msgs</exec_depend>" in text


@pytest.mark.parametrize("path,source_name,source_type,topic", [
    (SCAN_YAML, "scan", "scan", "/phase4/synthetic_scan"),
    (POINTS_YAML, "pointcloud", "pointcloud", "/phase4/synthetic_points"),
])
def test_collision_monitor_yaml_uses_installed_humble_schema(path, source_name, source_type, topic):
    params = _params(path)
    assert params["base_frame_id"] == "base_footprint"
    assert params["odom_frame_id"] == "odom"
    assert params["cmd_vel_in_topic"] == "/cmd_vel_nav_raw"
    assert params["cmd_vel_out_topic"] == "/cmd_vel_nav_safe"
    assert params["transform_tolerance"] == 0.5
    assert params["source_timeout"] == 0.5
    assert params["stop_pub_timeout"] == 2.0
    assert params["observation_sources"] == [source_name]
    assert params[source_name]["type"] == source_type
    assert params[source_name]["topic"] == topic
    assert params[source_name]["enabled"] is True
    assert params["polygons"] == ["PolygonStop", "PolygonSlow"]
    assert params["PolygonStop"]["action_type"] == "stop"
    assert params["PolygonSlow"]["action_type"] == "slowdown"
    assert params["PolygonSlow"]["slowdown_ratio"] == 0.30


@pytest.mark.parametrize("path", [SCAN_YAML, POINTS_YAML])
def test_stop_polygon_is_inside_slowdown_polygon(path):
    stop = _points(path, "PolygonStop")
    slow = _points(path, "PolygonSlow")
    slow_x = [p[0] for p in slow]
    slow_y = [p[1] for p in slow]
    assert all(min(slow_x) <= x <= max(slow_x) for x, _ in stop)
    assert all(min(slow_y) <= y <= max(slow_y) for _, y in stop)
    assert not (min(x for x, _ in stop) <= 0.0 <= max(x for x, _ in stop))


def test_launch_routes_raw_to_collision_monitor_and_safe_to_fake_base():
    text = LAUNCH.read_text()
    assert 'remappings=[("/cmd_vel", "/cmd_vel_nav_raw")]' in text
    assert '"cmd_vel_topic": "/cmd_vel_nav_safe"' in text
    assert 'package="nav2_collision_monitor"' in text
    assert 'executable="collision_monitor"' in text
    assert '"node_names": ["collision_monitor"]' in text
    assert "/cmd_vel_phase2_mock" not in text


def test_launch_has_required_arguments_and_default_scan_source():
    text = LAUNCH.read_text()
    assert '"observation_source"' in text
    assert 'default_value="scan"' in text
    assert 'choices=["scan", "points"]' in text
    assert '"start_synthetic_fixture"' in text
    assert '"use_sim_time", default_value="false"' in text
    assert '"use_rviz", default_value="false"' in text
    assert '"source_timeout", default_value="0.5"' in text
    assert '"visualize", default_value="false"' in text


def test_launch_omits_velocity_smoother_live_sensors_and_legacy_controllers():
    text = LAUNCH.read_text()
    forbidden = [
        "nav2_velocity_smoother",
        "velocity_smoother",
        "Pure Pursuit",
        "pure_pursuit",
        "laser_command_safety_filter",
        "wheelchair_controller_node",
        "ydlidar",
        "YDLIDAR",
        "MID-360",
        "Livox",
        "camera",
        "Gazebo",
        "plan_nav",
    ]
    for term in forbidden:
        assert term not in text


def test_launch_uses_no_recovery_bt_to_avoid_behavior_server_twist_authority():
    text = LAUNCH.read_text()
    assert "navigate_w_replanning_time.xml" in text
    assert "behavior_server" not in text
    assert "nav2_behaviors" not in text


def test_synthetic_fixture_modes_and_geometry_are_deterministic():
    stop_rect = _points(SCAN_YAML, "PolygonStop")
    slow_rect = _points(SCAN_YAML, "PolygonSlow")
    assert points_for_mode("CLEAR") == CLEAR_POINTS
    assert points_for_mode("SLOW") == SLOW_POINTS
    assert points_for_mode("STOP") == STOP_POINTS
    assert points_for_mode("SILENT") == ()
    assert all(not _inside_rect(point, slow_rect) for point in CLEAR_POINTS)
    assert all(_inside_rect(point, slow_rect) for point in SLOW_POINTS)
    assert all(not _inside_rect(point, stop_rect) for point in SLOW_POINTS)
    assert all(_inside_rect(point, stop_rect) for point in STOP_POINTS)
    assert len(SLOW_POINTS) > _params(SCAN_YAML)["PolygonSlow"]["max_points"]
    assert len(STOP_POINTS) > _params(SCAN_YAML)["PolygonStop"]["max_points"]


def test_laserscan_fixture_message_is_valid():
    stamp = Time(sec=1, nanosec=2)
    scan = points_to_scan(STOP_POINTS, "base_footprint", stamp)
    assert scan.header.frame_id == "base_footprint"
    assert scan.header.stamp is stamp
    assert scan.angle_min < 0.0 < scan.angle_max
    assert scan.angle_increment > 0.0
    assert scan.range_min > 0.0
    assert scan.range_max > scan.range_min
    assert len(scan.ranges) == len(scan.intensities)
    assert sum(math.isfinite(value) and value < scan.range_max for value in scan.ranges) >= len(STOP_POINTS)


def test_pointcloud_fixture_message_is_valid():
    stamp = Time(sec=1, nanosec=2)
    cloud = points_to_cloud(STOP_POINTS, "base_footprint", stamp)
    assert cloud.header.frame_id == "base_footprint"
    assert cloud.header.stamp is stamp
    assert cloud.height == 1
    assert cloud.width == len(STOP_POINTS)
    assert cloud.point_step == 12
    assert cloud.row_step == cloud.point_step * cloud.width
    assert len(cloud.data) == cloud.row_step
    assert [field.name for field in cloud.fields] == ["x", "y", "z"]


def test_source_authority_forbidden_terms_are_absent_from_p4b_runtime_files():
    runtime_text = "\n".join(
        path.read_text()
        for path in [LAUNCH, PREFLIGHT_LAUNCH, SCAN_YAML, POINTS_YAML, FIXTURE, RAW_FIXTURE, PREFLIGHT_MONITOR]
    )
    for topic in FORBIDDEN_TOPICS:
        assert topic not in runtime_text
    for term in FORBIDDEN_IMPLEMENTATION_TERMS:
        assert term not in runtime_text
    assert "CAN" not in runtime_text


def test_setup_installs_synthetic_fixture_once():
    text = SETUP.read_text()
    monitor_entry = "phase4_p4b_preflight_monitor = parking_robot_bringup.phase4_p4b_preflight_monitor:main"
    scenario_entry = "phase4_p4b_nav2_scenario_runner = parking_robot_bringup.phase4_p4b_nav2_scenario_runner:main"
    raw_entry = "phase4_p4b_raw_twist_fixture = parking_robot_bringup.phase4_p4b_raw_twist_fixture:main"
    entry = "phase4_p4b_synthetic_obstacles = parking_robot_bringup.phase4_p4b_synthetic_obstacles:main"
    assert text.count(monitor_entry) == 1
    assert text.count(scenario_entry) == 1
    assert text.count(raw_entry) == 1
    assert text.count(entry) == 1


def test_document_records_p4b_boundary_and_no_p4c_start():
    text = DOC.read_text()
    assert "/cmd_vel_nav_raw" in text
    assert "/cmd_vel_nav_safe" in text
    assert "P4-C may begin after P4-B fresh-source filtering is committed and pushed" in text
    assert "It does not implement `/vehicle_cmd_safe`" in text
    assert "P4-B does not claim chain-level stale-source safety" in text


def test_launch_file_is_parseable_python():
    ast.parse(LAUNCH.read_text())
    ast.parse(PREFLIGHT_LAUNCH.read_text())


def test_no_unexpected_phase4_publishers_are_declared_in_launch():
    text = LAUNCH.read_text()
    node_names = set(re.findall(r'name="([^"]+)"', text))
    assert "collision_monitor" in node_names
    assert "phase2_fake_base" in node_names
    assert "phase4_p4b_synthetic_obstacles" in node_names
    assert "guarded_vehicle_cmd_gate" not in node_names
    assert "wheelchair_cmd_adapter_node" not in node_names


def test_preflight_launch_uses_fixture_raw_publisher_only():
    text = PREFLIGHT_LAUNCH.read_text()
    assert "phase4_p4b_raw_twist_fixture" in text
    assert "/cmd_vel_nav_raw" not in text
    assert 'package="nav2_collision_monitor"' in text
    assert '"node_names": ["collision_monitor"]' in text


def test_raw_fixture_publishes_only_raw_twist_topic():
    text = RAW_FIXTURE.read_text()
    assert "/cmd_vel_nav_raw" in text
    assert "/cmd_vel_nav_safe" not in text
    for topic in FORBIDDEN_TOPICS:
        assert topic not in text


def test_nav2_scenario_runner_records_required_evidence_files_without_p4c_interfaces():
    text = SCENARIO_RUNNER.read_text()
    for filename in [
        "raw_cmd_timeline.tsv",
        "safe_cmd_timeline.tsv",
        "odom_timeline.tsv",
        "tf_timeline.tsv",
        "obstacle_mode_events.jsonl",
        "action_events.jsonl",
        "publisher_provenance.tsv",
        "process_authority.tsv",
        "resource_timeline.tsv",
        "monitor_heartbeat.tsv",
        "terminal_metrics.json",
    ]:
        assert filename in text
    assert "NavigateToPose" in text
    assert "SetParameters" in text
    assert "create_publisher(PoseWithCovarianceStamped, \"/initialpose\"" in text
    assert "create_publisher(Twist" not in text
