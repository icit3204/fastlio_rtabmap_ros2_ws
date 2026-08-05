from pathlib import Path
import ast
import xml.etree.ElementTree as ET

import yaml


PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[1]
GATE = PKG / "vehicle_cmd_safety" / "guarded_vehicle_cmd_gate.py"
VALIDITY = PKG / "vehicle_cmd_safety" / "collision_monitor_validity_monitor.py"
PERMISSION_FIXTURE = PKG / "vehicle_cmd_safety" / "phase4_p4c_permission_fixture.py"
SETUP = PKG / "setup.py"
PACKAGE_XML = PKG / "package.xml"
GATE_YAML = PKG / "config" / "phase4_p4c_gate_mock.yaml"
SCAN_YAML = PKG / "config" / "phase4_p4c_collision_validity_scan.yaml"
POINTS_YAML = PKG / "config" / "phase4_p4c_collision_validity_points.yaml"
GATE_LAUNCH = PKG / "launch" / "phase4_p4c_gate_preflight.launch.py"
VALIDITY_LAUNCH = PKG / "launch" / "phase4_p4c_collision_validity_preflight.launch.py"
E2E_LAUNCH = PKG / "launch" / "phase4_p4c_e2e_stale_observation.launch.py"
DOC_DESIGN = ROOT / "docs" / "phases" / "phase_04" / "PHASE_04_P4C_GENERIC_COMMAND_SAFETY_GATE_DESIGN.md"


def text(path: Path) -> str:
    return path.read_text()


def test_package_metadata_and_entry_points():
    ET.parse(PACKAGE_XML)
    package_text = text(PACKAGE_XML)
    for dep in ["geometry_msgs", "std_msgs", "std_srvs", "diagnostic_msgs", "lifecycle_msgs", "rcl_interfaces", "sensor_msgs"]:
        assert f"<exec_depend>{dep}</exec_depend>" in package_text
    setup_text = text(SETUP)
    assert setup_text.count("guarded_vehicle_cmd_gate = vehicle_cmd_safety.guarded_vehicle_cmd_gate:main") == 1
    assert setup_text.count("collision_monitor_validity_monitor = vehicle_cmd_safety.collision_monitor_validity_monitor:main") == 1
    assert setup_text.count("phase4_p4c_runtime_runner = vehicle_cmd_safety.phase4_p4c_runtime_runner:main") == 1


def test_gate_topic_contract_and_no_raw_or_wheelchair_subscription():
    gate_text = text(GATE)
    assert 'create_subscription(Twist, "/cmd_vel_nav_safe"' in gate_text
    assert 'create_publisher(TwistStamped, "/vehicle_cmd_safe"' in gate_text
    assert 'create_service(SetBool, "/vehicle_cmd_safety/arm"' in gate_text
    assert 'create_publisher(DiagnosticStatus, "/vehicle_cmd_safety/state"' in gate_text
    assert '"/cmd_vel_nav_raw"' not in gate_text
    for forbidden in ["/wheelchair_control_command_mock", "/wheelchair_control_command", "/wheelchair_control_command_raw"]:
        assert forbidden not in gate_text


def test_gate_safety_timers_use_retained_steady_clocks():
    gate_tree = ast.parse(text(GATE))
    gate_text = text(GATE)
    assert "from rclpy.clock import Clock, ClockType" in gate_text
    assert "Clock(clock_type=ClockType.STEADY_TIME)" in gate_text
    assert "self._heartbeat_clock" in gate_text
    assert "self._graph_clock" in gate_text

    timer_calls = [
        node
        for node in ast.walk(gate_tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_timer"
    ]
    callbacks = {}
    for call in timer_calls:
        if len(call.args) < 2:
            continue
        callback = call.args[1]
        if not isinstance(callback, ast.Attribute):
            continue
        clock_keywords = [kw for kw in call.keywords if kw.arg == "clock"]
        assert len(clock_keywords) == 1
        clock_value = clock_keywords[0].value
        assert isinstance(clock_value, ast.Attribute)
        callbacks[callback.attr] = clock_value.attr

    assert callbacks["_heartbeat_cb"] == "_heartbeat_clock"
    assert callbacks["_graph_cb"] == "_graph_clock"


def test_gate_ros_time_is_limited_to_output_and_diagnostic_stamps():
    gate_tree = ast.parse(text(GATE))
    parents = {}
    for parent in ast.walk(gate_tree):
        for child in ast.iter_child_nodes(parent):
            parents[child] = parent
    ros_now_functions = []
    for node in ast.walk(gate_tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "now"
            and isinstance(node.func.value, ast.Call)
            and isinstance(node.func.value.func, ast.Attribute)
            and node.func.value.func.attr == "get_clock"
        ):
            continue
        current = node
        while current in parents and not isinstance(current, ast.FunctionDef):
            current = parents[current]
        assert isinstance(current, ast.FunctionDef)
        ros_now_functions.append(current.name)
    assert sorted(ros_now_functions) == ["_publish_output", "_publish_status"]
    assert "time.time(" not in text(GATE)
    assert "time.monotonic()" in text(GATE)


def test_validity_monitor_topic_contract_and_no_safe_twist_inference():
    validity_text = text(VALIDITY)
    assert 'create_publisher(Bool, "/system/collision_monitor_valid"' in validity_text
    assert "/phase4/synthetic_scan" in validity_text
    assert "/phase4/synthetic_points" not in validity_text
    assert "/cmd_vel_nav_safe" not in validity_text
    assert "/vehicle_cmd_safe" not in validity_text


def test_yaml_limits_and_defaults_are_phase4_only():
    params = yaml.safe_load(text(GATE_YAML))["guarded_vehicle_cmd_gate"]["ros__parameters"]
    assert params["max_forward_velocity"] == 0.20
    assert params["max_angular_velocity"] == 0.50
    assert params["max_linear_increase_rate"] == 0.50
    assert params["max_angular_increase_rate"] == 1.00
    assert params["max_forward_velocity"] <= 0.25
    assert params["max_angular_velocity"] <= 0.8
    assert params["frame_id"] == "base_footprint"


def test_validity_profiles_match_p4b_synthetic_sources():
    scan = yaml.safe_load(text(SCAN_YAML))["collision_monitor_validity_monitor"]["ros__parameters"]
    points = yaml.safe_load(text(POINTS_YAML))["collision_monitor_validity_monitor"]["ros__parameters"]
    assert scan["source_type"] == "scan"
    assert scan["source_topic"] == "/phase4/synthetic_scan"
    assert scan["expected_observation_source_name"] == "scan"
    assert scan["expected_observation_source_type"] == "scan"
    assert points["source_type"] == "pointcloud"
    assert points["source_topic"] == "/phase4/synthetic_points"
    assert points["expected_observation_source_name"] == "pointcloud"
    assert points["expected_observation_source_type"] == "pointcloud"
    assert scan["source_freshness_sec"] == 0.50
    assert points["recovery_stability_sec"] == 0.50


def test_launch_files_are_parseable_and_omit_forbidden_nodes():
    ast.parse(text(GATE_LAUNCH))
    ast.parse(text(VALIDITY_LAUNCH))
    ast.parse(text(E2E_LAUNCH))
    combined = text(GATE_LAUNCH) + "\n" + text(VALIDITY_LAUNCH) + "\n" + text(E2E_LAUNCH)
    forbidden = [
        "wheelchair_controller_node",
        "wheelchair_cmd_adapter",
        "vehicle_cmd_safe_adapter",
        "laser_command_safety_filter",
        "pure_pursuit",
        "velocity_smoother",
        "UdpSender",
        "SocketCAN",
        "can0",
        "vcan",
        "ydlidar",
        "MID-360",
        "camera",
        "Gazebo",
    ]
    for item in forbidden:
        assert item not in combined


def test_permission_fixture_collision_publisher_is_conditional():
    from vehicle_cmd_safety.phase4_p4c_permission_fixture import enabled_permission_topics

    assert enabled_permission_topics(publish_collision=False) == (
        "/system/localization_valid",
        "/system/controller_valid",
    )
    assert enabled_permission_topics(publish_collision=True).count("/system/collision_monitor_valid") == 1
    assert "/system/localization_valid" in enabled_permission_topics(
        publish_localization=True,
        publish_controller=False,
        publish_collision=False,
    )
    assert "/system/controller_valid" in enabled_permission_topics(
        publish_localization=False,
        publish_controller=True,
        publish_collision=False,
    )


def test_permission_fixture_does_not_create_silent_collision_publisher_when_disabled():
    fixture_text = text(PERMISSION_FIXTURE)
    assert 'if self._publish_collision' in fixture_text
    assert 'self.create_publisher(Bool, "/system/collision_monitor_valid", 10)' in fixture_text
    assert 'else None' in fixture_text
    assert 'if self._collision_pub is not None' in fixture_text


def test_e2e_launch_disables_collision_permission_fixture_authority():
    e2e_text = text(E2E_LAUNCH)
    assert 'executable="phase4_p4c_permission_fixture"' in e2e_text
    assert '"publish_collision": False' in e2e_text
    assert e2e_text.count('executable="collision_monitor_validity_monitor"') == 1
    assert e2e_text.count('executable="phase4_p4c_permission_fixture"') == 1


def test_source_authority_forbidden_terms_absent_from_p4c_package():
    evidence_audit_sources = {
        "vehicle_cmd_safety/phase4_p4c_runtime_runner.py",
        "vehicle_cmd_safety/phase4_p4c_evidence_monitor.py",
    }
    allowed = {
        "package.xml",
        "setup.py",
        "setup.cfg",
        "resource/vehicle_cmd_safety",
        "config/phase4_p4c_gate_mock.yaml",
        "config/phase4_p4c_collision_validity_scan.yaml",
        "config/phase4_p4c_collision_validity_points.yaml",
    }
    collected = []
    for path in PKG.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(PKG).as_posix()
        if rel.startswith("test/"):
            continue
        if rel in evidence_audit_sources:
            continue
        if path.suffix in {".py", ".yaml", ".yml", ".xml", ".md"} or rel in allowed:
            collected.append(path.read_text())
    package_text = "\n".join(collected)
    assert "/cmd_vel_nav_raw" not in text(GATE)
    for topic in ["/wheelchair_control_command_mock", "/wheelchair_control_command", "/wheelchair_control_command_raw"]:
        assert topic not in package_text
    for term in ["SocketCAN", "UdpSender"]:
        assert term not in package_text


def test_design_document_exists_before_runtime():
    design = text(DOC_DESIGN)
    assert "pure deterministic core" in design
    assert "monotonic" in design
    assert "fault" in design.lower()
