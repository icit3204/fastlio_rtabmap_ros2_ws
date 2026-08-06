from pathlib import Path
import ast
import xml.etree.ElementTree as ET

import yaml


PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
NODE = PKG / "wheelchair_cmd_adapter" / "mock_wheelchair_cmd_adapter.py"
CORE = PKG / "wheelchair_cmd_adapter" / "conversion_core.py"
SETUP = PKG / "setup.py"
PACKAGE_XML = PKG / "package.xml"
CONFIG = PKG / "config" / "mock_wheelchair_cmd_adapter.yaml"
GATE = SRC / "vehicle_cmd_safety" / "vehicle_cmd_safety" / "guarded_vehicle_cmd_gate.py"
P4D3_RUNNER = PKG / "test" / "phase4_p4d3_integration_runner.py"


def text(path: Path) -> str:
    return path.read_text()


def test_package_metadata_and_boundary():
    ET.parse(PACKAGE_XML)
    package_text = text(PACKAGE_XML)
    assert "<name>wheelchair_cmd_adapter</name>" in package_text
    assert "vehicle_cmd_safety" not in package_text
    setup_text = text(SETUP)
    assert "mock_wheelchair_cmd_adapter = wheelchair_cmd_adapter.mock_wheelchair_cmd_adapter:main" in setup_text
    assert "guarded_vehicle_cmd_gate" not in setup_text


def test_yaml_contains_exact_reviewed_parameters_only():
    params = yaml.safe_load(text(CONFIG))["mock_wheelchair_cmd_adapter"]["ros__parameters"]
    assert params == {
        "input_topic": "/vehicle_cmd_safe",
        "output_topic": "/wheelchair_control_command_mock",
        "expected_frame": "base_footprint",
        "heartbeat_hz": 20.0,
        "input_timeout_sec": 0.25,
        "max_forward_velocity": 0.20,
        "max_angular_velocity": 0.50,
        "unsupported_axis_epsilon": "1e-6",
        "reverse_epsilon": "1e-6",
        "in_place_linear_epsilon": 0.01,
        "in_place_angular_epsilon": 0.02,
        "minimum_turn_radius_m": 1.0,
        "straight_radius_m": 10.0,
    }


def test_node_topic_contract():
    node_text = text(NODE)
    assert 'self.create_subscription(TwistStamped, self._config.input_topic' in node_text
    assert 'self.create_publisher(Float32MultiArray, self._config.output_topic' in node_text
    assert '"/vehicle_cmd_safe"' in text(CORE)
    assert '"/wheelchair_control_command_mock"' in text(CORE)
    assert '"/wheelchair_control_command"' not in node_text
    assert '"/wheelchair_control_command_raw"' not in node_text
    assert '"/cmd_vel_nav_raw"' not in node_text
    assert '"/cmd_vel_nav_safe"' not in node_text
    assert '"/cmd_vel"' not in node_text
    assert "validate_topic_contract(config.input_topic, config.output_topic)" in node_text


def test_forbidden_transport_terms_absent_from_production_source():
    production = text(NODE) + "\n" + text(CORE) + "\n" + text(CONFIG)
    for term in ["SocketCAN", "can0", "vcan", "UDP", "udp", "target_ip", "target_port", "can_frame_id"]:
        assert term not in production


def test_explicit_retained_steady_time_timers():
    node_text = text(NODE)
    assert "from rclpy.clock import Clock, ClockType" in node_text
    assert "Clock(clock_type=ClockType.STEADY_TIME)" in node_text
    assert "self._heartbeat_clock" in node_text
    assert "self._graph_clock" in node_text
    tree = ast.parse(node_text)
    callbacks = {}
    for call in ast.walk(tree):
        if not (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == "create_timer"
            and len(call.args) >= 2
            and isinstance(call.args[1], ast.Attribute)
        ):
            continue
        clock_keywords = [kw for kw in call.keywords if kw.arg == "clock"]
        assert len(clock_keywords) == 1
        assert isinstance(clock_keywords[0].value, ast.Attribute)
        callbacks[call.args[1].attr] = clock_keywords[0].value.attr
    assert callbacks["_heartbeat_cb"] == "_heartbeat_clock"
    assert callbacks["_graph_cb"] == "_graph_clock"


def test_core_is_pure_no_ros_graph_code_or_clocks():
    core_text = text(CORE)
    for term in [
        "rclpy",
        "create_publisher",
        "create_subscription",
        "get_publishers_info_by_topic",
        "Clock(",
        "time.monotonic",
        "get_clock",
    ]:
        assert term not in core_text


def test_guarded_vehicle_cmd_gate_unmodified_by_adapter_contract():
    gate_text = text(GATE)
    assert 'create_publisher(TwistStamped, "/vehicle_cmd_safe"' in gate_text
    assert "wheelchair_cmd_adapter" not in gate_text
    assert "/wheelchair_control_command_mock" not in gate_text


def test_p4d3_static_integration_contract_and_package_separation():
    gate_text = text(GATE)
    node_text = text(NODE)
    assert 'create_subscription(Twist, "/cmd_vel_nav_safe"' in gate_text
    for topic in ["/system/localization_valid", "/system/controller_valid", "/system/collision_monitor_valid"]:
        assert topic in gate_text
    assert 'create_publisher(TwistStamped, "/vehicle_cmd_safe"' in gate_text
    assert 'self.create_subscription(TwistStamped, self._config.input_topic' in node_text
    assert 'self.create_publisher(Float32MultiArray, self._config.output_topic' in node_text
    assert 'self.declare_parameter("frame_id", "base_footprint")' in gate_text
    assert 'self.declare_parameter("expected_frame", "base_footprint")' in node_text
    assert "/cmd_vel_nav_safe" not in node_text
    assert P4D3_RUNNER.is_file()
    assert "vehicle_cmd_safety" not in text(PACKAGE_XML)
