from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / "robot_bringup" / "launch" / "mk2e4_labmate_backend_mock.launch.py"
BRIDGE = ROOT / "wheelchair_cmd_adapter" / "wheelchair_cmd_adapter" / "gate_to_labmate_bridge.py"
CORE = ROOT / "wheelchair_controller" / "src" / "mkmini_backend_core.cpp"


def test_qualification_launch_is_unconditionally_mock_only():
    source = LAUNCH.read_text()
    assert '"output_transport": "mock"' in source
    assert '"auto_start": True' in source
    assert "DeclareLaunchArgument" not in source
    assert "can0" not in source
    assert "controller_server" not in source
    assert "bt_navigator" not in source


def test_one_normal_authority_below_gate():
    source = BRIDGE.read_text()
    assert '"/vehicle_cmd_safe"' in source
    assert '"/wheelchair_control_command"' in source
    for forbidden in ("cmd_vel", "keyboard", "pure_pursuit", "SocketCAN", "can0"):
        assert forbidden not in source


def test_codec_core_has_no_socketcan_io():
    source = CORE.read_text()
    for forbidden in ("socket(", "bind(", "write(", "can0"):
        assert forbidden not in source
