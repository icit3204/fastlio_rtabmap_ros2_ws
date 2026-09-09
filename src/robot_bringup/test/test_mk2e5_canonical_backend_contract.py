from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "bringup.launch.py"


def test_canonical_stationary_chain_selects_only_new_lower_backend():
    source = LAUNCH.read_text()
    assert "executable='gate_to_labmate_bridge'" in source
    assert "executable='wheelchair_controller_node'" in source
    assert "mkmini_command_chain_dry_run_node" not in source
    assert "pure_pursuit_controller_node" not in source
    assert "keyboard_can_control" not in source


def test_gate_to_backend_topic_and_type_ownership_contract():
    source = LAUNCH.read_text()
    assert "{'output_topic': '/vehicle_cmd_safe'} if with_chain else {}" in source
    bridge = (
        ROOT.parent
        / "wheelchair_cmd_adapter"
        / "wheelchair_cmd_adapter"
        / "gate_to_labmate_bridge.py"
    ).read_text()
    assert '"/vehicle_cmd_safe"' in bridge
    assert '"/wheelchair_control_command"' in bridge


def test_canonical_qualification_transport_is_hard_locked_mock():
    source = LAUNCH.read_text()
    assert "'output_transport': 'mock'" in source
    assert "'auto_start': True" in source
    assert "CANONICAL_MOCK_FORBIDDEN" in source
    function = source.split("def stationary_command_chain_actions", 1)[1]
    function = function.split("# ── 7.", 1)[0]
    assert "LaunchConfiguration(" not in function
    assert "'output_transport': 'can'" not in function
