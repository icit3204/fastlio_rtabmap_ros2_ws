from pathlib import Path

import yaml

from parking_robot_bringup.selected_collision_status import selected_validity


PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE.parent
LAUNCH = PACKAGE / "launch" / "r3h_physical_navigation.launch.py"
PHYSICAL = PACKAGE / "config" / "r3h_motion_aware_collision_physical.yaml"
GATE = SRC / "vehicle_cmd_safety" / "config" / "r3h_physical_gate.yaml"
FIXED = SRC / "robot_bringup" / "config" / "collision_monitor_dual_sensor.yaml"
ENFORCER = SRC / "robot_bringup" / "scripts" / "motion_aware_collision_mock.py"


def params(path, node):
    return yaml.safe_load(path.read_text())[node]["ros__parameters"]


def test_default_and_invalid_selector_contract():
    source = LAUNCH.read_text()
    assert 'DeclareLaunchArgument("collision_monitor_mode", default_value="fixed_qualified")' in source
    assert 'if mode not in ("fixed_qualified", "motion_aware_experimental")' in source
    assert "raise RuntimeError" in source
    assert 'LogInfo(msg=["ACTIVE_COLLISION_MODE=", collision_monitor_mode])' in source


def test_both_modes_feed_the_same_unchanged_gate_input():
    fixed = params(FIXED, "collision_monitor")
    physical = params(PHYSICAL, "motion_aware_collision_mock")
    gate = params(GATE, "guarded_vehicle_cmd_gate")
    assert fixed["cmd_vel_in_topic"] == "/cmd_vel_nav"
    assert fixed["cmd_vel_out_topic"] == "/cmd_vel"
    assert physical["input_cmd_topic"] == "/cmd_vel_nav"
    assert physical["output_topic"] == "/cmd_vel"
    assert gate["safe_input_topic"] == "/cmd_vel"
    assert gate["output_topic"] == "/vehicle_cmd_safe"


def test_physical_profile_is_exact_qualified_r23_contract():
    physical = params(PHYSICAL, "motion_aware_collision_mock")
    assert physical["physical_enforcement_enabled"] is True
    assert physical["integration_mock_enabled"] is False
    assert physical["slowdown_ratio"] == 0.30
    assert physical["max_points"] == 3
    assert physical["deescalation_observations"] == 2
    assert physical["minimum_turning_radius"] == 1.75
    assert physical["command_timeout_sec"] == 0.250
    assert physical["watchdog_zero_deadline_sec"] == 0.225
    assert physical["mid_topic"] == "/cloud_registered_nav2_obstacles"
    assert physical["tmini_topic"] == "/scan_collision_experimental"


def test_selector_is_mutually_exclusive_and_fixed_path_is_preserved():
    source = LAUNCH.read_text()
    fixed_condition = "' == 'fixed_qualified'"
    experimental_condition = "' == 'motion_aware_experimental'"
    assert source.count(fixed_condition) >= 4
    assert source.count(experimental_condition) >= 3
    assert source.count('executable="collision_monitor"') == 1
    assert source.count('executable="motion_aware_collision_mock"') == 1
    assert source.count('executable="guarded_vehicle_cmd_gate"') == 1
    assert source.count('executable="mkmini_physical_ros_backend"') == 1
    assert "collision_monitor_dual_sensor.yaml" in source


def test_experimental_validity_is_fail_closed_and_single_authority():
    source = LAUNCH.read_text()
    assert source.count('"validity_topic": "/system/collision_monitor_valid"') == 1
    assert source.count('"validity_output_topic": "/system/collision_monitor_valid"') == 1
    assert '"publish_validity": True' in source
    assert '"publish_validity": False' in source
    status = (PACKAGE / "parking_robot_bringup" / "selected_collision_status.py").read_text()
    assert "validity.data = selected_validity(state)" in status
    assert 'if state == "STALE":' in status
    assert 'ACTIVE_COLLISION_MODE={self.mode}' in status
    assert selected_validity("CLEAR") is True
    assert selected_validity("SLOW") is True
    assert selected_validity("STOP") is True
    assert selected_validity("STALE") is False


def test_enforcer_cannot_bypass_gate_or_enable_physical_accidentally():
    source = ENFORCER.read_text()
    assert '"physical_enforcement_enabled": False' in source
    assert '(physical_enforcement and output_topic == "/cmd_vel")' in source
    assert '"/vehicle_cmd_safe"' in source
    assert "mock and physical integration modes are exclusive" in source
    assert "mkmini_physical_ros_backend" not in source


def test_r22_perception_and_production_radius_are_unchanged():
    fixed = params(FIXED, "collision_monitor")
    assert fixed["mid360_body_cloud"]["topic"] == "/cloud_registered_nav2_obstacles"
    assert fixed["tmini_scan"]["topic"] == "/scan"
    nav = (SRC / "robot_bringup" / "config" / "nav2_common.yaml").read_text()
    assert "minimum_turning_radius: 1.75" in nav
