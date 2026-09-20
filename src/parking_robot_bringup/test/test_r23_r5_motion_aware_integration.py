from pathlib import Path
import sys

from geometry_msgs.msg import Twist

ROOT = Path(__file__).parents[1]
WORKSPACE_SRC = ROOT.parent
LAUNCH = ROOT / "launch/r23_r5_motion_aware_chain_mock.launch.py"
sys.path.insert(0, str(ROOT))

from parking_robot_bringup.selected_collision_status import classify_selected


def twist(v, w):
    message = Twist()
    message.linear.x = float(v)
    message.angular.z = float(w)
    return message


def test_selected_status_contract():
    assert classify_selected(twist(.04, 0), twist(.04, 0), input_age=.01, output_age=.01) == "CLEAR"
    assert classify_selected(twist(.04, 0), twist(.012, 0), input_age=.01, output_age=.01) == "SLOW"
    assert classify_selected(twist(.04, 0), twist(0, 0), input_age=.01, output_age=.01) == "STOP"
    assert classify_selected(twist(.04, 0), twist(.04, 0), input_age=.26, output_age=.01) == "STALE"


def test_selector_is_explicit_exclusive_and_defaults_fixed():
    source = LAUNCH.read_text()
    assert 'default_value="fixed_qualified"' in source
    assert 'VALID_MODES = ("fixed_qualified", "motion_aware_experimental")' in source
    assert "raise RuntimeError" in source
    assert source.count('"/r23_r5/collision_selected"') >= 3
    assert "r23_r5_fixed_collision_manager" in source
    assert "motion_aware_collision_mock" in source
    assert '"integration_mock_enabled": True' in source
    assert "fixed, fixed_manager, experimental_mask, experimental, *common" in source


def test_gate_and_terminal_transport_contract():
    source = LAUNCH.read_text()
    gate = (ROOT / "config/r23_r5_gate_mock.yaml").read_text()
    assert "safe_input_topic: /r23_r5/collision_selected" in gate
    assert "output_topic: /vehicle_cmd_safe" in gate
    assert 'executable="guarded_vehicle_cmd_gate"' in source
    assert 'executable="gate_to_labmate_bridge"' in source
    assert 'executable="wheelchair_controller_node"' in source
    assert '"output_transport": "mock"' in source
    assert "mkmini_physical_ros_backend" not in source
    assert "SocketCAN" not in source


def test_qualified_inputs_and_real_authorities_unchanged():
    source = LAUNCH.read_text()
    assert "collision_monitor_dual_sensor.yaml" in source
    assert "motion_aware_collision_mock.experimental.yaml" in source
    assert "tmini_collision_self_mask.experimental.yaml" in source
    fixed = (WORKSPACE_SRC / "robot_bringup/config/collision_monitor_dual_sensor.yaml").read_text()
    nav = (WORKSPACE_SRC / "robot_bringup/config/nav2_common.yaml").read_text()
    assert "minimum_turning_radius: 1.75" in nav
    assert "Phase5Stop" in fixed and "Phase5Slow" in fixed
    assert "/cmd_vel_motion_aware_mock" not in nav
    assert "parking_robot_mission_manager" not in source


def test_observability_exposes_mode_and_state():
    status = (ROOT / "parking_robot_bringup/selected_collision_status.py").read_text()
    assert 'KeyValue(key="active_collision_mode"' in status
    assert 'KeyValue(key="collision_state"' in status
    assert '"CLEAR"' in status and '"SLOW"' in status
    assert '"STOP"' in status and '"STALE"' in status
