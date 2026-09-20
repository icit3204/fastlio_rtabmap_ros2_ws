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
    assert source.count('"/r23_r5/collision_selected"') >= 2
    assert "r23_r5_fixed_collision_manager" in source
    assert "motion_aware_collision_mock" in source
    assert "r23_r5_motion_aware_collision_mock.yaml" in source
    assert "fixed, fixed_manager, experimental_mask, experimental, *common" in source


def test_live_sensor_topics_and_tf_authority_are_explicitly_selectable():
    source = LAUNCH.read_text()
    assert '"mid_obstacle_topic", default_value="/r23_r5/mid_obstacles"' in source
    assert '"tmini_scan_topic", default_value="/r23_r5/scan"' in source
    assert '"publish_test_static_tf", default_value="true"' in source
    assert 'condition=IfCondition(publish_test_static_tf)' in source


def test_gate_and_terminal_transport_contract():
    source = LAUNCH.read_text()
    gate = (ROOT / "config/r23_r5_gate_mock.yaml").read_text()
    terminal = (ROOT / "config/r23_r5_wheelchair_controller_mock.yaml").read_text()
    assert "safe_input_topic: /r23_r5/collision_selected" in gate
    assert "output_topic: /vehicle_cmd_safe" in gate
    assert 'executable="guarded_vehicle_cmd_gate"' in source
    assert 'executable="gate_to_labmate_bridge"' in source
    assert 'executable="wheelchair_controller_node"' in source
    assert "r23_r5_wheelchair_controller_mock.yaml" in source
    assert "output_transport: mock" in terminal
    assert "output_transport: can" not in terminal
    assert "auto_start: true" in terminal
    assert "mkmini_physical_ros_backend" not in source
    assert "SocketCAN" not in source


def test_qualified_inputs_and_real_authorities_unchanged():
    source = LAUNCH.read_text()
    assert "collision_monitor_dual_sensor.yaml" in source
    mock = (ROOT / "config/r23_r5_motion_aware_collision_mock.yaml").read_text()
    mask = (ROOT / "config/r23_r5_tmini_collision_self_mask.yaml").read_text()
    qualified_mock = (WORKSPACE_SRC / "robot_bringup/config/motion_aware_collision_mock.experimental.yaml").read_text()
    qualified_mask = (WORKSPACE_SRC / "robot_bringup/config/tmini_collision_self_mask.experimental.yaml").read_text()
    for token in ("slowdown_ratio: 0.30", "max_points: 3",
                  "deescalation_observations: 2", "minimum_turning_radius: 1.750",
                  "watchdog_zero_deadline_sec: 0.225"):
        assert token in mock and token in qualified_mock
    for token in ("footprint_x_min: -0.145", "footprint_x_max: 0.755",
                  "footprint_y_min: -0.300", "footprint_y_max: 0.300"):
        assert token in mask and token in qualified_mask
    assert "integration_mock_enabled: true" in mock
    assert "output_topic: /r23_r5/collision_selected" in mock
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
