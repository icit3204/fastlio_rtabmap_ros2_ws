import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from vehicle_cmd_safety.gate_core import AuthoritySnapshot, GateConfig, GateCore, Twist6


ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / "config" / "phase4_p4e_chassis_nav2_params.yaml"
PHASE2 = ROOT / "config" / "phase2_nav2_params.yaml"
GATE = ROOT / "config" / "phase4_p4e_gate_mock.yaml"
BT = ROOT / "behavior_trees" / "phase4_p4e_chassis_navigate_to_pose.xml"
BT_THROUGH = ROOT / "behavior_trees" / "phase4_p4e_chassis_navigate_through_poses.xml"
LAUNCH = ROOT / "launch" / "phase4_p4e_chassis_full_chain.launch.py"
PACKAGE_XML = ROOT / "package.xml"


def load_yaml(path):
    return yaml.safe_load(path.read_text())


def test_mppi_ackermann_profile_exact_installed_schema():
    follow = load_yaml(NAV)["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["motion_model"] == "Ackermann"
    assert follow["vx_min"] == 0.0
    assert follow["vx_max"] == 0.20
    assert follow["vy_max"] == 0.0
    assert follow["wz_max"] == 0.50
    assert follow["AckermannConstraints"] == {"min_turning_r": 1.0}


def test_mission_manager_progress_checker_is_the_only_phase4_local_policy():
    controller = load_yaml(NAV)["controller_server"]["ros__parameters"]
    assert controller["progress_checker_plugin"] == "mission_manager_progress_checker"
    assert "progress_checker_plugins" not in controller
    assert controller["mission_manager_progress_checker"] == {
        "plugin": "parking_robot_nav2_plugins::MissionManagerProgressChecker"
    }
    text = NAV.read_text()
    assert "nav2_controller::SimpleProgressChecker" not in text
    assert "nav2_controller::PoseProgressChecker" not in text
    assert "movement_time_allowance" not in text
    assert "required_movement_radius" not in text


def test_bringup_declares_custom_progress_checker_runtime_dependency():
    root = ET.parse(PACKAGE_XML).getroot()
    dependencies = {element.text for element in root.findall("exec_depend")}
    assert "parking_robot_nav2_plugins" in dependencies


def test_phase2_semantic_diff_is_bounded_to_chassis_profile():
    old, new = load_yaml(PHASE2), load_yaml(NAV)
    old_follow = old["controller_server"]["ros__parameters"]["FollowPath"]
    new_follow = new["controller_server"]["ros__parameters"]["FollowPath"]
    changed = {k for k in set(old_follow) | set(new_follow) if old_follow.get(k) != new_follow.get(k)}
    assert changed == {"motion_model", "vx_max", "wz_max", "AckermannConstraints"}
    for section in set(old) - {"controller_server", "behavior_server", "bt_navigator"}:
        assert new[section] == old[section]
    old_bt = old["bt_navigator"]["ros__parameters"].copy()
    new_bt = new["bt_navigator"]["ros__parameters"].copy()
    old_bt.pop("default_nav_to_pose_bt_xml")
    new_bt.pop("default_nav_to_pose_bt_xml")
    assert new_bt.pop("default_nav_through_poses_bt_xml") == "phase4_p4e_chassis_navigate_through_poses.xml"
    old_bt.pop("default_nav_through_poses_bt_xml")
    assert new_bt == old_bt


def test_behavior_server_is_wait_only_and_nonvelocity():
    behavior = load_yaml(NAV)["behavior_server"]["ros__parameters"]
    assert behavior["behavior_plugins"] == ["wait"]
    assert behavior["wait"]["plugin"] == "nav2_behaviors/Wait"
    assert not ({"spin", "backup", "drive_on_heading"} & set(behavior))


def test_custom_bt_preserves_structure_budget_and_only_reviewed_recovery():
    root = ET.parse(BT).getroot()
    tags = {node.tag for node in root.iter()}
    assert {"ComputePathToPose", "FollowPath", "ClearEntireCostmap", "Wait"} <= tags
    assert not ({"Spin", "BackUp", "DriveOnHeading"} & tags)
    outer = next(n for n in root.iter("RecoveryNode") if n.attrib.get("name") == "NavigateRecovery")
    assert outer.attrib["number_of_retries"] == "6"
    recovery_actions = next(n for n in root.iter("RoundRobin") if n.attrib.get("name") == "RecoveryActions")
    assert {n.tag for n in recovery_actions.iter()} <= {
        "RoundRobin", "Sequence", "ClearEntireCostmap", "Wait"
    }
    through_tags = {node.tag for node in ET.parse(BT_THROUGH).getroot().iter()}
    assert not ({"Spin", "BackUp", "DriveOnHeading"} & through_tags)


def test_gate_profile_changes_only_generic_angular_increase_rate():
    old = load_yaml(ROOT.parent / "vehicle_cmd_safety" / "config" / "phase4_p4c_gate_mock.yaml")
    new = load_yaml(GATE)
    old_p = old["guarded_vehicle_cmd_gate"]["ros__parameters"]
    new_p = new["guarded_vehicle_cmd_gate"]["ros__parameters"]
    changed = {k for k in set(old_p) | set(new_p) if old_p.get(k) != new_p.get(k)}
    assert changed == {"max_angular_increase_rate"}
    assert new_p["max_linear_increase_rate"] == 0.50
    assert new_p["max_angular_increase_rate"] == 0.50
    assert new_p["max_slew_dt_sec"] == 0.10


def _gate(linear_rate=0.5, angular_rate=0.5):
    return GateCore(GateConfig(max_forward_velocity=0.2, max_angular_velocity=0.5,
                               max_linear_increase_rate=linear_rate,
                               max_angular_increase_rate=angular_rate,
                               max_slew_dt_sec=0.1))


def _inside(command):
    return (-1e-12 <= command.linear_x <= 0.2 + 1e-12
            and abs(command.angular_z) <= 0.5 + 1e-12
            and abs(command.angular_z) <= command.linear_x + 1e-12)


def _transition(start, target, dt):
    core = _gate()
    core.last_output = Twist6(linear_x=start[0], angular_z=start[1])
    core.last_output_time = 0.0
    core.safe_command = Twist6(linear_x=target[0], angular_z=target[1])
    outputs = []
    now = 0.0
    for _ in range(1000):
        now += dt
        out = core._limited_output(now)
        outputs.append(out)
        core.last_output, core.last_output_time = out, now
        if math.isclose(out.linear_x, target[0], abs_tol=1e-12) and math.isclose(out.angular_z, target[1], abs_tol=1e-12):
            break
    assert len(outputs) < 1000
    assert all(_inside(out) for out in outputs if not out.is_zero())
    return outputs


def test_real_gate_core_closes_adapter_cone_for_transition_matrix_and_dt():
    transitions = [
        ((0, 0), (.2, 0)), ((0, 0), (.2, .2)), ((0, 0), (.2, -.2)),
        ((.2, 0), (.2, .2)), ((.2, .2), (.2, 0)),
        ((.2, .1), (.2, .2)), ((.2, .2), (.2, .1)),
        ((.2, .2), (.2, -.2)), ((.1, 0), (.2, 0)), ((.2, 0), (.1, 0)),
        ((.1, .05), (.2, .2)), ((.2, .1), (.1, .1)),
        ((.1, .1), (.2, .05)), ((.2, .2), (.1, .05)),
    ]
    for dt in (0.001, 0.05, 0.1, 0.25):
        for start, target in transitions:
            _transition(start, target, dt)


def test_historical_startup_regression_old_fails_new_stays_feasible():
    target = Twist6(linear_x=0.04711674153804779, angular_z=-0.011280647478997707)
    dt = 0.010417315002996474
    old, new = _gate(0.5, 1.0), _gate(0.5, 0.5)
    for core in (old, new):
        core.last_output = Twist6()
        core.last_output_time = 0.0
        core.safe_command = target
    old_out = old._limited_output(dt)
    new_out = new._limited_output(dt)
    assert abs(old_out.linear_x / old_out.angular_z) < 1.0
    assert math.isclose(abs(old_out.linear_x / old_out.angular_z), 0.5, rel_tol=1e-9)
    assert abs(new_out.linear_x / new_out.angular_z) >= 1.0 - 1e-12


def test_immediate_zero_paths_remain_real_core_semantics():
    core = _gate()
    core.last_output = Twist6(linear_x=.2, angular_z=.2)
    core.last_output_time = 1.0
    core.safe_command = Twist6()
    assert core._limited_output(1.001).is_zero()


def _armed_gate():
    core = _gate()
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), 1.0)
    core.set_safe_command(Twist6(linear_x=.2, angular_z=.2), 2.1)
    for name in ("localization", "controller", "collision"):
        core.set_permission(name, True, 2.1)
    assert core.request_arm(True, 2.2)[0]
    core.tick(2.21)
    return core


def test_fault_disarm_stale_and_permission_failure_zero_are_immediate():
    disarm = _armed_gate()
    assert disarm.request_arm(False, 2.22)[0]
    assert disarm.tick(2.23).output.is_zero()

    stale = _armed_gate()
    assert stale.tick(2.50).output.is_zero()
    assert stale.state == "FAULT"

    permission = _armed_gate()
    permission.set_safe_command(Twist6(linear_x=.2, angular_z=.2), 2.22)
    permission.set_permission("controller", False, 2.22)
    assert permission.tick(2.23).output.is_zero()
    assert permission.state == "FAULT"

    invalid = _armed_gate()
    invalid.set_safe_command(Twist6(linear_x=math.nan), 2.22)
    assert invalid.tick(2.23).output.is_zero()
    assert invalid.state == "FAULT"


def test_collision_monitor_transformations_preserve_domain():
    for v, w in ((0, 0), (.2, 0), (.2, .2), (.1, -.05)):
        source = Twist6(linear_x=v, angular_z=w)
        assert _inside(source)
        clear = source
        slowdown = Twist6(linear_x=.30 * v, angular_z=.30 * w)
        stop = Twist6()
        assert all(_inside(item) for item in (clear, slowdown, stop))
    collision = load_yaml(ROOT / "config" / "phase4_p4e_collision_monitor_scan.yaml")
    text = str(collision)
    assert "30.0" in text and "0.3" in text


def test_launch_wires_only_new_profiles_and_existing_safe_chain():
    source = LAUNCH.read_text()
    ast.parse(source)
    for required in ("phase4_p4e_chassis_nav2_params.yaml",
                     "phase4_p4e_chassis_navigate_to_pose.xml",
                     "phase4_p4e_chassis_navigate_through_poses.xml",
                     "phase4_p4e_gate_mock.yaml",
                     "phase4_p4e_collision_monitor_scan.yaml",
                     "mock_wheelchair_cmd_adapter", "phase4_vehicle_cmd_fake_base",
                     "mission_manager_node"):
        assert required in source
    assert "plan_nav" not in source
    assert source.count('executable="mission_manager_node"') == 1
