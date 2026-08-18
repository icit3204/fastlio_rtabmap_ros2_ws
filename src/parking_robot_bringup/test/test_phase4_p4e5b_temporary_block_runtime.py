from pathlib import Path

from parking_robot_bringup.phase4_p4e5b_temporary_block_runner import (
    command_mag,
    first_pair,
)


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "parking_robot_bringup" / "phase4_p4e5b_temporary_block_runner.py"
SETUP = ROOT / "setup.py"


def _command(stamp, linear, angular=0.0):
    return (stamp, 0, linear, 0.0, 0.0, 0.0, 0.0, angular)


def test_command_magnitude_uses_linear_and_angular_authority():
    assert command_mag(_command(1, 3.0, 4.0)) == 5.0


def test_first_pair_uses_later_sample_as_qualifying_reference():
    raw = _command(100, 0.2)
    safe = _command(120, 0.0)
    found = first_pair([(raw, safe, 20)], lambda r, s: command_mag(r) > .01 and command_mag(s) < .01)
    assert found["reference_ns"] == 120
    assert found["skew_ns"] == 20


def test_runner_has_no_navigate_to_pose_action_client():
    text = RUNNER.read_text(encoding="utf-8")
    assert "ActionClient(" not in text
    assert "MissionCancelRunner" in text


def test_runner_permits_only_clear_and_stop_fixture_modes():
    text = RUNNER.read_text(encoding="utf-8")
    assert 'if value not in ("CLEAR", "STOP")' in text
    assert 'mode("STOP")' in text
    assert 'mode("CLEAR")' in text


def test_runner_bounds_temporary_entry_and_recovery():
    text = RUNNER.read_text(encoding="utf-8")
    assert "1_000_000_000 <= stop_delta <= 1_500_000_000" in text
    assert "500_000_000 <= recovery_delta <= 1_000_000_000" in text
    assert "node.spin(.70)" in text


def test_runner_requires_mission_wide_active_latch():
    text = RUNNER.read_text(encoding="utf-8")
    assert 'latest_policy("ACTIVE", wp1_ns, uuid1)' in text
    assert '== "INITIAL_PRIMING"' in text
    assert "activation start reset on waypoint-1" in text


def test_runner_has_exactly_one_route_start_and_arm_boundary():
    text = RUNNER.read_text(encoding="utf-8")
    assert 'node.trigger("start")' in text
    assert "node.arm_gate()" in text
    assert '"route_count": node.route_count' in text
    assert '"cancel_count": node.cancel_count' in text


def test_runner_is_installed_as_evidence_tool():
    text = SETUP.read_text(encoding="utf-8")
    assert "phase4_p4e5b_temporary_block_runner = " in text

