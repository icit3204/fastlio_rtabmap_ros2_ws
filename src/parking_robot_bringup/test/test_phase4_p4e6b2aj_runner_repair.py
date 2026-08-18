import ast
import json
import subprocess
import time
from pathlib import Path

import pytest

import parking_robot_bringup.phase4_p4e6b_health_failure_runner as runner
from parking_robot_bringup.phase4_p4e6b_health_failure_runner import SCENARIOS, nz
from phase4_p4e6b_terminal_closure_controller import (
    assert_matrix_after_freeze, commit_final_freeze,
)


ROOT = Path(__file__).parents[1]
RUNNER = ROOT / "parking_robot_bringup" / "phase4_p4e6b_health_failure_runner.py"


def test_nz_scalar_and_twist_sequence_semantics():
    assert not nz(0.0)
    assert not nz([0.0] * 6)
    assert not nz([1.0e-6] * 6)
    assert nz(1.0e-5)
    assert nz([0.0, 0.0, 0.0, 0.0, 0.0, -1.0e-5])


def test_prospective_runner_self_contains_nz_binding():
    source = RUNNER.read_text()
    assert "def nz(values):" in source
    assert "runner_module.nz" not in source
    assert 'globals()["nz"]' not in source


def test_active_case_paths_have_no_external_binding_requirement():
    for case in ("B-S01", "B-S02", "B-S03", "B-C01"):
        assert case in SCENARIOS
    compile(source := RUNNER.read_text(), str(RUNNER), "exec")
    assert "NameError" not in source


def _fake_runner(*, raw_nonzero=True, safe_nonzero=True, gate_nonzero=False,
                 mock_nonzero=False, terminal=False):
    node = object.__new__(runner.HealthRuntimeRunner)
    now = time.monotonic_ns()
    twist = lambda nonzero: (now, 0, 0.1 if nonzero else 0.0, 0, 0, 0, 0)
    node.samples = {
        "/cmd_vel_nav_raw": [twist(raw_nonzero)] * 3,
        "/cmd_vel_nav_safe": [(now - 600_000_000, 0, 0.1 if safe_nonzero else 0.0, 0, 0, 0, 0),
                               (now - 300_000_000, 0, 0.1 if safe_nonzero else 0.0, 0, 0, 0, 0),
                               twist(safe_nonzero)],
        "/vehicle_cmd_safe": [twist(gate_nonzero)],
        "/wheelchair_control_command_mock": [(now, 0, 0.1 if mock_nonzero else 0.0)],
    }
    node.permissions = {topic: (True, now) for topic in runner.TOPICS[2:5]}
    node.diag_conditions = {"vehicle_cmd_safety/guarded_vehicle_cmd_gate":
                            (now, {"state": "DISARMED", "fault_latched": "false"})}
    node.states = ([{"monotonic_ns": now, "mission_id": "m", "route_id": "r",
                     "waypoint_index": 0, "state_name": "CANCELLING",
                     "reason_code": "OTHER", "active_goal_uuid": "u"}] if terminal else [])
    node.evidence = lambda *args, **kwargs: {"event": args[0] if args else ""}
    node.emit = lambda *args, **kwargs: args[1] if len(args) > 1 else kwargs
    return node


def test_active_precondition_accepts_current_nonzero_raw_and_safe(monkeypatch):
    monkeypatch.setattr(runner.rclpy, "spin_once", lambda *args, **kwargs: None)
    node = _fake_runner()
    result = node.wait_active_case_precondition({"monotonic_ns": time.monotonic_ns()-700_000_000,
                                                 "mission_id": "m", "route_id": "r",
                                                 "waypoint_index": 0, "active_goal_uuid": "u"}, .1)
    assert result["event"] == "P4E6B_ACTIVE_PRECONDITION_PASS"


@pytest.mark.parametrize("kwargs", [
    {"raw_nonzero": False}, {"safe_nonzero": False},
    {"gate_nonzero": True}, {"mock_nonzero": True},
])
def test_active_precondition_fails_closed_for_invalid_commands(monkeypatch, kwargs):
    monkeypatch.setattr(runner.rclpy, "spin_once", lambda *args, **kwargs: None)
    node = _fake_runner(**kwargs)
    with pytest.raises(RuntimeError):
        node.wait_active_case_precondition({"monotonic_ns": time.monotonic_ns()-700_000_000,
                                            "mission_id": "m", "route_id": "r",
                                            "waypoint_index": 0, "active_goal_uuid": "u"}, .002)


def test_active_precondition_fails_closed_on_competing_terminal(monkeypatch):
    monkeypatch.setattr(runner.rclpy, "spin_once", lambda *args, **kwargs: None)
    node = _fake_runner(terminal=True)
    with pytest.raises(RuntimeError, match="PRE_INJECTION_COMPETING_REASON"):
        node.wait_active_case_precondition({"monotonic_ns": time.monotonic_ns()-700_000_000,
                                            "mission_id": "m", "route_id": "r",
                                            "waypoint_index": 0, "active_goal_uuid": "u"}, .1)


def test_pyflakes_has_no_undefined_runtime_globals():
    result = subprocess.run(["/usr/bin/python3", "-m", "pyflakes", str(RUNNER)],
                            text=True, capture_output=True)
    assert "undefined name" not in result.stdout + result.stderr


def test_freeze_barrier_commits_durably_before_matrix(tmp_path):
    record = commit_final_freeze(tmp_path, {
        "HEAD": "head-sha", "runner": "runner-sha",
        "matrix": "matrix-sha", "controller": "controller-sha",
    })
    assert record["committed"]
    assert (tmp_path / "FINAL_FREEZE_COMMITTED").is_file()
    assert_matrix_after_freeze(freeze_committed_ns=10,
                               matrix_created_ns=20,
                               runtime_started_ns=30)


@pytest.mark.parametrize("identities", [
    {"runner": "r", "matrix": "m", "controller": "c"},
    {"HEAD": "h", "runner": "r", "matrix": "m"},
    {"HEAD": "h", "runner": "r", "matrix": "m", "controller": ""},
])
def test_freeze_barrier_missing_identity_denies(tmp_path, identities):
    with pytest.raises(RuntimeError):
        commit_final_freeze(tmp_path, identities)


@pytest.mark.parametrize("values", [(10, 10, 20), (10, 20, 20), (30, 20, 40)])
def test_process_order_is_strict(values):
    with pytest.raises(RuntimeError):
        assert_matrix_after_freeze(freeze_committed_ns=values[0],
                                   matrix_created_ns=values[1],
                                   runtime_started_ns=values[2])
