import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

import phase4_p4e6b_s01_attempt4_controller as c


def _write_child(path, value):
    return (
        "import json, pathlib, sys, time; "
        f"p=pathlib.Path({str(path)!r}); p.parent.mkdir(parents=True, exist_ok=True); "
        f"p.write_text(json.dumps({value!r}) if isinstance({value!r}, dict) else str({value!r})); "
        "time.sleep(30)"
    )


def _valid_evidence(token, translation=0.001, include_applied=True,
                    outcome=True, delay=0.35):
    root = {
        "episode_token": token,
        "terminal": {"origin_ns": 100, "mission_id": "m", "route_id": "r",
                      "waypoint": 1, "uuid": "u"},
        "states": [
            {"mission_id": "m", "route_id": "r", "waypoint_index": 1,
             "active_goal_uuid": "u", "monotonic_ns": 110,
             "state_name": "CANCELLING", "reason_code": "FEEDBACK_STALE"},
            {"mission_id": "m", "route_id": "r", "waypoint_index": 1,
             "active_goal_uuid": "u", "monotonic_ns": 120,
             "state_name": "CANCELLING", "reason_code": "HEALTH_CANCEL_ACK_ACCEPTED"},
            {"mission_id": "m", "route_id": "r", "waypoint_index": 1,
             "active_goal_uuid": "u", "monotonic_ns": 130,
             "state_name": "FAILED", "reason_code": "FEEDBACK_STALE"},
        ],
        "statuses": [{"goal_uuid": "u", "status_name": "CANCELING", "monotonic_ns": 125},
                     {"goal_uuid": "u", "status_name": "CANCELED", "monotonic_ns": 140}],
        "drain_start_ns": 1_000,
        "drain_end_ns": 251_000_000,
        "vehicle_rows": [
            {"monotonic_ns": 90, "linear_x": 0.1, "linear_y": 0, "linear_z": 0,
             "angular_x": 0, "angular_y": 0, "angular_z": 0},
            {"monotonic_ns": 220, "linear_x": 0, "linear_y": 0, "linear_z": 0,
             "angular_x": 0, "angular_y": 0, "angular_z": 0}],
        "applied_rows": ([{"monotonic_ns": 90, "linear_x": 0.1, "linear_y": 0,
                            "linear_z": 0, "angular_x": 0, "angular_y": 0,
                            "angular_z": 0},
                           {"monotonic_ns": 220, "linear_x": 0, "linear_y": 0,
                            "linear_z": 0, "angular_x": 0, "angular_y": 0,
                            "angular_z": 0}] if include_applied else []),
        "odometry_rows": [{"monotonic_ns": 220, "x": 0, "y": 0, "yaw": 0},
                           {"monotonic_ns": 230_000_000, "x": translation, "y": 0, "yaw": 0.001}],
        "post_zero_commanded_angular": 0,
        "post_zero_applied_angular": 0,
        "post_zero_angular_zero": True,
        "delay": delay,
        "write_outcome": outcome,
    }
    return root


class FakeCompletionFactory:
    """Subprocess-backed fake factory; no ROS or product endpoint is used."""
    def __init__(self, root, *, runner_evidence=None, outcome=None,
                 runner_rc=0, cleanup=None):
        self.output = Path(root)
        self.token = "B2AO_FAKE_TOKEN"
        self.calls = []
        self.processes = {}
        self.runner_rc = runner_rc
        self.runner_evidence_data = runner_evidence or {
            "suppression_request_count": 1,
            "injection_response_successful": True,
            "injection_response_reason": "INJECTED",
            "relay_injection_count": 1,
            "feedback_stale_age_sec": 2.015479,
            "originating_reason": "FEEDBACK_STALE",
            "cancel_count": 1,
            "duplicate_cancel": 0,
            "health_cancel_ack": "HEALTH_CANCEL_ACK_ACCEPTED",
        }
        self.outcome_data = outcome if outcome is not None else _valid_evidence(self.token)
        self.cleanup_data = cleanup

    def _spawn_child(self, role):
        base = self.output / role
        base.mkdir(parents=True, exist_ok=True)
        if role == "witness":
            ready = base / "TERMINAL_WITNESS_READY"
            outcome = base / "TERMINAL_WITNESS_OUTCOME_COMMITTED"
            payload = json.dumps(self.outcome_data)
            if self.outcome_data.get("write_outcome", True):
                completion = (f"time.sleep({float(self.outcome_data.get('delay', .35))!r}); "
                              f"pathlib.Path({str(outcome)!r}).write_text({payload!r}); ")
            else:
                completion = "time.sleep(30); "
            script = ("import pathlib,time; "
                      f"p=pathlib.Path({str(ready)!r}); p.write_text('READY'); "
                      + completion + "time.sleep(30)")
        elif role == "runner":
            evidence = base / "runner_evidence.json"
            payload = json.dumps(self.runner_evidence_data)
            script = (f"import pathlib; pathlib.Path({str(evidence)!r}).write_text({payload!r}); "
                      f"raise SystemExit({self.runner_rc})")
        else:
            script = "import time; time.sleep(30)"
        proc = subprocess.Popen([sys.executable, "-c", script], cwd=str(self.output),
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                start_new_session=True)
        wrapped = c.LiveRosProcess(role, proc, open(os.devnull, "w"), open(os.devnull, "w"))
        self.processes[role] = wrapped
        self.calls.append(role)
        return wrapped

    def spawn(self, role):
        return self._spawn_child(role)

    def witness_ready(self, timeout=3):
        path = self.output / "witness" / "TERMINAL_WITNESS_READY"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if path.exists(): return True
            time.sleep(.01)
        raise TimeoutError("TERMINAL_WITNESS_READY_TIMEOUT")

    def matrix_alive(self, process, timeout=1):
        if process.poll() is not None: raise RuntimeError("MATRIX_NOT_ALIVE")
        return True

    def wait_runner(self, timeout=3):
        process = self.processes["runner"]
        deadline = time.monotonic() + timeout
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(.01)
        if process.poll() is None: raise TimeoutError("RUNNER_TIMEOUT")
        return {"pid": process.pid, "pgid": process.pgid, "sid": process.sid,
                "start_monotonic_ns": process.start_monotonic_ns,
                "exit_monotonic_ns": process.exit_monotonic_ns,
                "returncode": process.returncode}

    def runner_exit_code(self):
        return self.processes["runner"].poll()

    def runner_alive(self):
        return self.processes["runner"].poll() is None

    def runner_evidence(self):
        return json.loads((self.output / "runner" / "runner_evidence.json").read_text())

    def wait_witness_outcome(self, timeout=3):
        path = self.output / "witness" / "TERMINAL_WITNESS_OUTCOME_COMMITTED"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if path.exists(): return json.loads(path.read_text())
            time.sleep(.01)
        raise TimeoutError("TERMINAL_WITNESS_OUTCOME_TIMEOUT")

    def terminate_owned(self):
        for process in self.processes.values():
            if process.poll() is None:
                process.process.terminate()
        for process in self.processes.values():
            try: process.process.wait(timeout=2)
            except subprocess.TimeoutExpired: process.process.kill()

    def cleanup_global_zero(self):
        if self.cleanup_data is not None:
            self.terminate_owned()
            return self.cleanup_data
        self.terminate_owned()
        time.sleep(.02)
        return {"scan1": [], "scan2": [], "graph_clean": True}


def run_case(tmp_path, **kwargs):
    factory = FakeCompletionFactory(tmp_path / "fake", **kwargs)
    result = c.S01Attempt4Controller(tmp_path / "episode").execute_live_synthetic_completion(factory)
    assert result["S01_ATTEMPT4_RUNTIME_STARTED"] is False
    assert not (tmp_path / "episode" / "S01_ATTEMPT4_RUNTIME_STARTED").exists()
    return result, factory


def test_subprocess_full_completion_uses_shared_live_post_runner(tmp_path):
    result, factory = run_case(tmp_path)
    assert result["final_result"] == "CONSUMED_PASS"
    assert factory.calls == ["witness", "matrix", "runner"]
    assert result["runner_wait"]["returncode"] == 0
    assert result["v2_result"]["pass"] is True
    assert result["physical_result"]["pass"] is True
    assert result["cleanup"]["scan1"] == []


def test_early_runner_failure_cannot_be_rescued(tmp_path):
    result, _ = run_case(tmp_path, runner_evidence={"suppression_request_count": 0}, runner_rc=7)
    assert result["final_result"] == "CONSUMED_FAILED"
    assert result["classification"] == "RUNNER_FAILURE_BEFORE_TERMINAL"


def test_witness_timeout_fails_closed(tmp_path):
    outcome = _valid_evidence("B2AO_FAKE_TOKEN", outcome=False)
    result, _ = run_case(tmp_path, outcome=outcome)
    assert result["final_result"] == "CONSUMED_FAILED"


def test_physical_failure_fails_closed(tmp_path):
    result, _ = run_case(tmp_path, outcome=_valid_evidence("B2AO_FAKE_TOKEN", translation=.03))
    assert result["final_result"] == "CONSUMED_FAILED"


def test_cleanup_failure_prevents_pass(tmp_path):
    result, _ = run_case(tmp_path, cleanup={"scan1": ["matrix"], "scan2": ["matrix"], "graph_clean": False})
    assert result["final_result"] == "CONSUMED_FAILED"


@pytest.mark.parametrize("rc", [0, 9])
def test_live_process_return_code_is_actual(tmp_path, rc):
    factory = c.LiveRosFactory(tmp_path / str(rc), domain=91, token="t", episode_uuid="u",
                               runner_command=[sys.executable, "-c", f"raise SystemExit({rc})"])
    process = factory.spawn("runner")
    waited = factory.wait_runner(timeout=3)
    assert factory.runner_exit_code() == rc
    assert waited["returncode"] == rc
    factory.terminate_owned()
