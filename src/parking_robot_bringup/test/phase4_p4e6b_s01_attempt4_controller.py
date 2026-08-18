"""Dedicated, qualification-only S01 attempt-4 execution controller.

This module provides the complete prospective lifecycle as a dry/mock state
machine.  It has no ROS application control and never writes the real attempt
runtime marker in dry mode.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from phase4_p4e6b_hash_pinned_freeze import (
    REQUIRED_PINNED_IDENTITY, commit_pinned_freeze, load_expected_authority,
    sha256_file, assert_process_order,
)
from parking_robot_bringup.phase4_p4e6b_terminal_closure import (
    adjudicate_terminal_v2, physical_closure_from_rows,
)
from parking_robot_bringup.phase4_p4e6b_live_evidence import (
    authoritative_global_zero, load_genuine_runner_evidence,
    load_genuine_witness_outcome, post_zero_angular_zero, require_feedback_stale_age,
)

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
AUTHORITY_FILE = HERE / "phase4_p4e6b_s01_attempt4_authority.json"
RUNNER = HERE.parent / "parking_robot_bringup" / "phase4_p4e6b_health_failure_runner.py"
MATRIX_SOURCE = WORKSPACE / "src/parking_robot_bringup/launch/phase4_p4e6b_health_matrix.launch.py"
MATRIX_INSTALLED = WORKSPACE / "install/parking_robot_bringup/share/parking_robot_bringup/launch/phase4_p4e6b_health_matrix.launch.py"
WITNESS = HERE / "phase4_p4e6b_terminal_closure_witness.py"
RELAY = HERE.parent / "parking_robot_bringup" / "phase4_p4e6b_feedback_relay.py"
V2 = HERE.parent / "parking_robot_bringup" / "phase4_p4e6b_terminal_closure.py"
LIVE_EVIDENCE = HERE.parent / "parking_robot_bringup" / "phase4_p4e6b_live_evidence.py"
FREEZE = HERE / "phase4_p4e6b_hash_pinned_freeze.py"
CONTROLLER = Path(__file__).resolve()


def _sha(path: Path) -> str:
    return sha256_file(Path(path))


def actual_identities(*, controller_path: Path = CONTROLLER,
                      runner_path: Path = RUNNER,
                      matrix_installed: Path = MATRIX_INSTALLED,
                      witness_path: Path = WITNESS,
                      relay_path: Path = RELAY,
                      freeze_path: Path = FREEZE,
                      v2_path: Path = V2,
                      live_evidence_path: Path = LIVE_EVIDENCE) -> dict:
    """Hash the artifacts actually selected for the prospective attempt."""
    return {
        "runner": _sha(runner_path),
        "matrix": _sha(matrix_installed),
        "matrix_source": _sha(MATRIX_SOURCE),
        "matrix_installed": _sha(matrix_installed),
        "live_controller": _sha(controller_path),
        "s01_controller": _sha(controller_path),
        "freeze_controller": _sha(freeze_path),
        "witness": _sha(witness_path),
        "relay": _sha(relay_path),
        "terminal_v2": _sha(v2_path),
        "live_evidence": _sha(live_evidence_path),
    }


def runtime_import_binding() -> dict:
    """Non-effectful child proof of the installed runner import identity."""
    code = ("import hashlib, pathlib, sys, parking_robot_bringup.phase4_p4e6b_health_failure_runner as m; "
            "p=pathlib.Path(m.__file__); h=hashlib.sha256(p.read_bytes()).hexdigest(); "
            "print(__import__('json').dumps({'module_file':str(p),'module_sha256':h,'executable':sys.executable,'pythonpath':sys.path},sort_keys=True))")
    result = subprocess.run(["/usr/bin/python3", "-c", code], capture_output=True, text=True,
                            check=True, env=os.environ.copy())
    return json.loads(result.stdout.strip())


def matrix_binding() -> dict:
    return {"source": str(MATRIX_SOURCE), "source_sha256": _sha(MATRIX_SOURCE),
            "installed": str(MATRIX_INSTALLED), "installed_sha256": _sha(MATRIX_INSTALLED),
            "byte_identical": MATRIX_SOURCE.read_bytes() == MATRIX_INSTALLED.read_bytes()}


def _durable(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, sort_keys=True, indent=2) + "\n"
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8") as stream:
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(tmp, path)
    fd = os.open(str(path.parent), os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)
    if path.read_text(encoding="utf-8") != payload:
        raise RuntimeError("DRY_EVIDENCE_READBACK_MISMATCH")


class RecordingProcess:
    def __init__(self, role: str, pid: int, alive: bool = True):
        self.role, self.pid, self.alive = role, pid, alive
    def poll(self):
        return None if self.alive else 1


class RecordingFactory:
    """Safe process seam used only by B2AM tests."""
    def __init__(self, *, fail=None, ready=True, matrix_alive=True, runner_exit=0,
                 witness_outcome=True):
        self.fail, self.ready, self.matrix_alive_flag = fail, ready, matrix_alive
        self.runner_exit, self.witness_outcome = runner_exit, witness_outcome
        self.calls, self.processes = [], {}
    def spawn(self, role):
        self.calls.append(role)
        if self.fail == role: raise RuntimeError(role.upper() + "_Popen_FAILURE")
        process = RecordingProcess(role, 880000 + len(self.calls),
                                   alive=(role != "matrix" or self.matrix_alive))
        self.processes[role] = process
        return process
    def witness_ready(self):
        if not self.ready: raise TimeoutError("TERMINAL_WITNESS_READY_TIMEOUT")
        return True
    def runner_exit_code(self): return self.runner_exit
    def matrix_alive(self, process):
        if not self.matrix_alive_flag: raise RuntimeError("MATRIX_NOT_ALIVE")
        if process.poll() is not None: raise RuntimeError("MATRIX_NOT_ALIVE")
        return True
    def terminate_owned(self): return None


class LiveRosProcess:
    def __init__(self, role, process, stdout, stderr):
        self.role, self.process, self.stdout, self.stderr = role, process, stdout, stderr
        self.pid = process.pid
        self.pgid = os.getpgid(process.pid)
        self.sid = os.getsid(process.pid)
        self.episode_token = None
        self.episode_uuid = None
        self.ros_domain = None
        self.start_monotonic_ns = time.monotonic_ns()
        self.exit_monotonic_ns = None
        self.returncode = None
    def poll(self):
        value = self.process.poll()
        if value is not None and self.exit_monotonic_ns is None:
            self.exit_monotonic_ns, self.returncode = time.monotonic_ns(), value
        return value


class LiveRosFactory:
    """Exact owned subprocess authority for the future S01 path."""
    def __init__(self, output_dir: Path, *, domain: int, token: str, episode_uuid: str,
                 runner_path: Path = RUNNER, witness_path: Path = WITNESS,
                 matrix_launch: Path = MATRIX_INSTALLED, runner_command=None):
        self.output = Path(output_dir); self.output.mkdir(parents=True, exist_ok=True)
        self.domain, self.token, self.episode_uuid = domain, token, episode_uuid
        self.runner_path, self.witness_path, self.matrix_launch = Path(runner_path), Path(witness_path), Path(matrix_launch)
        self.runner_command = list(runner_command) if runner_command is not None else None
        self.calls, self.processes = [], {}
        self.env = os.environ.copy()
        self.env.update({"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1",
                         "P4E6B_EPISODE_TOKEN": token, "P4E6B_EPISODE_UUID": episode_uuid,
                         "PYTHONUNBUFFERED": "1"})
        self.ros2 = shutil.which("ros2", path=self.env.get("PATH"))

    def command(self, role: str) -> list[str]:
        if role == "witness":
            return [sys.executable, str(self.witness_path), "--output-dir",
                    str(self.output / "terminal_witness"), "--reason", "FEEDBACK_STALE"]
        if role == "matrix":
            if not self.ros2: raise RuntimeError("ROS2_EXECUTABLE_NOT_FOUND")
            return [str(Path(self.ros2).resolve()), "launch", "parking_robot_bringup",
                    "phase4_p4e6b_health_matrix.launch.py", "enable_health_runner:=false",
                    "case_id:=B-S01"]
        if role == "runner":
            if self.runner_command is not None:
                return list(self.runner_command)
            evidence = self.output / "runner"
            code = ("from pathlib import Path; "
                    "import parking_robot_bringup.phase4_p4e6b_health_failure_runner as m; "
                    f"m.execute_campaign('B-S01', Path({str(evidence)!r}), [])")
            return [sys.executable, "-c", code]
        raise ValueError(role)

    def spawn(self, role: str):
        argv = self.command(role); self.calls.append({"role": role, "argv": argv})
        out_dir = self.output / "process_logs"; out_dir.mkdir(parents=True, exist_ok=True)
        stdout = (out_dir / (role + ".stdout.log")).open("w", buffering=1)
        stderr = (out_dir / (role + ".stderr.log")).open("w", buffering=1)
        process = subprocess.Popen(argv, cwd=str(self.output), env=self.env,
                                   stdout=stdout, stderr=stderr, start_new_session=True)
        wrapped = LiveRosProcess(role, process, stdout, stderr)
        wrapped.episode_token = self.token
        wrapped.episode_uuid = self.episode_uuid
        wrapped.ros_domain = self.domain
        self.processes[role] = wrapped
        return wrapped

    def witness_ready(self, timeout=15):
        marker = self.output / "terminal_witness" / "TERMINAL_WITNESS_READY"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if marker.is_file(): return True
            process = self.processes.get("witness")
            if process and process.poll() is not None: raise RuntimeError("WITNESS_EXIT_BEFORE_READY")
            time.sleep(.05)
        raise TimeoutError("TERMINAL_WITNESS_READY_TIMEOUT")

    def matrix_alive(self, process, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is None: return True
            time.sleep(.05)
        raise RuntimeError("MATRIX_NOT_ALIVE")

    def runner_exit_code(self):
        process = self.processes.get("runner")
        if process is None: return None
        return process.poll()

    def runner_alive(self):
        process = self.processes.get("runner")
        return process is not None and process.poll() is None

    def wait_runner(self, timeout=30):
        process = self.processes.get("runner")
        if process is None: raise RuntimeError("RUNNER_NOT_STARTED")
        deadline = time.monotonic() + timeout
        while process.poll() is None and time.monotonic() < deadline: time.sleep(.05)
        if process.poll() is None: raise TimeoutError("RUNNER_TIMEOUT")
        return {"pid": process.pid, "pgid": process.pgid, "sid": process.sid,
                "start_monotonic_ns": process.start_monotonic_ns,
                "exit_monotonic_ns": process.exit_monotonic_ns,
                "returncode": process.returncode,
                "stdout": str(self.output / "process_logs" / "runner.stdout.log"),
                "stderr": str(self.output / "process_logs" / "runner.stderr.log")}

    def runner_evidence(self):
        return load_genuine_runner_evidence(self.output / "runner")

    def wait_witness_outcome(self, timeout=3):
        path = self.output / "terminal_witness" / "TERMINAL_WITNESS_OUTCOME_COMMITTED"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if path.is_file(): return load_genuine_witness_outcome(self.output / "terminal_witness")
            process = self.processes.get("witness")
            if process and process.poll() is not None: raise RuntimeError("TERMINAL_WITNESS_FAILURE")
            time.sleep(.05)
        raise TimeoutError("TERMINAL_WITNESS_OUTCOME_TIMEOUT")

    def cleanup_global_zero(self):
        self.terminate_owned()
        result = authoritative_global_zero()
        if not result["pass"]: raise RuntimeError("CLEANUP_GLOBAL_ZERO_FAILURE")
        return {"scan1": result["scan1"], "scan2": result["scan2"],
                "graph_clean": result["graph"]["clean"], "graph": result["graph"]}

    def terminate_owned(self):
        for process in list(self.processes.values())[::-1]:
            if process.poll() is None:
                try: os.killpg(process.pgid, signal.SIGTERM)
                except ProcessLookupError: pass
        deadline = time.monotonic() + 5
        for process in list(self.processes.values())[::-1]:
            while process.poll() is None and time.monotonic() < deadline: time.sleep(.05)
            if process.poll() is None:
                try: os.killpg(process.pgid, signal.SIGKILL)
                except ProcessLookupError: pass
            process.stdout.close(); process.stderr.close()


def adjudicate_live_s01_attempt(runner_evidence: dict, witness_outcome: dict,
                                v2_result: dict, physical_result: dict,
                                cleanup: dict) -> dict:
    """Single final PASS predicate for the live attempt path."""
    required = (
        runner_evidence.get("suppression_request_count") == 1,
        runner_evidence.get("injection_response_successful") is True,
        runner_evidence.get("injection_response_reason") == "INJECTED",
        runner_evidence.get("relay_injection_count") == 1,
        float(runner_evidence.get("feedback_stale_age_sec", -1)) > 2.0,
        runner_evidence.get("originating_reason") == "FEEDBACK_STALE",
        runner_evidence.get("cancel_count") == 1,
        runner_evidence.get("duplicate_cancel") == 0,
        runner_evidence.get("health_cancel_ack") == "HEALTH_CANCEL_ACK_ACCEPTED",
        witness_outcome.get("actual_drain_sec", 0) >= 0.250,
        v2_result.get("pass") is True,
        physical_result.get("pass") is True,
        math_is_finite(physical_result.get("rotation_rad")),
        witness_outcome.get("post_zero_angular_zero", False) is True,
        cleanup.get("scan1") == [] and cleanup.get("scan2") == [],
    )
    status = "CONSUMED_PASS" if all(required) else "CONSUMED_FAILED"
    return {"final_result": status, "predicate": list(required),
            "runner_evidence": runner_evidence, "witness_outcome": witness_outcome,
            "v2_result": v2_result, "physical_result": physical_result,
            "cleanup": cleanup}


def math_is_finite(value):
    try:
        return __import__("math").isfinite(float(value))
    except (TypeError, ValueError):
        return False


class S01Attempt4Controller:
    def __init__(self, output_dir: Path, *, authority_path: Path = AUTHORITY_FILE,
                 identity_overrides=None, preflight_zero=None, preflight_fn=None,
                 attempt_identity=None, attempt_number=4):
        self.output = Path(output_dir); self.output.mkdir(parents=True, exist_ok=True)
        self.authority_path = Path(authority_path)
        self.identity_overrides = dict(identity_overrides or {})
        self.preflight_zero = preflight_zero
        self.preflight_fn = preflight_fn
        self.attempt_identity = attempt_identity
        self.attempt_number = int(attempt_number)
        if self.attempt_number not in (4, 5):
            raise RuntimeError("S01_ATTEMPT_NUMBER_UNSUPPORTED")
        self.events = []

    def event(self, name, **extra):
        row = {"event": name, "monotonic_ns": time.monotonic_ns(), **extra}
        self.events.append(row); return row

    def _runtime_marker_name(self):
        return f"S01_ATTEMPT{self.attempt_number}_RUNTIME_STARTED"

    def _attempt_artifact(self, suffix):
        return self.output / f"S01_ATTEMPT{self.attempt_number}_{suffix}"

    def _freeze(self):
        expected = load_expected_authority(self.authority_path)
        actual = actual_identities()
        actual.update(self.identity_overrides)
        if "matrix_installed" in self.identity_overrides:
            actual["matrix"] = self.identity_overrides["matrix_installed"]
        self.event("EXPECTED_IDENTITIES_LOADED", keys=list(expected))
        self.event("ACTUAL_IDENTITIES_CALCULATED", identities=actual)
        extra_keys = ("s01_controller", "terminal_v2", "live_evidence")
        if any(expected.get(key) != actual.get(key) for key in extra_keys):
            raise RuntimeError("FINAL_FREEZE_IDENTITY_MISMATCH")
        head = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
        freeze = commit_pinned_freeze(self.output, expected, actual, head=head,
                                      dirty_summary={"dirty": bool(dirty), "sha256": hashlib.sha256(dirty.encode()).hexdigest()})
        self.event("FINAL_FREEZE_COMMITTED", **freeze)
        return freeze, expected, actual

    def _admit_preflight(self, mode):
        if mode == "LIVE":
            if self.preflight_zero is False:
                raise RuntimeError("GLOBAL_ZERO_PREFLIGHT_FAILURE")
            result = self.preflight_fn() if self.preflight_fn is not None else authoritative_global_zero()
            if not result.get("pass"):
                raise RuntimeError("GLOBAL_ZERO_PREFLIGHT_FAILURE")
            self.event("GLOBAL_ZERO_PREFLIGHT_PASS", authoritative=True,
                       scan1=result.get("scan1", []), scan2=result.get("scan2", []),
                       graph=result.get("graph", {}))
            return result
        if self.preflight_zero is False:
            raise RuntimeError("GLOBAL_ZERO_PREFLIGHT_FAILURE")
        self.event("GLOBAL_ZERO_PREFLIGHT_PASS", authoritative=False)
        return {"pass": True, "scan1": [], "scan2": [], "graph": {"clean": True}}

    def _write_attempt_marker(self, matrix, freeze):
        marker = self.output / self._runtime_marker_name()
        _durable(marker, {"event": self._runtime_marker_name(),
                          "attempt_identity": self.attempt_identity,
                          "matrix_pid": matrix.pid, "matrix_pgid": matrix.pgid,
                          "matrix_sid": matrix.sid, "episode_token": getattr(matrix, "episode_token", None),
                          "episode_uuid": getattr(matrix, "episode_uuid", None),
                          "ros_domain": getattr(matrix, "ros_domain", None),
                          "monotonic_ns": time.monotonic_ns(),
                          "freeze_sha256": freeze["freeze_artifact_sha256"]})
        return marker

    def _pre_runtime_failure(self, factory, exc):
        cleanup = {"pass": True, "classification": "NOT_RUN"}
        cleanup_error = None
        try:
            factory.terminate_owned()
            if isinstance(factory, LiveRosFactory):
                cleanup = factory.cleanup_global_zero()
        except Exception as cleanup_exc:
            cleanup_error = repr(cleanup_exc)
            cleanup = {"pass": False, "error": cleanup_error}
        result = {"final_result": "UNCONSUMED_PRE_RUNTIME_FAILURE",
                  "classification": "UNCONSUMED_PRE_RUNTIME_FAILURE",
                  "error": repr(exc), "attempt_consumed": False,
                  self._runtime_marker_name(): False,
                  "cleanup": cleanup, "events": self.events}
        if cleanup_error is not None:
            result["cleanup_failure"] = cleanup_error
        _durable(self._attempt_artifact("PRE_RUNTIME_FAILURE.json"), result)
        return result

    def execute(self, factory, *, mode="DRY", supervisor_authority=None,
                stop_before_marker=False, terminal=True, physical=True,
                runner_before_terminal=False, runner_exit_after_terminal=False,
                attempt_identity=None, preflight_fn=None):
        """One lifecycle implementation for recording and real factories."""
        mode = str(mode).upper()
        if attempt_identity is not None:
            self.attempt_identity = attempt_identity
        if preflight_fn is not None:
            self.preflight_fn = preflight_fn
        if mode == "LIVE":
            nonattempt_authorized = (
                stop_before_marker
                and supervisor_authority == "B2AN_NONATTEMPT_SMOKE"
            )
            effectful_authorized = (
                not stop_before_marker
                and supervisor_authority == f"SUPERVISOR_AUTHORIZED_S01_ATTEMPT{self.attempt_number}"
            )
            if not (nonattempt_authorized or effectful_authorized):
                raise RuntimeError("LIVE_MODE_AUTHORITY_DENIED")
        elif mode == "LIVE_SYNTHETIC" and supervisor_authority != "B2AO_SYNTHETIC_COMPLETION":
            raise RuntimeError("LIVE_MODE_AUTHORITY_DENIED")
        if mode == "LIVE" and not self.attempt_identity:
            raise RuntimeError("LIVE_ATTEMPT_IDENTITY_REQUIRED")
        self._admit_preflight(mode)
        freeze, expected, actual = self._freeze()
        try:
            witness_created = self.event("WITNESS_PROCESS_CREATED")
            witness = factory.spawn("witness")
            self.event("WITNESS_SPAWNED", pid=witness.pid, pgid=getattr(witness, "pgid", None), sid=getattr(witness, "sid", None))
            factory.witness_ready()
            witness_ready = self.event("TERMINAL_WITNESS_READY")
            matrix_created = self.event("MATRIX_PROCESS_CREATED")
            matrix = factory.spawn("matrix")
            factory.matrix_alive(matrix)
            self.event("MATRIX_ALIVE")
        except Exception as exc:
            return self._pre_runtime_failure(factory, exc)
        if mode == "LIVE" and stop_before_marker:
            self.event("B2AN_NONATTEMPT_MATRIX_SMOKE")
            factory.terminate_owned()
            self.event("CLEANUP_GLOBAL_ZERO")
            return {"pass": True, "mode": mode, "attempt_marker_written": False,
                    "attempt_consumed": False, self._runtime_marker_name(): False,
                    "events": self.events, "expected": expected, "actual": actual,
                    "matrix_command": factory.calls}
        marker = self.event("RUNTIME_MARKER_ELIGIBLE")
        if mode == "LIVE":
            self._write_attempt_marker(matrix, freeze)
            self.event(self._runtime_marker_name())
        runner_created = self.event("RUNNER_PROCESS_CREATED")
        runner = factory.spawn("runner")
        if mode == "DRY":
            if runner_before_terminal or (factory.runner_exit_code() != 0 and not runner_exit_after_terminal):
                self.event("RUNNER_FAILURE_BEFORE_TERMINAL")
                return self._result(False, expected, actual, freeze, witness_created, witness_ready, matrix_created, marker, runner_created)
            if not terminal: raise RuntimeError("SYNTHETIC_TERMINAL_MISSING")
            self.event("SYNTHETIC_FEEDBACK_STALE_TERMINAL")
            self.event("WITNESS_POST_FAILED_DRAIN", drain_sec=0.250)
            if factory.runner_exit_code() != 0: self.event("RUNNER_EXIT_AFTER_TERMINAL_WITNESS_DRAIN_CONTINUES")
            self.event("TERMINAL_WITNESS_OUTCOME_COMMITTED")
            if not physical: raise RuntimeError("PHYSICAL_CLOSURE_FAILURE")
            self.event("V2_TERMINAL_PASS"); self.event("PHYSICAL_CLOSURE_PASS", translation_m=0.01, rotation_rad=0.001, rotation_acceptance="REPORT_ONLY")
            self.event("CLEANUP_GLOBAL_ZERO")
            return self._result(True, expected, actual, freeze, witness_created, witness_ready, matrix_created, marker, runner_created)
        try:
            runner_wait = factory.wait_runner(timeout=45)
            self.event("RUNNER_EXIT", **runner_wait)
            runner_evidence = factory.runner_evidence()
            if isinstance(factory, LiveRosFactory):
                require_feedback_stale_age(runner_evidence)
            valid_origin = (
                runner_evidence.get("suppression_request_count") == 1 and
                runner_evidence.get("injection_response_successful") is True and
                runner_evidence.get("injection_response_reason") == "INJECTED" and
                runner_evidence.get("relay_injection_count") == 1 and
                runner_evidence.get("feedback_stale_age_sec") is not None and
                float(runner_evidence.get("feedback_stale_age_sec", -1)) > 2.0 and
                runner_evidence.get("originating_reason") == "FEEDBACK_STALE" and
                runner_evidence.get("cancel_count") == 1 and
                runner_evidence.get("duplicate_cancel") == 0 and
                runner_evidence.get("health_cancel_ack") == "HEALTH_CANCEL_ACK_ACCEPTED")
            if not valid_origin:
                self.event("RUNNER_FAILURE_BEFORE_TERMINAL")
                result = {"final_result": "CONSUMED_FAILED", "classification": "RUNNER_FAILURE_BEFORE_TERMINAL",
                          "runner_wait": runner_wait, "runner_evidence": runner_evidence}
            else:
                witness_outcome = factory.wait_witness_outcome(timeout=5)
                if witness_outcome.get("episode_token") not in (None, factory.token):
                    raise RuntimeError("TERMINAL_WITNESS_FAILURE")
                if "actual_drain_sec" not in witness_outcome:
                    start_ns = witness_outcome.get("drain_start_ns")
                    end_ns = witness_outcome.get("drain_end_ns")
                    if start_ns is None or end_ns is None:
                        raise RuntimeError("TERMINAL_WITNESS_FAILURE")
                    witness_outcome["actual_drain_sec"] = (int(end_ns) - int(start_ns)) / 1e9
                terminal = witness_outcome["terminal"]
                v2_result = adjudicate_terminal_v2(
                    origin_ns=terminal["origin_ns"], reason="FEEDBACK_STALE",
                    mission_id=terminal["mission_id"], route_id=terminal["route_id"],
                    waypoint=terminal["waypoint"], uuid=terminal["uuid"],
                    states=witness_outcome["states"], statuses=witness_outcome["statuses"],
                    drain_start_ns=witness_outcome["drain_start_ns"],
                    drain_end_ns=witness_outcome["drain_end_ns"],
                    source_result_canceled=(witness_outcome.get("source_result_canceled") is True))
                physical_result = physical_closure_from_rows(
                    vehicle_rows=witness_outcome.get("vehicle_rows", []),
                    applied_rows=witness_outcome.get("applied_rows", []),
                    odometry_rows=witness_outcome.get("odometry_rows", []),
                    terminal_ns=terminal["origin_ns"], drain_end_ns=witness_outcome["drain_end_ns"])
                witness_outcome["post_zero_angular_zero"] = post_zero_angular_zero(
                    witness_outcome, physical_result.get("anchor_ns", terminal["origin_ns"]))
                self.event("V2_TERMINAL_RESULT", result=v2_result)
                self.event("PHYSICAL_RESULT", result=physical_result)
                result = {"runner_wait": runner_wait, "runner_evidence": runner_evidence,
                          "witness_outcome": witness_outcome, "v2_result": v2_result,
                          "physical_result": physical_result}
            cleanup = factory.cleanup_global_zero()
            result["cleanup"] = cleanup
            if result.get("final_result") is None:
                evidence_result = adjudicate_live_s01_attempt(
                    result["runner_evidence"], result["witness_outcome"],
                    result["v2_result"], result["physical_result"], cleanup)
                evidence_result.update({"runner_wait": result["runner_wait"],
                                        "runner_evidence": result["runner_evidence"],
                                        "witness_outcome": result["witness_outcome"],
                                        "v2_result": result["v2_result"],
                                        "physical_result": result["physical_result"],
                                        "cleanup": cleanup})
                result = evidence_result
            else:
                result["final_result"] = "CONSUMED_FAILED"
            real_attempt = mode == "LIVE"
            result.update({"attempt_marker_written": real_attempt, "attempt_consumed": real_attempt,
                           self._runtime_marker_name(): real_attempt, "events": self.events})
            _durable(self._attempt_artifact("FINAL_RESULT.json"), result)
            return result
        except TimeoutError as exc:
            try: factory.terminate_owned()
            except Exception: pass
            real_attempt = mode == "LIVE"
            result = {"final_result": "CONSUMED_FAILED", "classification": "RUNNER_TIMEOUT",
                      "error": str(exc), "attempt_marker_written": real_attempt,
                      "attempt_consumed": real_attempt, self._runtime_marker_name(): real_attempt, "events": self.events}
            _durable(self._attempt_artifact("FINAL_RESULT.json"), result)
            return result
        except Exception as exc:
            try: factory.terminate_owned()
            except Exception: pass
            real_attempt = mode == "LIVE"
            classification = ("STALE_AGE_EVIDENCE_MISSING"
                              if "STALE_AGE_EVIDENCE_MISSING" in str(exc)
                              else "TERMINAL_WITNESS_FAILURE")
            result = {"final_result": "CONSUMED_FAILED", "classification": classification,
                      "error": str(exc), "attempt_marker_written": real_attempt,
                      "attempt_consumed": real_attempt, self._runtime_marker_name(): real_attempt, "events": self.events}
            _durable(self._attempt_artifact("FINAL_RESULT.json"), result)
            return result

    def execute_dry(self, factory: RecordingFactory, **kwargs):
        return self.execute(factory, mode="DRY", **kwargs)

    def execute_live_nonattempt(self, factory: LiveRosFactory):
        return self.execute(factory, mode="LIVE", supervisor_authority="B2AN_NONATTEMPT_SMOKE",
                            attempt_identity="B2AS_NONATTEMPT_SMOKE",
                            stop_before_marker=True)

    def execute_live_synthetic_completion(self, factory):
        """Run the live post-runner lifecycle against qualification fakes only.

        This intentionally uses the shared LIVE lifecycle while withholding the
        real attempt authority and the real consumption marker.
        """
        return self.execute(factory, mode="LIVE_SYNTHETIC",
                            supervisor_authority="B2AO_SYNTHETIC_COMPLETION",
                            attempt_identity="B2AS_SYNTHETIC_COMPLETION")

    def _execute_legacy_dry(self, factory: RecordingFactory, *, terminal=True,
                    physical=True, runner_before_terminal=False,
                    runner_exit_after_terminal=False) -> dict:
        """Run the actual dedicated entry point with only fake processes."""
        freeze, expected, actual = self._freeze()
        witness_created = self.event("WITNESS_PROCESS_CREATED")
        witness = factory.spawn("witness")
        self.event("WITNESS_SPAWNED", pid=witness.pid)
        factory.witness_ready()
        witness_ready = self.event("TERMINAL_WITNESS_READY")
        matrix_created = self.event("MATRIX_PROCESS_CREATED")
        matrix = factory.spawn("matrix")
        if matrix.poll() is not None: raise RuntimeError("MATRIX_EXITED_BEFORE_RUNTIME_MARKER")
        self.event("MATRIX_ALIVE")
        marker = self.event("RUNTIME_MARKER_ELIGIBLE")
        runner_created = self.event("RUNNER_PROCESS_CREATED")
        runner = factory.spawn("runner")
        if runner_before_terminal or (factory.runner_exit_code() != 0 and not runner_exit_after_terminal):
            self.event("RUNNER_FAILURE_BEFORE_TERMINAL")
            return self._result(False, expected, actual, freeze, witness_created, witness_ready, matrix_created, marker, runner_created)
        if not terminal: raise RuntimeError("SYNTHETIC_TERMINAL_MISSING")
        self.event("SYNTHETIC_FEEDBACK_STALE_TERMINAL")
        self.event("WITNESS_POST_FAILED_DRAIN", drain_sec=0.250)
        if factory.runner_exit_code() != 0:
            self.event("RUNNER_EXIT_AFTER_TERMINAL_WITNESS_DRAIN_CONTINUES")
        if not factory.witness_outcome: raise RuntimeError("WITNESS_OUTCOME_COMMIT_MISSING")
        self.event("TERMINAL_WITNESS_OUTCOME_COMMITTED")
        if not physical: raise RuntimeError("PHYSICAL_CLOSURE_FAILURE")
        self.event("V2_TERMINAL_PASS")
        self.event("PHYSICAL_CLOSURE_PASS", translation_m=0.01, rotation_rad=0.001,
                   rotation_acceptance="REPORT_ONLY")
        self.event("CLEANUP_GLOBAL_ZERO")
        return self._result(True, expected, actual, freeze, witness_created, witness_ready, matrix_created, marker, runner_created)

    def _result(self, passed, expected, actual, freeze, witness_created, witness_ready,
                matrix_created, marker, runner_created):
        order = [freeze["committed_ns"], witness_created["monotonic_ns"], witness_ready["monotonic_ns"],
                 matrix_created["monotonic_ns"], marker["monotonic_ns"], runner_created["monotonic_ns"]]
        return {"pass": passed, "expected": expected, "actual": actual, "events": self.events,
                "strict_order": all(a < b for a, b in zip(order, order[1:])),
                "attempt_marker_written": False, "attempt_consumed": False,
                self._runtime_marker_name(): False, "order_ns": order}
