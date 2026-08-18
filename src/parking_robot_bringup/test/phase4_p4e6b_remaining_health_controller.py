"""Qualification-only controller for the fixed remaining Phase-4 cases.

The case set is deliberately closed: S02, S03, and C01 only.  The controller
reuses the accepted S01 process/freeze/witness primitives but keeps distinct
case markers and case-specific policy evidence.
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from phase4_p4e6b_hash_pinned_freeze import (
    commit_pinned_freeze, sha256_file,
)
from phase4_p4e6b_s01_attempt4_controller import (
    CONTROLLER as S01_CONTROLLER,
    FREEZE, LIVE_EVIDENCE, MATRIX_INSTALLED, MATRIX_SOURCE, RELAY, RUNNER, V2,
    WITNESS, LiveRosFactory, _durable, actual_identities,
)
from parking_robot_bringup.phase4_p4e6b_live_evidence import (
    authoritative_global_zero, load_genuine_witness_outcome, post_zero_angular_zero,
)
from parking_robot_bringup.phase4_p4e6b_remaining_evidence import (
    load_genuine_remaining_case_evidence,
)
from parking_robot_bringup.phase4_p4e6b_terminal_closure import (
    adjudicate_terminal_v2, physical_closure_from_rows,
)


CASE_CONFIG = {
    "S02": {"case_id": "B-S02", "expected_reason": "ODOMETRY_STALE", "attempt_number": 1},
    "S03": {"case_id": "B-S03", "expected_reason": "TF_STALE", "attempt_number": 1},
    "C01": {"case_id": "B-C01", "expected_reason": "COMMAND_PAIR_STALE", "attempt_number": 1},
}
AUTHORITY_TOKENS = {name: f"SUPERVISOR_AUTHORIZED_{name}_ATTEMPT1" for name in CASE_CONFIG}
REQUIRED_DIRECT_IDENTITIES = (
    "remaining_controller", "s01_base_controller", "remaining_evidence",
    "base_live_evidence", "terminal_v2", "runner", "matrix",
    "freeze_controller", "witness", "feedback_relay", "observation_relay",
    "tf_observation_relay", "feedback_relay_installed",
    "observation_relay_installed", "tf_observation_relay_installed",
)
CONTROLLER = Path(__file__).resolve()
REMAINING_EVIDENCE = CONTROLLER.parent.parent / "parking_robot_bringup" / "phase4_p4e6b_remaining_evidence.py"
OBSERVATION_RELAY = CONTROLLER.parent.parent / "parking_robot_bringup" / "phase4_p4e6b_observation_relay.py"
TF_OBSERVATION_RELAY = CONTROLLER.parent.parent / "parking_robot_bringup" / "phase4_p4e6b_tf_observation_relay.py"
WORKSPACE = CONTROLLER.parents[3]
FEEDBACK_RELAY_INSTALLED = WORKSPACE / "install/parking_robot_bringup/lib/parking_robot_bringup/phase4_p4e6b_feedback_relay"
OBSERVATION_RELAY_INSTALLED = WORKSPACE / "install/parking_robot_bringup/lib/parking_robot_bringup/phase4_p4e6b_observation_relay"
TF_OBSERVATION_RELAY_INSTALLED = WORKSPACE / "install/parking_robot_bringup/lib/parking_robot_bringup/phase4_p4e6b_tf_observation_relay"


def case_config(case_name: str) -> dict:
    try:
        return dict(CASE_CONFIG[str(case_name).upper()])
    except KeyError as exc:
        raise RuntimeError("REMAINING_HEALTH_CASE_DENIED") from exc


class RemainingLiveRosFactory(LiveRosFactory):
    def __init__(self, output_dir: Path, *, case_name: str, domain: int,
                 token: str, episode_uuid: str, **kwargs):
        self.case_name = str(case_name).upper()
        self.config = case_config(self.case_name)
        super().__init__(output_dir, domain=domain, token=token,
                         episode_uuid=episode_uuid, **kwargs)

    def command(self, role: str) -> list[str]:
        if role == "witness":
            return [sys.executable, str(self.witness_path), "--output-dir",
                    str(self.output / "terminal_witness"), "--reason",
                    self.config["expected_reason"]]
        if role == "matrix":
            if not self.ros2:
                raise RuntimeError("ROS2_EXECUTABLE_NOT_FOUND")
            return [str(Path(self.ros2).resolve()), "launch", "parking_robot_bringup",
                    "phase4_p4e6b_health_matrix.launch.py", "enable_health_runner:=false",
                    f"case_id:={self.config['case_id']}"]
        if role == "runner":
            evidence = self.output / "runner"
            code = ("from pathlib import Path; "
                    "import parking_robot_bringup.phase4_p4e6b_health_failure_runner as m; "
                    f"m.execute_campaign({self.config['case_id']!r}, Path({str(evidence)!r}), [])")
            return [sys.executable, "-c", code]
        raise ValueError(role)

    def runner_evidence(self):
        return load_genuine_remaining_case_evidence(self.output / "runner", self.case_name)

    def cleanup_global_zero(self):
        had_owned_processes = bool(self.processes)
        owned_pgids = {int(process.pgid) for process in self.processes.values()
                       if getattr(process, "pgid", None) is not None}
        self.terminate_owned()
        initial = authoritative_global_zero()
        result = initial
        settle_observation_used = False
        if had_owned_processes and not initial.get("pass", False):
            first = initial.get("scan1") or []
            second = initial.get("scan2") or []
            first_is_owned_only = all(
                int(row.get("pgid", -1)) in owned_pgids for row in first
            )
            if first_is_owned_only and not second:
                settle_observation_used = True
                result = authoritative_global_zero()
        return {"scan1": result.get("scan1"), "scan2": result.get("scan2"),
                "wait_sec": result.get("wait_sec"), "graph": result.get("graph"),
                "graph_clean": bool(result.get("graph", {}).get("clean")),
                "pass": bool(result.get("pass")), "initial": initial,
                "settle_observation_used": settle_observation_used}


class RemainingHealthController:
    """Single fixed S02/S03/C01 lifecycle; no arbitrary case framework."""
    def __init__(self, output_dir: Path, *, case_name: str,
                 authority_path: Path, controller_path: Path = CONTROLLER):
        self.output = Path(output_dir)
        self.output.mkdir(parents=True, exist_ok=True)
        self.case_name = str(case_name).upper()
        self.config = case_config(self.case_name)
        self.authority_path = Path(authority_path)
        self.controller_path = Path(controller_path)
        self.events: list[dict] = []

    def _marker_name(self) -> str:
        return f"{self.case_name}_ATTEMPT1_RUNTIME_STARTED"

    def _artifact(self, suffix: str) -> Path:
        return self.output / f"{self.case_name}_ATTEMPT1_{suffix}"

    def _event(self, name: str, **extra) -> dict:
        row = {"event": name, "monotonic_ns": time.monotonic_ns(), **extra}
        self.events.append(row)
        return row

    def expected_token(self) -> str:
        return AUTHORITY_TOKENS[self.case_name]

    def actual_identities(self) -> dict:
        identities = actual_identities(controller_path=self.controller_path)
        return {
            "remaining_controller": sha256_file(self.controller_path),
            "s01_base_controller": sha256_file(S01_CONTROLLER),
            "remaining_evidence": sha256_file(REMAINING_EVIDENCE),
            "base_live_evidence": sha256_file(LIVE_EVIDENCE),
            "terminal_v2": identities["terminal_v2"],
            "runner": identities["runner"],
            "matrix": identities["matrix"],
            "freeze_controller": identities["freeze_controller"],
            "witness": identities["witness"],
            "feedback_relay": identities["relay"],
            "observation_relay": sha256_file(OBSERVATION_RELAY),
            "tf_observation_relay": sha256_file(TF_OBSERVATION_RELAY),
            "feedback_relay_installed": sha256_file(FEEDBACK_RELAY_INSTALLED),
            "observation_relay_installed": sha256_file(OBSERVATION_RELAY_INSTALLED),
            "tf_observation_relay_installed": sha256_file(TF_OBSERVATION_RELAY_INSTALLED),
        }

    def admission(self, *, mode: str, supervisor_authority: str | None,
                  stop_before_marker: bool = False) -> dict:
        mode = str(mode).upper()
        if mode == "LIVE":
            if stop_before_marker:
                raise RuntimeError("REMAINING_NONATTEMPT_LIVE_DENIED")
            if supervisor_authority != self.expected_token():
                raise RuntimeError("LIVE_MODE_AUTHORITY_DENIED")
        elif mode != "DRY":
            raise RuntimeError("REMAINING_MODE_DENIED")
        document = json.loads(self.authority_path.read_text(encoding="utf-8"))
        expected = document.get("required_pinned_identity")
        if not isinstance(expected, dict):
            raise RuntimeError("EXPECTED_IDENTITY_AUTHORITY_INVALID")
        actual = self.actual_identities()
        missing = [key for key in REQUIRED_DIRECT_IDENTITIES
                   if key not in expected or key not in actual]
        mismatches = [key for key in REQUIRED_DIRECT_IDENTITIES
                      if key in expected and key in actual and actual[key] != expected[key]]
        if missing or mismatches:
            raise RuntimeError("FINAL_FREEZE_IDENTITY_MISMATCH")
        return {"case_name": self.case_name, "case_id": self.config["case_id"],
                "expected_reason": self.config["expected_reason"],
                "attempt_number": 1, "expected": expected, "actual": actual,
                "identity_match": True, "mode": mode}

    def command_contract(self, factory: RemainingLiveRosFactory) -> dict:
        commands = {role: factory.command(role) for role in ("witness", "matrix", "runner")}
        runner = commands["runner"]
        if "Path(" not in " ".join(runner):
            raise RuntimeError("RUNNER_PATH_INTERFACE_FAILURE")
        return {"case": self.case_name, "case_id": self.config["case_id"],
                "expected_reason": self.config["expected_reason"], "commands": commands,
                "environment": {"ROS_DOMAIN_ID": factory.env["ROS_DOMAIN_ID"],
                                 "P4E6B_EPISODE_TOKEN": factory.env["P4E6B_EPISODE_TOKEN"],
                                 "P4E6B_EPISODE_UUID": factory.env["P4E6B_EPISODE_UUID"]}}

    def execute_dry(self, *, supervisor_authority: str | None = None) -> dict:
        admission = self.admission(mode="DRY", supervisor_authority=supervisor_authority)
        return {"pass": True, "admission": admission,
                "marker_name": self._marker_name(),
                "pre_runtime_failure_name": self._artifact("PRE_RUNTIME_FAILURE.json").name,
                "final_result_name": self._artifact("FINAL_RESULT.json").name,
                "processes_created": 0, "attempt_marker_written": False}

    def _write_marker(self, matrix, freeze: dict, attempt_identity: str) -> None:
        _durable(self.output / self._marker_name(), {
            "event": self._marker_name(), "attempt_identity": attempt_identity,
            "case_name": self.case_name, "case_id": self.config["case_id"],
            "matrix_pid": matrix.pid, "matrix_pgid": getattr(matrix, "pgid", None),
            "matrix_sid": getattr(matrix, "sid", None), "ros_domain": getattr(matrix, "ros_domain", None),
            "monotonic_ns": time.monotonic_ns(),
            "freeze_sha256": freeze["freeze_artifact_sha256"],
        })

    def execute_live(self, factory: RemainingLiveRosFactory, *,
                     supervisor_authority: str, attempt_identity: str) -> dict:
        if not attempt_identity:
            raise RuntimeError("LIVE_ATTEMPT_IDENTITY_REQUIRED")
        try:
            admission = self.admission(mode="LIVE", supervisor_authority=supervisor_authority)
            preflight = factory.cleanup_global_zero()
            if not preflight.get("pass", False) or not preflight.get("graph_clean", False):
                raise RuntimeError("GLOBAL_ZERO_PREFLIGHT_FAILURE")
            self._event("GLOBAL_ZERO_PREFLIGHT_PASS", result=preflight)
            expected, actual = admission["expected"], admission["actual"]
            freeze_expected = dict(expected)
            freeze_expected.update({
                "live_controller": expected["remaining_controller"],
                "s01_controller": expected["s01_base_controller"],
            })
            freeze_actual = dict(actual)
            freeze_actual.update({
                "live_controller": actual["remaining_controller"],
                "s01_controller": actual["s01_base_controller"],
                "relay": actual["feedback_relay"],
            })
            freeze_expected["relay"] = expected["feedback_relay"]
            head = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                                  text=True, check=True).stdout.strip()
            freeze = commit_pinned_freeze(self.output, freeze_expected, freeze_actual, head=head,
                                          dirty_summary={"dirty": True})
            self._event("FINAL_FREEZE_COMMITTED", **freeze)
            witness = factory.spawn("witness")
            factory.witness_ready()
            self._event("TERMINAL_WITNESS_READY")
            matrix = factory.spawn("matrix")
            factory.matrix_alive(matrix)
            self._event("MATRIX_ALIVE")
            self._write_marker(matrix, freeze, attempt_identity)
            self._event(self._marker_name())
            runner = factory.spawn("runner")
            wait = factory.wait_runner(timeout=45)
            evidence = factory.runner_evidence()
            if evidence.get("expected_reason") != self.config["expected_reason"]:
                raise RuntimeError("REMAINING_CASE_REASON_MISMATCH")
            witness_outcome = factory.wait_witness_outcome(timeout=5)
            terminal = witness_outcome["terminal"]
            v2 = adjudicate_terminal_v2(
                origin_ns=terminal["origin_ns"], reason=self.config["expected_reason"],
                mission_id=terminal["mission_id"], route_id=terminal["route_id"],
                waypoint=terminal["waypoint"], uuid=terminal["uuid"],
                states=witness_outcome["states"], statuses=witness_outcome["statuses"],
                drain_start_ns=witness_outcome["drain_start_ns"],
                drain_end_ns=witness_outcome["drain_end_ns"], source_result_canceled=False)
            physical = physical_closure_from_rows(
                vehicle_rows=witness_outcome["vehicle_rows"], applied_rows=witness_outcome["applied_rows"],
                odometry_rows=witness_outcome["odometry_rows"], terminal_ns=terminal["origin_ns"],
                drain_end_ns=witness_outcome["drain_end_ns"])
            witness_outcome["post_zero_angular_zero"] = post_zero_angular_zero(
                witness_outcome, physical.get("anchor_ns", terminal["origin_ns"]))
            cleanup = factory.cleanup_global_zero()
            passed = (wait["returncode"] in (0, 1) and evidence.get("cancel_count") == 1 and
                      evidence.get("health_cancel_ack") == "HEALTH_CANCEL_ACK_ACCEPTED" and
                      evidence.get("originating_reason") == self.config["expected_reason"] and
                      v2.get("pass") is True and physical.get("pass") is True and
                      witness_outcome["post_zero_angular_zero"] is True and
                      cleanup.get("pass") is True and cleanup.get("graph_clean") is True)
            result = {"final_result": "CONSUMED_PASS" if passed else "CONSUMED_FAILED",
                      "attempt_consumed": True, self._marker_name(): True,
                      "attempt_identity": attempt_identity, "runner_wait": wait,
                      "runner_evidence": evidence, "witness_outcome": witness_outcome,
                      "v2_result": v2, "physical_result": physical, "cleanup": cleanup,
                      "events": self.events}
        except Exception as exc:
            marker_present = (self.output / self._marker_name()).is_file()
            cleanup = None
            try:
                cleanup = factory.cleanup_global_zero()
            except Exception:
                cleanup = {"cleanup_failed": True}
            if marker_present:
                result = {"final_result": "CONSUMED_FAILED", "attempt_consumed": True,
                          self._marker_name(): True, "attempt_identity": attempt_identity,
                          "classification": type(exc).__name__, "error": str(exc),
                          "cleanup": cleanup, "events": self.events}
                _durable(self._artifact("FINAL_RESULT.json"), result)
            else:
                result = {"classification": "UNCONSUMED_PRE_RUNTIME_FAILURE",
                          "attempt_consumed": False, self._marker_name(): False,
                          "attempt_identity": attempt_identity,
                          "error": str(exc), "cleanup": cleanup, "events": self.events}
                _durable(self._artifact("PRE_RUNTIME_FAILURE.json"), result)
                return result
        _durable(self._artifact("FINAL_RESULT.json"), result)
        return result


__all__ = ["CASE_CONFIG", "AUTHORITY_TOKENS", "RemainingHealthController",
           "RemainingLiveRosFactory", "case_config"]
