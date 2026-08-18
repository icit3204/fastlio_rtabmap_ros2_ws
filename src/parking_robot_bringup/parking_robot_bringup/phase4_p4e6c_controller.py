"""Bounded, fail-closed P4-E.6C qualification controller.

The controller owns orchestration policy only.  Production Mission Manager
progress semantics remain in MissionProgressSupervisorCore.  Effectful hooks
are injected by the eventual supervisor-authorized launch; this module never
creates a campaign merely by being imported.
"""
from __future__ import annotations

import json
import hashlib
import os
from pathlib import Path
from typing import Any, Callable, Mapping

from .phase4_p4e6c_progress_runner import case_config


class ProgressQualificationController:
    def __init__(self, case: str, output: Path, *, supervisor_authority: str | None = None,
                 attempt_number: int = 1):
        self.case = str(case).upper()
        self.config = case_config(self.case)
        self.output = Path(output)
        self.supervisor_authority = supervisor_authority
        self.attempt_number = int(attempt_number)

    @property
    def marker_name(self):
        if self.case == "C-P01" and self.attempt_number == 2:
            return "CP01_ATTEMPT2_RUNTIME_STARTED"
        return self.config["marker"]

    def _attempt_artifact(self, suffix: str) -> Path:
        return self.output / f"{self.marker_name.removesuffix('_RUNTIME_STARTED')}_{suffix}"

    @property
    def expected_authority(self) -> str:
        if self.supervisor_authority:
            return self.supervisor_authority
        return f"SUPERVISOR_AUTHORIZED_{self.case.replace('-', '')}_ATTEMPT1"

    def admission(self, *, expected_identities: dict, actual_identities: dict) -> None:
        for key, expected in expected_identities.items():
            if key not in actual_identities or actual_identities[key] != expected:
                raise RuntimeError(f"P4E6C_IDENTITY_DENIED:{key}")

    def execute_dry(self, *, expected_identities: dict, actual_identities: dict) -> dict:
        self.admission(expected_identities=expected_identities, actual_identities=actual_identities)
        if self.marker_name in {"CP01_ATTEMPT1_RUNTIME_STARTED", "CP02_ATTEMPT1_RUNTIME_STARTED"}:
            return {"case": self.case, "marker": self.marker_name, "attempt_consumed": False,
                    "progress_injections": 0, "recovery_injections": 0,
                    "route_mission_started": 0, "gate_armed": 0}
        raise RuntimeError("P4E6C_MARKER_DENIED")

    @staticmethod
    def _call(hooks: Mapping[str, Callable], name: str, *args, **kwargs):
        callback = hooks.get(name)
        if callback is None:
            raise RuntimeError(f"P4E6C_LIVE_HOOK_MISSING:{name}")
        return callback(*args, **kwargs)

    @staticmethod
    def _preflight_ok(result: Mapping[str, Any]) -> bool:
        graph = result.get("graph", {}) or {}
        return (result.get("scan1") == [] and result.get("scan2") == []
                and int(graph.get("returncode", 1)) == 0
                and bool(graph.get("clean")) and bool(result.get("pass")))

    def _write_json(self, path: Path, value: Mapping[str, Any]) -> None:
        self.output.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".tmp")
        payload = (json.dumps(dict(value), indent=2, sort_keys=True) + "\n").encode("utf-8")
        fd = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(str(temporary), str(path))
            directory_fd = os.open(str(self.output), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except Exception:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
            raise

    def _commit_marker(self, attempt_identity: str) -> Path:
        marker = self.output / self.marker_name
        if marker.exists():
            raise RuntimeError("P4E6C_ATTEMPT_ALREADY_CONSUMED")
        self._write_json(marker, {"attempt_identity": attempt_identity,
                                  "case": self.case, "marker": self.marker_name,
                                  "attempt_consumed": True})
        return marker

    def execute_live(self, *, attempt_identity: str, supervisor_authority: str,
                     expected_identities: Mapping[str, str],
                     actual_identities: Mapping[str, str],
                     hooks: Mapping[str, Callable]) -> dict:
        """Run the complete lifecycle through injected, owned runtime hooks.

        The hook boundary is deliberate: tests can exercise every consumption
        edge without launching ROS.  A real supervisor supplies the callbacks
        for the already-qualified launch, factory, runner, witness, and
        adjudicators.  Fault stimulation is called only after the durable
        marker is committed.
        """
        if supervisor_authority != self.expected_authority:
            raise RuntimeError("P4E6C_LIVE_AUTHORITY_DENIED")
        if not attempt_identity:
            raise RuntimeError("P4E6C_ATTEMPT_IDENTITY_REQUIRED")
        self.admission(expected_identities=dict(expected_identities), actual_identities=dict(actual_identities))
        if self.marker_name in {"CP01_ATTEMPT1_RUNTIME_STARTED", "CP02_ATTEMPT1_RUNTIME_STARTED"} and (self.output / self.marker_name).exists():
            raise RuntimeError("P4E6C_ATTEMPT_ALREADY_CONSUMED")
        owned = None
        marker_present = False
        preflight = None
        try:
            preflight = self._call(hooks, "authoritative_global_zero")
            if not self._preflight_ok(preflight):
                raise RuntimeError("GLOBAL_ZERO_PREFLIGHT_FAILURE")
            if not bool(self._call(hooks, "environment_valid")):
                raise RuntimeError("P4E6C_ENVIRONMENT_DENIED")
            owned = self._call(hooks, "launch")
            if not bool(self._call(hooks, "processes_alive", owned)):
                raise RuntimeError("P4E6C_PROCESS_START_FAILURE")
            self._call(hooks, "campaign_boot_ready", owned)
            self._call(hooks, "route_mission_start", owned)
            self._call(hooks, "active_identity_ready", owned)
            self._call(hooks, "gate_ready", owned)
            self._call(hooks, "pre_arm_command_ready", owned)
            self._call(hooks, "arm_gate", owned)
            self._call(hooks, "active_case_ready", owned)
            self._call(hooks, "witness_ready", owned)
            self._call(hooks, "runner_ready", owned)
            self._call(hooks, "injection_ready", owned)
            self._commit_marker(attempt_identity)
            marker_present = True
            # Stimulation is intentionally after consumption.
            runtime = self._call(hooks, "run_after_marker", owned)
            terminal = self._call(hooks, "terminal_adjudication", runtime)
            physical = self._call(hooks, "physical_adjudication", runtime)
            cleanup = self._call(hooks, "cleanup", owned)
            witness = self._call(hooks, "witness_seal", runtime, terminal, physical, cleanup)
            actual_reason = runtime.get("actual_reason")
            passed = (actual_reason == self.config["reason"]
                      and bool(terminal.get("pass"))
                      and bool(physical.get("pass")) and bool(cleanup.get("pass"))
                      and bool(witness.get("pass"))
                      and not witness.get("missing_files"))
            result = {"attempt_identity": attempt_identity, "marker_present": True,
                      "attempt_consumed": True, "case": self.case,
                      "expected_reason": self.config["reason"],
                      "actual_reason": actual_reason,
                      "terminal": terminal, "physical": physical, "cleanup": cleanup,
                      "witness": witness,
                      "final_classification": "CONSUMED_PASS" if passed else "CONSUMED_FAILED"}
            self._write_json(self._attempt_artifact("FINAL_RESULT.json"), result)
            return result
        except Exception as exc:
            # A bound launch hook may have created and registered its process
            # group before failing while recording ownership evidence. Recover
            # that ownership object before entering the normal cleanup path.
            if owned is None:
                launch_binding = hooks.get("launch")
                binding = getattr(launch_binding, "__self__", None)
                candidate = getattr(binding, "process", None)
                if candidate is not None:
                    owned = candidate
            if owned is not None:
                try:
                    cleanup = self._call(hooks, "cleanup", owned)
                except Exception as cleanup_exc:
                    cleanup = {"pass": False, "classification": "CLEANUP_FAILURE", "error": str(cleanup_exc)}
            else:
                cleanup = {"pass": False, "classification": "NO_PROCESS_SET"}
            if not marker_present:
                try:
                    failure = {"classification": "UNCONSUMED_PRE_RUNTIME_FAILURE", "reason": str(exc),
                               "cleanup": cleanup}
                    if isinstance(preflight, Mapping) and not self._preflight_ok(preflight):
                        graph = preflight.get("graph", {}) or {}
                        if preflight.get("scan1") or preflight.get("scan2"):
                            detail = "GLOBAL_TARGET_PROCESS_RESIDUE"
                        elif graph.get("returncode") != 0:
                            detail = "DAEMONLESS_GRAPH_QUERY_FAILURE"
                        elif graph.get("target_nodes"):
                            detail = "DAEMONLESS_GRAPH_TARGET_RESIDUE"
                        else:
                            detail = "DAEMONLESS_GRAPH_QUERY_FAILURE"
                        failure.update({
                            "reason": "GLOBAL_ZERO_PREFLIGHT_FAILURE",
                            "global_zero_preflight": preflight,
                            "failure_class_detail": detail,
                        })
                        evidence_path = self.output / "global_zero_preflight.json"
                        if evidence_path.is_file():
                            failure["global_zero_preflight_path"] = str(evidence_path)
                            digest = hashlib.sha256(evidence_path.read_bytes()).hexdigest()
                            failure["global_zero_preflight_sha256"] = digest
                    self._write_json(self._attempt_artifact("PRE_RUNTIME_FAILURE.json"), failure)
                except Exception:
                    pass
                raise RuntimeError("UNCONSUMED_PRE_RUNTIME_FAILURE") from exc
            result = {"attempt_identity": attempt_identity, "marker_present": True,
                      "attempt_consumed": True, "case": self.case,
                      "expected_reason": self.config["reason"],
                      "final_classification": "CONSUMED_FAILED", "error": str(exc),
                      "cleanup": cleanup}
            try:
                result["witness"] = self._call(hooks, "witness_seal", {}, {}, {}, cleanup)
            except Exception as witness_exc:
                result["witness"] = {"pass": False, "error": str(witness_exc)}
            self._write_json(self._attempt_artifact("FINAL_RESULT.json"), result)
            return result

    def write_pre_runtime_failure(self, reason: str) -> Path:
        path = self._attempt_artifact("PRE_RUNTIME_FAILURE.json")
        self._write_json(path, {"classification": "UNCONSUMED_PRE_RUNTIME_FAILURE", "reason": reason})
        return path
