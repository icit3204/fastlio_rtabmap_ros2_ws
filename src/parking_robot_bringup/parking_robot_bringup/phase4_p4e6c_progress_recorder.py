"""Typed, fresh-row progress evidence boundary for P4-E.6C."""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

from .phase4_p4e6c_progress_runner import ProgressAdmission


READINESS_FIELDS = (
    "mission_active", "policy_active", "gate_armed", "collision_clear",
    "health_current", "odom_current", "tf_current", "feedback_current",
    "command_pair_current", "movement_intent", "action_terminal", "recovery_delta",
    "mission_id", "route_id", "active_goal_uuid",
)


@dataclass(frozen=True)
class Readiness:
    admission: ProgressAdmission
    mission_id: str
    route_id: str
    active_goal_uuid: str
    generation: str | None = None
    recovery_relay_ready: bool = False


def readiness_from_mapping(values: Mapping[str, Any], *, case: str,
                           require_active_case: bool = True) -> Readiness:
    missing = [field for field in READINESS_FIELDS if field not in values]
    if str(case).upper() == "C-P02" and "recovery_relay_ready" not in values:
        missing.append("recovery_relay_ready")
    if missing:
        raise RuntimeError("P4E6C_READINESS_FIELD_MISSING:" + ",".join(missing))
    if values["mission_active"] is not True or values["policy_active"] is not True:
        raise RuntimeError("P4E6C_READINESS_MISSION_NOT_ACTIVE")
    if require_active_case and values["gate_armed"] is not True:
        raise RuntimeError("P4E6C_READINESS_GATE_NOT_ARMED")
    if values["action_terminal"] is not False:
        raise RuntimeError("P4E6C_READINESS_ACTION_TERMINAL")
    for key in ("mission_id", "route_id", "active_goal_uuid"):
        if not str(values[key]):
            raise RuntimeError("P4E6C_READINESS_IDENTITY_MISSING:" + key)
    if int(values["recovery_delta"]) >= 6:
        raise RuntimeError("P4E6C_READINESS_RECOVERY_NONZERO")
    if str(case).upper() == "C-P02" and require_active_case:
        if values["recovery_relay_ready"] is not True or int(values["recovery_delta"]) != 0:
            raise RuntimeError("P4E6C_READINESS_RECOVERY_RELAY_NOT_READY")
    admission = ProgressAdmission(**{
        key: bool(values[key]) for key in (
            "mission_active", "policy_active", "gate_armed", "collision_clear",
            "health_current", "odom_current", "tf_current", "feedback_current",
            "command_pair_current", "movement_intent", "action_terminal")
    }, recovery_delta=int(values["recovery_delta"]))
    return Readiness(admission=admission, mission_id=str(values["mission_id"]),
                     route_id=str(values["route_id"]),
                     active_goal_uuid=str(values["active_goal_uuid"]),
                     generation=None if values.get("generation") is None else str(values["generation"]),
                     recovery_relay_ready=bool(values.get("recovery_relay_ready", False)))


class FreshEvidenceReader:
    """Consume each JSONL row once, requiring sequence/time monotonicity."""

    def __init__(self, path: Path, *, baseline_sequence: int = -1,
                 baseline_monotonic_ns: int = -1, transaction_id: str | None = None):
        self.path = Path(path)
        self._sequence = baseline_sequence
        self._monotonic_ns = baseline_monotonic_ns
        self.transaction_id = transaction_id

    def next(self, *, timeout_sec: float = 0.0) -> dict:
        import time
        deadline = time.monotonic() + timeout_sec
        if not self.path.is_file():
            raise RuntimeError("P4E6C_EVIDENCE_FILE_MISSING:" + self.path.name)
        while True:
            rows = [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines()
                    if line.strip()]
            for row in rows:
                if self.transaction_id is not None and row.get("transaction_id") != self.transaction_id:
                    continue
                sequence = int(row.get("sequence", -1))
                monotonic_ns = int(row.get("monotonic_ns", row.get("observation_monotonic_ns", -1)))
                if sequence > self._sequence or monotonic_ns > self._monotonic_ns:
                    self._sequence = sequence
                    self._monotonic_ns = monotonic_ns
                    return row
            if time.monotonic() >= deadline:
                break
            time.sleep(0.01)
        raise RuntimeError("P4E6C_STALE_EVIDENCE_ROW")


class ProgressEvidenceRecorder:
    """Persist genuine observation rows using the controller's durable writer."""

    COMMON_FILES = (
        "mission_policy_diagnostics.jsonl", "mission_state_events.jsonl",
        "progress_events.jsonl", "navigate_action_status_events.jsonl",
        "block_cancel_ack_events.jsonl", "pose_clamp_events.jsonl",
        "physical_evidence.jsonl", "cleanup_result.json", "feedback_receipts.jsonl",
        "route_mission_events.jsonl", "mission_service_events.jsonl",
        "terminal_drain_events.jsonl", "recovery_injection_events.jsonl",
        "clamp_warmup_events.jsonl",
    )

    def __init__(self, output: Path, case: str, durable_write):
        self.output = Path(output)
        self.case = str(case).upper()
        self.durable_write = durable_write
        self._sequence = {}

    def _append(self, filename: str, row: Mapping[str, Any]) -> dict:
        path = self.output / filename
        self.output.mkdir(parents=True, exist_ok=True)
        sequence = self._sequence.get(filename, 0)
        self._sequence[filename] = sequence + 1
        value = dict(row)
        value.setdefault("sequence", sequence)
        value.setdefault("monotonic_ns", time_monotonic_ns())
        with path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(value, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        return value

    def write_readiness(self, values: Mapping[str, Any], *, require_active_case: bool = True) -> Readiness:
        readiness = readiness_from_mapping(values, case=self.case,
                                           require_active_case=require_active_case)
        self.durable_write(self.output / "readiness.json", dict(values))
        return readiness

    def record(self, filename: str, row: Mapping[str, Any]) -> dict:
        allowed = set(self.COMMON_FILES) | {
            "recovery_feedback_events.jsonl", "process_group_events.jsonl",
            "gate_service_events.jsonl", "gate_state_events.jsonl",
            "service_endpoint_readiness.json", "readiness_candidate_events.jsonl",
            "clamp_acceptance_events.jsonl",
        }
        if filename not in allowed:
            raise RuntimeError("P4E6C_EVIDENCE_FILE_DENIED")
        return self._append(filename, row)

    def record_clamp(self, row: Mapping[str, Any]) -> dict:
        return self.record("pose_clamp_events.jsonl", row)

    def record_progress(self, row: Mapping[str, Any]) -> dict:
        return self.record("progress_events.jsonl", row)

    def record_recovery(self, row: Mapping[str, Any]) -> dict:
        if self.case != "C-P02":
            raise RuntimeError("P4E6C_RECOVERY_EVIDENCE_CASE_DENIED")
        return self.record("recovery_feedback_events.jsonl", row)

    def write_cleanup(self, result: Mapping[str, Any]) -> None:
        self.durable_write(self.output / "cleanup_result.json", dict(result))


def time_monotonic_ns() -> int:
    import time
    return time.monotonic_ns()
