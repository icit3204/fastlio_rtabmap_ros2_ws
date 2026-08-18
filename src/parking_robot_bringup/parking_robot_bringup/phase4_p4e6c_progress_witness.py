"""Immutable progress-failure witness transaction."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path


COMMON_REQUIRED_FILES = (
    "mission_policy_diagnostics.jsonl", "mission_state_events.jsonl",
    "progress_events.jsonl", "navigate_action_status_events.jsonl",
    "block_cancel_ack_events.jsonl", "pose_clamp_events.jsonl",
    "physical_evidence.jsonl", "cleanup_result.json",
)
CASE_REQUIRED_FILES = {
    "C-P01": COMMON_REQUIRED_FILES,
    "C-P02": COMMON_REQUIRED_FILES + ("recovery_feedback_events.jsonl",),
}


class ProgressWitness:
    """Readiness plus non-self-referential evidence sealing."""

    def __init__(self, output: Path, case: str = "C-P01"):
        self.output = Path(output)
        self.case = str(case).upper()
        if self.case not in CASE_REQUIRED_FILES:
            raise RuntimeError("P4E6C_WITNESS_CASE_DENIED")
        self.ready = False
        self.sealed = False

    def mark_ready(self, *, identity: dict, mechanism: dict) -> dict:
        if self.sealed:
            raise RuntimeError("P4E6C_WITNESS_ALREADY_SEALED")
        if not identity or not mechanism.get("ready", False):
            raise RuntimeError("P4E6C_WITNESS_NOT_READY")
        self.ready = True
        return {"event": "PROGRESS_WITNESS_READY", "identity": dict(identity),
                "mechanism": dict(mechanism)}

    def seal(self, *, identity: dict, outcome: dict) -> dict:
        if not self.ready or self.sealed:
            raise RuntimeError("P4E6C_WITNESS_SEAL_DENIED")
        required_files = list(CASE_REQUIRED_FILES[self.case])
        present_files = []
        missing_files = []
        hashes = {}
        for name in required_files:
            path = self.output / name
            if path.is_file() and path.stat().st_size > 0:
                present_files.append(name)
                hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
            else:
                missing_files.append(name)
        terminal_complete = bool(outcome.get("terminal_evidence_complete", False))
        physical_complete = bool(outcome.get("physical_evidence_complete", False))
        passed = not missing_files and terminal_complete and physical_complete
        result = {"identity": dict(identity), "case": self.case,
                  "required_files": required_files, "present_files": present_files,
                  "missing_files": missing_files, "file_hashes": hashes,
                  "terminal_evidence_complete": terminal_complete,
                  "physical_evidence_complete": physical_complete,
                  "outcome": dict(outcome), "pass": passed}
        if not passed:
            raise RuntimeError("P4E6C_WITNESS_SEAL_FAIL")
        marker = self.output / "PROGRESS_WITNESS_COMMITTED"
        marker.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.sealed = True
        return result
