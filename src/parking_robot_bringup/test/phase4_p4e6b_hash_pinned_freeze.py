"""Qualification-only hash-pinned, fail-closed pre-matrix admission."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import time
from pathlib import Path


REQUIRED_PINNED_IDENTITY = (
    "runner", "matrix", "live_controller", "freeze_controller",
    "witness", "relay",
)


def load_expected_authority(path: Path) -> dict:
    """Load supervisor-supplied expected identities; never derive them."""
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    expected = document.get("required_pinned_identity")
    if not isinstance(expected, dict):
        raise RuntimeError("EXPECTED_IDENTITY_AUTHORITY_INVALID")
    verify_expected_identities(expected, expected)
    return dict(expected)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_expected_identities(expected: dict, actual: dict) -> dict:
    missing = [key for key in REQUIRED_PINNED_IDENTITY if key not in expected or key not in actual]
    mismatches = {
        key: {"expected": expected.get(key), "actual": actual.get(key)}
        for key in REQUIRED_PINNED_IDENTITY
        if expected.get(key) != actual.get(key)
    }
    if missing or mismatches:
        raise RuntimeError("FINAL_FREEZE_IDENTITY_MISMATCH")
    return {"identity_match": True, "required_keys": list(REQUIRED_PINNED_IDENTITY),
            "missing": [], "mismatches": {}}


def _durable_json(path: Path, value: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, sort_keys=True, indent=2) + "\n"
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    parent_fd = os.open(str(path.parent), os.O_DIRECTORY)
    try:
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)
    if path.read_text(encoding="utf-8") != payload:
        raise RuntimeError("FINAL_FREEZE_READBACK_MISMATCH")
    return hashlib.sha256(payload.encode()).hexdigest()


def commit_pinned_freeze(out: Path, expected: dict, actual: dict,
                         *, head: str, dirty_summary: dict) -> dict:
    """Compare first; only matching identities may create durable freeze files."""
    match = verify_expected_identities(expected, actual)
    record = {
        "expected_identities": dict(expected),
        "actual_identities": dict(actual),
        "identity_match": match["identity_match"],
        "HEAD": head,
        "dirty_summary": dirty_summary,
        "freeze_commit_monotonic_ns": time.monotonic_ns(),
    }
    record["freeze_artifact_sha256"] = hashlib.sha256(
        json.dumps(record, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    freeze_sha = _durable_json(Path(out) / "FINAL_FREEZE.json", record)
    committed = {"event": "FINAL_FREEZE_COMMITTED", "freeze_artifact_sha256": freeze_sha,
                 "identity_match": True, "monotonic_ns": time.monotonic_ns()}
    commit_ns = time.monotonic_ns()
    _durable_json(Path(out) / "FINAL_FREEZE_COMMITTED", committed)
    return {"identity_match": True, "freeze_artifact_sha256": freeze_sha,
            "committed_ns": commit_ns, "marker": "FINAL_FREEZE_COMMITTED"}


def assert_process_order(freeze_committed_ns: int, matrix_created_ns: int,
                         runtime_started_ns: int) -> None:
    if not (freeze_committed_ns < matrix_created_ns < runtime_started_ns):
        raise RuntimeError("FINAL_FREEZE_PROCESS_ORDER_INVALID")


class RecordingPopen:
    calls = []

    def __init__(self, argv, **kwargs):
        self.argv = list(argv)
        self.kwargs = kwargs
        self.pid = len(type(self).calls) + 10000
        self.returncode = None
        type(self).calls.append(self)


def prospective_admission(out: Path, expected: dict, actual: dict,
                           *, head: str = "approved-head", dirty_summary=None,
                           popen=RecordingPopen) -> dict:
    """Dry admission seam: Popen is unreachable until the pinned freeze commits."""
    popen.calls = []
    freeze = commit_pinned_freeze(out, expected, actual, head=head,
                                  dirty_summary=dirty_summary or {"dirty": True})
    created_ns = time.monotonic_ns()
    process = popen(["matrix", "--qualification-dry-run"], start_new_session=True)
    runtime_marker_ns = time.monotonic_ns()
    assert_process_order(freeze["committed_ns"], created_ns, runtime_marker_ns)
    return {"freeze": freeze, "popen_call_count": len(popen.calls),
            "matrix_created_ns": created_ns, "runtime_started_marker_ns": runtime_marker_ns}
