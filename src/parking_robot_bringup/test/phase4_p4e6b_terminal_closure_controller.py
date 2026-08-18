"""Qualification-only controller lifetime integration.

The controller never owns application ROS control.  It waits for a genuine
witness commit after a runner exits and only then permits teardown.
"""

from __future__ import annotations

import json
import hashlib
import os
import subprocess
import time
from pathlib import Path


def wait_for_witness_commit(witness_dir: Path, *, timeout_sec: float = 2.0) -> dict:
    deadline = time.monotonic() + timeout_sec
    marker = Path(witness_dir) / "TERMINAL_WITNESS_OUTCOME_COMMITTED"
    while time.monotonic() < deadline:
        if marker.is_file():
            return json.loads(marker.read_text(encoding="utf-8"))
        time.sleep(0.005)
    raise TimeoutError("terminal witness commit not observed")


def classify_runner_exit(*, returncode: int, terminal_origin_seen: bool,
                         witness_dir: Path, timeout_sec: float = 2.0) -> dict:
    if returncode != 0 and not terminal_origin_seen:
        return {"classification": "RUNNER_FAILURE_BEFORE_TERMINAL", "teardown_allowed": True}
    evidence = wait_for_witness_commit(witness_dir, timeout_sec=timeout_sec)
    return {"classification": "WITNESS_CLOSURE_COMMITTED", "teardown_allowed": True,
            "witness": evidence}


def _durable_json(path: Path, value: dict) -> str:
    """Write, fsync, atomically rename, and read back a freeze record."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    payload = json.dumps(value, sort_keys=True, indent=2) + "\n"
    with tmp.open("w", encoding="utf-8") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)
    parent_fd = os.open(str(path.parent), os.O_DIRECTORY)
    try:
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)
    if path.read_text(encoding="utf-8") != payload:
        raise RuntimeError("FINAL_FREEZE_READBACK_MISMATCH")
    return hashlib.sha256(payload.encode()).hexdigest()


def commit_final_freeze(out: Path, identities: dict) -> dict:
    """Fail-closed pre-runtime freeze admission; no process creation occurs here."""
    required = {"HEAD", "runner", "matrix", "controller"}
    missing = sorted(required - set(identities))
    if missing or any(not identities.get(key) for key in required):
        raise RuntimeError("FINAL_FREEZE_MISSING_IDENTITY:" + ",".join(missing))
    record = {"identities": identities, "committed_before_matrix": True,
              "commit_monotonic_ns": time.monotonic_ns()}
    record["freeze_sha256"] = hashlib.sha256(
        json.dumps(record, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    freeze_path = Path(out) / "FINAL_FREEZE.json"
    freeze_sha = _durable_json(freeze_path, record)
    marker = Path(out) / "FINAL_FREEZE_COMMITTED"
    _durable_json(marker, {"event": "FINAL_FREEZE_COMMITTED",
                           "freeze_sha256": freeze_sha,
                           "monotonic_ns": time.monotonic_ns()})
    return {"freeze_path": str(freeze_path), "freeze_sha256": freeze_sha,
            "committed": True, "committed_ns": time.monotonic_ns()}


def assert_matrix_after_freeze(*, freeze_committed_ns: int,
                               matrix_created_ns: int,
                               runtime_started_ns: int) -> bool:
    if not (freeze_committed_ns < matrix_created_ns < runtime_started_ns):
        raise RuntimeError("FINAL_FREEZE_PROCESS_ORDER_INVALID")
    return True


def prospective_freeze_identities(controller_path: Path, runner_path: Path,
                                   matrix_path: Path) -> dict:
    """Collect the minimum identity set before any live matrix Popen."""
    head = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                          text=True, check=True).stdout.strip()
    return {
        "HEAD": head,
        "runner": hashlib.sha256(Path(runner_path).read_bytes()).hexdigest(),
        "matrix": hashlib.sha256(Path(matrix_path).read_bytes()).hexdigest(),
        "controller": hashlib.sha256(Path(controller_path).read_bytes()).hexdigest(),
    }
