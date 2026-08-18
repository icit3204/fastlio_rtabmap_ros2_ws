"""Actual runner no-route seam with worker-authored durable lifecycle phases."""
import argparse
import faulthandler
import hashlib
import json
import os
import signal
import sys
import threading
import time
import traceback
from pathlib import Path


def _phase(path, phase, **extra):
    """Append a worker-originated phase and fsync it before continuing."""
    row = {"phase": phase, "monotonic_ns": time.monotonic_ns(),
           "wall_time": time.time(), "pid": os.getpid(), "ppid": os.getppid(),
           "pgid": os.getpgrp(), "thread_id": threading.get_ident(),
           "episode_uuid": os.environ.get("P4E6B_EPISODE_UUID", ""),
           "episode_token": os.environ.get("P4E6B_EPISODE_TOKEN", ""), **extra}
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    return row


def _durable_json(path, value):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, sort_keys=True, indent=2) + "\n")
    with tmp.open("rb") as handle:
        os.fsync(handle.fileno())
    os.replace(tmp, path)
    fd = os.open(str(path.parent), os.O_DIRECTORY)
    os.fsync(fd)
    os.close(fd)


def _file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _commit_outcome(phases, output, outcome, artifact_name):
    """Publish the outcome barrier only after authoritative evidence is sealed."""
    artifact = output / artifact_name
    if not artifact.is_file():
        raise RuntimeError(f"missing outcome evidence: {artifact_name}")
    try:
        evidence = json.loads(artifact.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"unparseable outcome evidence: {artifact_name}") from exc
    if outcome == "READY":
        event = evidence.get("event", {})
        if event.get("event") != "PREMISSION_HEALTH_READY" or event.get("ready") is not True:
            raise RuntimeError("READY evidence self-consistency failure")
    artifact_sha256 = _file_sha256(artifact)
    return _phase(phases, "OUTCOME_EVIDENCE_COMMITTED", outcome=outcome,
                  artifact=artifact_name, artifact_sha256=artifact_sha256)


ENV_KEYS = ("PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH",
            "LD_LIBRARY_PATH", "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION",
            "ROS_LOCALHOST_ONLY", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "PYTHONHOME",
            "PYTHONNOUSERSITE", "P4E6B_EPISODE_TOKEN", "P4E6B_EPISODE_UUID")


def _environment_record():
    """A stable, selected environment snapshot for pre-import failures."""
    values = {key: os.environ.get(key) for key in ENV_KEYS}
    encoded = json.dumps(values, sort_keys=True, separators=(",", ":")).encode()
    return values, hashlib.sha256(encoded).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--case-id", default="B-H01")
    parser.add_argument("--diagnostic-qos", choices=("reliable", "best_effort"), default="best_effort")
    parser.add_argument("--bootstrap-only", action="store_true")
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    phases = output / "worker_phase.jsonl"
    stack_file = (output / "worker_faulthandler.txt").open("w", encoding="utf-8")
    faulthandler.enable(file=stack_file, all_threads=True)
    faulthandler.register(signal.SIGUSR1, file=stack_file, all_threads=True)
    _phase(phases, "PROCESS_START")
    node = None
    rclpy = None
    helper_enter = None
    outcome = None
    exit_code = 0
    try:
        _phase(phases, "ROS_IMPORT_BEGIN")
        try:
            import rclpy as imported_rclpy
            import importlib.metadata
            from rclpy.qos import ReliabilityPolicy
            from ament_index_python.packages import get_package_prefix
            from parking_robot_bringup.phase4_p4e6b_health_failure_runner import HealthRuntimeRunner
            rclpy = imported_rclpy
        except Exception as exc:
            values, fingerprint = _environment_record()
            now = time.monotonic_ns()
            _phase(phases, "ROS_IMPORT_EXCEPTION", exception_class=type(exc).__name__,
                   exception_message=str(exc), import_exception_monotonic_ns=now)
            _durable_json(output / "ros_import_exception.json", {
                "exception_class": type(exc).__name__, "exception_message": str(exc),
                "traceback": traceback.format_exc(), "monotonic_ns": now,
                "sys_executable": sys.executable, "sys_path": sys.path,
                "environment": values, "environment_fingerprint_sha256": fingerprint,
                "pid": os.getpid(), "ppid": os.getppid(), "pgid": os.getpgrp(),
                "episode_uuid": os.environ.get("P4E6B_EPISODE_UUID", ""),
                "episode_token": os.environ.get("P4E6B_EPISODE_TOKEN", ""),
            })
            outcome = "ROS_IMPORT_EXCEPTION"
            exit_code = 1
            # Preserve the durable import evidence, run finally, and retain a
            # non-zero process result.  A plain return would incorrectly turn
            # this pre-import failure into exit status zero.
            raise SystemExit(exit_code)
        values, fingerprint = _environment_record()
        _phase(phases, "ROS_IMPORT_END", sys_executable=sys.executable,
               environment_fingerprint_sha256=fingerprint)
        _durable_json(output / "ros_import_success.json", {
            "sys_executable": sys.executable, "sys_path": sys.path,
            "rclpy_module_path": rclpy.__file__,
            "ros2cli_version": importlib.metadata.version("ros2cli"),
            "parking_robot_bringup_prefix": get_package_prefix("parking_robot_bringup"),
            "parking_robot_interfaces_prefix": get_package_prefix("parking_robot_interfaces"),
            "environment": values, "environment_fingerprint_sha256": fingerprint,
        })
        if args.bootstrap_only:
            outcome = "BOOTSTRAP_ONLY"
            return
        _phase(phases, "RCLPY_INIT_BEGIN")
        rclpy.init()
        _phase(phases, "RCLPY_INIT_END")
        _phase(phases, "RUNNER_CONSTRUCT_BEGIN")
        node = HealthRuntimeRunner(
            output, args.case_id,
            premission_diagnostic_reliability=(ReliabilityPolicy.RELIABLE if args.diagnostic_qos == "reliable"
                                               else ReliabilityPolicy.BEST_EFFORT),
            premission_diagnostic_depth=1)
        _phase(phases, "RUNNER_CONSTRUCT_END")
        _phase(phases, "PUBLISH_INITIAL_BEGIN")
        node.publish_initial()
        _phase(phases, "PUBLISH_INITIAL_END")
        _phase(phases, "INITIAL_SPIN_BEGIN")
        node.spin(.5)
        _phase(phases, "INITIAL_SPIN_END")
        # Direct deadline origin: fsync marker, then immediately call the real helper.
        helper_enter = _phase(phases, "HELPER_ENTER")
        row = node.wait_active_premission_health()
        returned = _phase(phases, "HELPER_RETURN_READY",
                          helper_elapsed_ns=time.monotonic_ns() - helper_enter["monotonic_ns"])
        ready = {"event": row, "helper_enter_monotonic_ns": helper_enter["monotonic_ns"],
                 "helper_elapsed_ns": returned["helper_elapsed_ns"], "diagnostic_qos": args.diagnostic_qos,
                 "route_mission": 0, "mission_start": 0, "uuid": 0, "gate_arm": 0,
                 "health_injection": 0}
        _durable_json(output / "premission_seam.json", ready)
        _durable_json(output / "ready_record_sha256.json", {
            "sha256": hashlib.sha256(json.dumps(row, sort_keys=True, separators=(",", ":")).encode()).hexdigest()})
        _commit_outcome(phases, output, "READY", "premission_seam.json")
        outcome = "READY"
    except Exception as exc:
        now = time.monotonic_ns()
        _phase(phases, "HELPER_EXCEPTION", exception_class=type(exc).__name__, exception_message=str(exc),
               helper_elapsed_ns=None if helper_enter is None else now - helper_enter["monotonic_ns"])
        _durable_json(output / "helper_exception.json", {
            "exception_class": type(exc).__name__, "exception_message": str(exc),
            "traceback": traceback.format_exc(), "helper_enter_monotonic_ns": None if helper_enter is None else helper_enter["monotonic_ns"],
            "exception_monotonic_ns": now,
            "helper_elapsed_sec": None if helper_enter is None else (now-helper_enter["monotonic_ns"])/1e9})
        _commit_outcome(phases, output, "TIMEOUT" if "PREMISSION_HEALTH_READINESS_TIMEOUT" in str(exc) else "EXCEPTION",
                        "helper_exception.json")
        outcome = "EXCEPTION"
        exit_code = 1
    finally:
        if node is not None:
            _phase(phases, "WRITER_FINALIZE_BEGIN")
            node.writer.finalize()
            _phase(phases, "WRITER_FINALIZE_END")
            _phase(phases, "RUNNER_CLOSE_BEGIN")
            node.close()
            _phase(phases, "RUNNER_CLOSE_END")
            _phase(phases, "DESTROY_NODE_BEGIN")
            node.destroy_node()
            _phase(phases, "DESTROY_NODE_END")
        if rclpy is not None and rclpy.ok():
            _phase(phases, "RCLPY_SHUTDOWN_BEGIN")
            rclpy.shutdown()
            _phase(phases, "RCLPY_SHUTDOWN_END")
        _phase(phases, "PROCESS_EXIT", outcome=outcome, exit_code=exit_code)
        stack_file.flush()
        os.fsync(stack_file.fileno())
        stack_file.close()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
