"""Crash-accountable no-route real-helper readiness campaign harness (F3)."""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

from phase4_p4e6b_fresh30_harness import append, clean, durable_json, proc_token


ROOT = Path(__file__).resolve().parents[3]
FROZEN_RUNNER = ROOT / "src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6b_health_failure_runner.py"
FROZEN_RUNNER_SHA256 = "458d2a6e184079a1236d28664c4554292a9b4a2cef72e3c576f5c4b89d545b71"
SEAM = ROOT / "src/parking_robot_bringup/test/phase4_p4e6b_premission_seam.py"
MATRIX = [
    "ros2", "launch", "parking_robot_bringup", "phase4_p4e6b_health_matrix.launch.py",
    "enable_health_runner:=false",
]
OBSERVER = ROOT / "src/parking_robot_bringup/test/phase4_p4e6b_formal_lifecycle_observer.py"
HELPER_TIMEOUT_SEC = 8.0
CHILD_PYTHON = "/usr/bin/python3"
ENV_KEYS = ("PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH",
            "LD_LIBRARY_PATH", "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION",
            "ROS_LOCALHOST_ONLY", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "PYTHONHOME",
            "PYTHONNOUSERSITE", "P4E6B_EPISODE_TOKEN", "P4E6B_EPISODE_UUID")
TARGET_PROCESS_MARKERS = (
    "/nav2_lifecycle_manager/lifecycle_manager", "planner_server", "controller_server",
    "behavior_server", "bt_navigator", "waypoint_follower", "collision_monitor",
    "collision_monitor_validity_monitor", "parking_robot_mission_manager",
    "guarded_vehicle_cmd_gate", "phase4_vehicle_cmd_fake_base", "wheelchair_cmd_adapter",
    "phase4_p4e6b_health_failure_runner", "phase4_p4e6b_premission_seam",
)


def _open(path):
    return path.open("w", encoding="utf-8")


def _root_record(process):
    return {"pid": process.pid, "pgid": process.pid}


def environment_snapshot(env):
    """Full selected values plus a canonical fingerprint; never mutates env."""
    values = {key: env.get(key) for key in ENV_KEYS}
    encoded = json.dumps(values, sort_keys=True, separators=(",", ":")).encode()
    return {"values": values, "sha256": hashlib.sha256(encoded).hexdigest()}


def child_environment(parent, domain, token, episode_uuid):
    """Preserve the sourced parent and override only episode-specific fields."""
    env = parent.copy()
    env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
               P4E6B_EPISODE_TOKEN=token, P4E6B_EPISODE_UUID=episode_uuid,
               PYTHONUNBUFFERED="1")
    return env


def _probe_command():
    return [CHILD_PYTHON, "-c", "\n".join((
        "import importlib.metadata", "import rclpy, launch, launch_ros, ament_index_python",
        "from ament_index_python.packages import get_package_prefix",
        "assert importlib.metadata.version('ros2cli')",
        "assert get_package_prefix('parking_robot_bringup')",
        "assert get_package_prefix('parking_robot_interfaces')",
    ))]


def environment_admission(env, include_ros2=True):
    """Run the exact child interpreter/environment without launching a matrix."""
    result = subprocess.run(_probe_command(), env=env, capture_output=True, text=True)
    row = {"python_argv": _probe_command(), "returncode": result.returncode,
           "stdout": result.stdout, "stderr": result.stderr,
           "environment": environment_snapshot(env)}
    if include_ros2 and result.returncode == 0:
        ros2 = subprocess.run(["ros2", "pkg", "prefix", "parking_robot_bringup"], env=env,
                              capture_output=True, text=True)
        row["ros2_pkg_prefix"] = {"returncode": ros2.returncode,
                                   "stdout": ros2.stdout, "stderr": ros2.stderr}
        row["passed"] = ros2.returncode == 0
    else:
        row["passed"] = result.returncode == 0
    return row


def global_target_processes():
    """Read-only target scan; never used as a broad cleanup authority."""
    mine = {os.getpid(), os.getppid()}
    rows = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(proc.name)
            if pid in mine:
                continue
            command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            if not any(marker in command for marker in TARGET_PROCESS_MARKERS):
                continue
            stat = (proc / "stat").read_text().split()
            rows.append({"pid": pid, "ppid": int(stat[3]), "pgid": int(stat[4]),
                         "sid": int(stat[5]), "command": command})
        except (FileNotFoundError, PermissionError, IndexError, ValueError):
            pass
    return sorted(rows, key=lambda row: row["pid"])


def _valid_ready(seam):
    """Validate evidence emitted by the real runner, without recreating its model."""
    event = seam.get("event", {})
    def below(value, limit):
        return value is not None and value < limit

    generation = event.get("generation")
    required = (
        event.get("ready") is True,
        event.get("publisher_count") == 1,
        isinstance(generation, list) and len(generation) == 2,
        isinstance(generation, list) and generation[0] == "collision_monitor_validity_monitor",
        isinstance(generation, list) and bool(generation[1]),
        bool(event.get("diagnostic_writer_gid")),
        bool(event.get("epoch_start_ns")),
        bool(event.get("epoch_start_ros_ns")),
        event.get("bool_post_epoch_count", 0) >= 2,
        event.get("bool_value") is True,
        event.get("diagnostic_state") == "VALID",
        event.get("diagnostic_reason") == "VALID",
        (event.get("semantic_healthy_stable_sec") or 0.0) >= 1.0,
        below(event.get("bool_age_ns"), 250_000_000),
        below(event.get("diagnostic_age_ns"), 250_000_000),
        below(event.get("diagnostic_transport_age_ns"), 250_000_000),
        below(event.get("source_age_upper_bound_sec"), 0.5),
        below(seam.get("helper_elapsed_ns"), HELPER_TIMEOUT_SEC * 1e9),
    )
    return all(required)


def _record_campaign(root, value):
    append(root / "campaign_journal.jsonl", value)


def _phases(path):
    if not path.exists():
        return []
    try:
        return [json.loads(line) for line in path.read_text().splitlines() if line]
    except json.JSONDecodeError:
        return []


def _campaign_rows(root):
    return _phases(Path(root) / "campaign_journal.jsonl")


def _unfinished_episode(rows):
    """A committed episode is never eligible for an implicit replacement."""
    committed = {row.get("episode_index") for row in rows if row.get("state") in
                 ("MATRIX_COMMITTED", "READINESS_COMMITTED")}
    sealed = {row.get("episode_index") for row in rows if row.get("state") == "SEALED"}
    return sorted(index for index in committed - sealed if index is not None)


def pre_episode_admission(root, identity, parent_environment, child_environment_record, parent_admission, child_admission):
    """The sole fail-closed gate immediately preceding every matrix Popen."""
    root = Path(root)
    if hashlib.sha256(FROZEN_RUNNER.read_bytes()).hexdigest() != FROZEN_RUNNER_SHA256:
        return {"passed": False, "reason": "FROZEN_RUNNER_SHA_MISMATCH"}
    rows = _campaign_rows(root)
    unfinished = _unfinished_episode(rows)
    previous = identity["episode_index"] - 1
    previous_sealed = previous <= 0 or any(row.get("state") == "SEALED" and
                                            row.get("episode_index") == previous for row in rows)
    first = global_target_processes()
    time.sleep(2.0)
    second = global_target_processes()
    passed = (not unfinished and previous_sealed and not first and not second and
              parent_admission["passed"] and child_admission["passed"])
    return {"passed": passed, "reason": None if passed else "PRE_EPISODE_ADMISSION_REJECTED",
            "runner_sha256": FROZEN_RUNNER_SHA256, "unfinished_episodes": unfinished,
            "previous_episode": previous, "previous_sealed": previous_sealed,
            "global_first": first, "global_second": second,
            "parent_env_sha256": parent_environment["sha256"],
            "child_env_sha256": child_environment_record["sha256"]}


def lifecycle_accounting(phases):
    """Pure controller accounting from worker-authored phases."""
    names = [row.get("phase") for row in phases]
    if names.count("HELPER_ENTER") > 1:
        return {"state": "DUPLICATE_HELPER_ENTER", "committed": False, "outcome": None}
    if "HELPER_ENTER" not in names:
        return {"state": "PRE_HELPER", "committed": False, "outcome": None}
    if "HELPER_RETURN_READY" in names:
        outcome = "READY"
    elif "HELPER_EXCEPTION" in names:
        outcome = "EXCEPTION"
    else:
        outcome = None
    return {"state": "COMMITTED", "committed": True, "outcome": outcome,
            "process_exit": "PROCESS_EXIT" in names}


def run_one_readiness_episode(root, index, domain):
    root = Path(root)
    token = uuid.uuid4().hex
    episode_uuid = uuid.uuid4().hex
    episode = root / f"episode_{index:02d}"
    episode.mkdir()
    journal = episode / "state_journal.jsonl"
    identity = {
        "campaign_id": root.name,
        "episode_index": index,
        "episode_uuid": episode_uuid,
        "ros_domain_id": domain,
        "episode_token": token,
    }
    durable_json(episode / "identity.json", identity)
    append(journal, {"state": "PLANNED"})
    _record_campaign(root, {"state": "PLANNED", "episode_index": index, "episode_uuid": episode_uuid})

    parent_environment = environment_snapshot(os.environ)
    env = child_environment(os.environ, domain, token, episode_uuid)
    child_environment_record = environment_snapshot(env)
    durable_json(episode / "environment_fingerprint.json", {
        "parent": parent_environment, "child": child_environment_record,
        "worker_python": CHILD_PYTHON,
    })
    parent_admission = environment_admission(os.environ, include_ros2=False)
    durable_json(episode / "parent_environment_admission.json", parent_admission)
    if not parent_admission["passed"]:
        append(journal, {"state": "PARENT_ENV_ADMISSION_FAIL"})
        return False
    child_admission = environment_admission(env)
    durable_json(episode / "child_environment_admission.json", child_admission)
    if not child_admission["passed"]:
        append(journal, {"state": "CHILD_ENV_ADMISSION_FAIL"})
        return False
    transaction = pre_episode_admission(root, identity, parent_environment, child_environment_record,
                                        parent_admission, child_admission)
    durable_json(episode / "pre_episode_admission.json", transaction)
    if not transaction["passed"]:
        append(journal, {"state": "PRE_EPISODE_ADMISSION_FAIL", "reason": transaction["reason"]})
        _record_campaign(root, {"state": "PRE_EPISODE_ADMISSION_FAIL", "episode_index": index,
                                "episode_uuid": episode_uuid, "reason": transaction["reason"]})
        return False
    append(journal, {"state": "PRE_EPISODE_ADMISSION_PASS", **transaction})
    _record_campaign(root, {"state": "PRE_EPISODE_ADMISSION_PASS", "episode_index": index,
                            "episode_uuid": episode_uuid, "token": token, "domain": domain, **transaction})
    append(journal, {"state": "SPAWNING"})
    _record_campaign(root, {"state": "SPAWNING", "episode_index": index, "episode_uuid": episode_uuid})
    append(journal, {"state": "ENV_ADMISSION_PASS",
                     "parent_env_sha256": parent_environment["sha256"],
                     "child_env_sha256": child_environment_record["sha256"]})
    _record_campaign(root, {"state": "ENV_ADMISSION_PASS", "episode_index": index,
                            "episode_uuid": episode_uuid, "domain": domain,
                            "parent_env_sha256": parent_environment["sha256"],
                            "child_env_sha256": child_environment_record["sha256"]})
    matrix = None
    worker = None
    observer = None
    committed = False
    pass_result = False
    seam = {}
    failure = None
    try:
        observer_dir = episode / "passive_lifecycle_observer"
        observer = subprocess.Popen([CHILD_PYTHON, str(OBSERVER), "--output-dir", str(observer_dir)],
                                    env=env, start_new_session=True,
                                    stdout=_open(episode / "observer.out"), stderr=_open(episode / "observer.err"))
        observer_ready_deadline = time.monotonic() + 10.0
        while time.monotonic() < observer_ready_deadline and not (observer_dir / "READY.json").exists():
            if observer.poll() is not None: break
            time.sleep(.02)
        if not (observer_dir / "READY.json").exists():
            failure = "PASSIVE_OBSERVER_NOT_READY"
            raise RuntimeError(failure)
        append(journal, {"state": "PASSIVE_OBSERVER_READY", "observer": _root_record(observer)})
        matrix = subprocess.Popen(MATRIX, env=env, start_new_session=True,
                                  stdout=_open(episode / "matrix.out"), stderr=_open(episode / "matrix.err"))
        matrix_record = _root_record(matrix)
        append(journal, {"state": "MATRIX_COMMITTED", "matrix": matrix_record})
        _record_campaign(root, {"state": "MATRIX_COMMITTED", "episode_index": index,
                                "episode_uuid": episode_uuid, "token": token, "domain": domain,
                                "matrix": matrix_record})

        append(journal, {"state": "HELPER_STARTING"})
        worker = subprocess.Popen(["/usr/bin/python3", str(SEAM), "--output-dir", str(episode),
                                   "--diagnostic-qos", "best_effort"], env=env, start_new_session=True,
                                  stdout=_open(episode / "worker.out"), stderr=_open(episode / "worker.err"))
        worker_record = _root_record(worker)
        worker_spawn_ns = time.monotonic_ns()
        append(journal, {"state": "WORKER_SPAWNED", "worker": worker_record, "worker_spawn_ns": worker_spawn_ns})
        _record_campaign(root, {"state": "WORKER_SPAWNED", "episode_index": index,
                                "episode_uuid": episode_uuid, "token": token, "worker": worker_record,
                                "worker_spawn_ns": worker_spawn_ns})
        append(journal, {"state": "WAITING_FOR_HELPER_ENTER"})
        phase_path = episode / "worker_phase.jsonl"
        prehelper_deadline = time.monotonic() + 20.0
        phase_rows = []
        while time.monotonic() < prehelper_deadline:
            phase_rows = _phases(phase_path)
            accounting = lifecycle_accounting(phase_rows)
            if accounting["state"] == "DUPLICATE_HELPER_ENTER":
                failure = "DUPLICATE_HELPER_ENTER"
                break
            if accounting["committed"]:
                helper_enter = next(row for row in phase_rows if row["phase"] == "HELPER_ENTER")
                committed_ns = helper_enter["monotonic_ns"]
                append(journal, {"state": "READINESS_COMMITTED", "matrix": matrix_record,
                                 "worker": worker_record, "worker_spawn_ns": worker_spawn_ns,
                                 "helper_enter_monotonic_ns": committed_ns})
                _record_campaign(root, {"state": "READINESS_COMMITTED", "episode_index": index,
                                        "episode_uuid": episode_uuid, "token": token, "domain": domain,
                                        "matrix": matrix_record, "worker": worker_record,
                                        "worker_spawn_ns": worker_spawn_ns,
                                        "helper_enter_monotonic_ns": committed_ns})
                committed = True
                append(journal, {"state": "WAITING_PREMISSION_HEALTH"})
                break
            if worker.poll() is not None:
                failure = "WORKER_PRE_HELPER_INITIALIZATION_FAILURE"
                break
            time.sleep(.02)
        if not committed and failure is None:
            failure = "WORKER_PRE_HELPER_HANG"
        if committed and failure is None:
            helper_deadline_ns = committed_ns + int((HELPER_TIMEOUT_SEC + .25) * 1e9)
            final_deadline = committed_ns + int((HELPER_TIMEOUT_SEC + 4.0) * 1e9)
            dumped = False
            while time.monotonic_ns() < final_deadline:
                phase_rows = _phases(phase_path)
                accounting = lifecycle_accounting(phase_rows)
                if accounting["state"] == "DUPLICATE_HELPER_ENTER":
                    failure = "DUPLICATE_HELPER_ENTER"
                    break
                if accounting["outcome"] is not None:
                    if worker.poll() is not None:
                        break
                if not dumped and time.monotonic_ns() >= helper_deadline_ns and accounting["outcome"] is None and worker.poll() is None:
                    os.kill(worker.pid, signal.SIGUSR1)
                    append(journal, {"state": "FAULTHANDLER_SIGUSR1", "helper_enter_monotonic_ns": committed_ns})
                    dumped = True
                time.sleep(.02)
            phase_rows = _phases(phase_path)
            accounting = lifecycle_accounting(phase_rows)
            if accounting["outcome"] is None:
                failure = "HELPER_BLOCKING_NO_OUTCOME"
            elif accounting["outcome"] == "EXCEPTION":
                failure = "HELPER_EXCEPTION"
            elif worker.poll() is None:
                failure = "POST_HELPER_TEARDOWN_HANG"

        ready_path = episode / "premission_seam.json"
        if failure is None and worker.returncode == 0 and ready_path.exists():
            seam = json.loads(ready_path.read_text())
            pass_result = _valid_ready(seam)
            if not pass_result:
                failure = "READY_EVIDENCE_INVALID"
        elif failure is None:
            failure = "HELPER_FAILED_OR_TIMEOUT"
        durable_json(episode / "ready_event.json", {"passed": pass_result, "failure": failure,
                                                       "worker_returncode": None if worker is None else worker.poll(),
                                                       "seam": seam})
        durable_json(episode / "validator_result.json", {"passed": pass_result, "failure": failure,
                                                            "schema": "frozen_ready_event_generation_identity"})
        append(journal, {"state": "READINESS_PASS" if pass_result else "READINESS_FAIL",
                         "failure": failure, "ready": seam.get("event", {})})
        _record_campaign(root, {"state": "READINESS_PASS" if pass_result else "READINESS_FAIL",
                                "episode_index": index, "failure": failure})
    except Exception as exc:
        failure = f"HARNESS_EXCEPTION:{type(exc).__name__}:{exc}"
        durable_json(episode / "ready_event.json", {"passed": False, "failure": failure, "seam": seam})
        append(journal, {"state": "READINESS_FAIL", "failure": failure})
        if committed:
            _record_campaign(root, {"state": "READINESS_FAIL", "episode_index": index, "failure": failure})
        else:
            _record_campaign(root, {"state": "PRECOMMIT_HARNESS_FAILURE", "episode_index": index, "failure": failure})
    finally:
        if matrix is not None:
            cleanup_ok = clean(token, matrix, episode)
        elif observer is not None:
            cleanup_ok = clean(token, observer, episode)
        else:
            durable_json(episode / "process_before_cleanup.json", proc_token(token))
            durable_json(episode / "cleanup_actions.json", {"actions": []})
            durable_json(episode / "process_after_cleanup_0.json", proc_token(token))
            durable_json(episode / "process_after_cleanup_2s.json", proc_token(token))
            cleanup_ok = not proc_token(token)
        global_now = global_target_processes()
        durable_json(episode / "global_process_after_cleanup_0.json", global_now)
        time.sleep(2.0)
        global_delayed = global_target_processes()
        durable_json(episode / "global_process_after_cleanup_2s.json", global_delayed)
        cleanup_ok = cleanup_ok and not global_now and not global_delayed
        append(journal, {"state": "CLEANUP_VERIFIED" if cleanup_ok else "CLEANUP_FAIL"})
        _record_campaign(root, {"state": "CLEANUP_VERIFIED" if cleanup_ok else "CLEANUP_FAIL",
                                "episode_index": index})
        summary = {"identity": identity, "readiness_committed": committed,
                   "matrix": None if matrix is None else _root_record(matrix),
                   "worker": None if worker is None else _root_record(worker),
                   "observer": None if observer is None else _root_record(observer),
                   "readiness_pass": pass_result, "failure": failure, "seam": seam,
                   "cleanup_verified": cleanup_ok, "route_mission": 0, "mission_start": 0,
                   "uuid": 0, "gate_arm": 0, "health_injection": 0}
        durable_json(episode / "summary.json", summary)
        if committed and pass_result and cleanup_ok:
            append(journal, {"state": "SEALED"})
            _record_campaign(root, {"state": "SEALED", "episode_index": index})
        return committed and pass_result and cleanup_ok


def _dummy_multiple_roots(root, index):
    """Actual multiple-root token/cleanup test; it does not start ROS."""
    directory = Path(root) / f"dummy_{index}"
    directory.mkdir()
    token = uuid.uuid4().hex
    env = dict(os.environ, P4E6B_EPISODE_TOKEN=token)
    append(directory / "state_journal.jsonl", {"state": "PLANNED"})
    matrix = subprocess.Popen(["bash", "-c", "sleep 30 & setsid sleep 30 & wait"], env=env,
                              start_new_session=True)
    worker = subprocess.Popen(["bash", "-c", "sleep 30 & setsid sleep 30 & wait"], env=env,
                              start_new_session=True)
    append(directory / "state_journal.jsonl", {"state": "MATRIX_COMMITTED", "matrix": _root_record(matrix)})
    append(directory / "state_journal.jsonl", {"state": "READINESS_COMMITTED", "worker": _root_record(worker)})
    time.sleep(0.2)
    before = proc_token(token)
    cleanup_ok = clean(token, matrix, directory)
    try:
        worker.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass
    durable_json(directory / "result.json", {"token": token, "before": before,
                                               "matrix": _root_record(matrix), "worker": _root_record(worker),
                                               "cleanup_ok": cleanup_ok})
    return cleanup_ok and len(before) >= 4


def selftest(root):
    root = Path(root)
    ownership = [_dummy_multiple_roots(root, index) for index in range(1, 4)]
    crash = root / "crash_journal.jsonl"
    append(crash, {"state": "PLANNED", "case": "before_commit"})
    append(crash, {"state": "READINESS_COMMITTED", "case": "after_commit_no_summary"})
    append(crash, {"state": "WAITING_PREMISSION_HEALTH", "case": "during_observing"})
    append(crash, {"state": "TERMINATING", "case": "during_cleanup"})
    reconstruction = {"precommit_noncounted": ["before_commit"],
                      "counted_even_without_summary": ["after_commit_no_summary", "during_observing", "during_cleanup"]}
    durable_json(root / "crash_reconstruction.json", reconstruction)
    serialization = []
    for index in range(1, 4):
        directory = root / f"serialization_{index}"
        directory.mkdir()
        durable_json(directory / "identity.json", {"index": index})
        append(directory / "state_journal.jsonl", {"state": "READINESS_COMMITTED"})
        durable_json(directory / "summary.json", {"sealed": True})
        append(directory / "state_journal.jsonl", {"state": "SEALED"})
        serialization.append(True)
    passed = all(ownership) and all(serialization)
    durable_json(root / "selftest.json", {"multiple_root_ownership": ownership,
                                           "crash_accounting": reconstruction,
                                           "serialization": serialization, "pass": passed})
    return passed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--index", type=int)
    parser.add_argument("--domain", type=int)
    args = parser.parse_args()
    Path(args.root).mkdir(parents=True, exist_ok=True)
    if args.selftest:
        raise SystemExit(0 if selftest(args.root) else 1)
    if args.index is None or args.domain is None:
        parser.error("--index and --domain are required")
    raise SystemExit(0 if run_one_readiness_episode(args.root, args.index, args.domain) else 2)


if __name__ == "__main__":
    main()
