"""Sealed P4-E.6C live binding.

This module is the only approved effectful entry point for a future C.3
campaign.  It computes identities from fixed workspace artifacts and never
accepts callback/module paths from a caller.  C.2A invokes only its
non-effectful admission/binding audit.
"""
from __future__ import annotations

import hashlib
import json
import os
import signal
import subprocess
import time
import re
from contextlib import contextmanager
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from parking_robot_interfaces.msg import MissionState, RouteMission
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
from .phase4_p4e6c_controller import ProgressQualificationController
from .phase4_p4e6c_progress_recorder import FreshEvidenceReader, ProgressEvidenceRecorder, readiness_from_mapping
from .phase4_p4e6c_pose_clamp import PersistentPoseClamp, validate_clamp_snapshot, clamp_warmup_status
from .phase4_p4e6c_progress_evidence import command_pair_ready


WORKSPACE = Path("/home/dog/fastlio_rtabmap_ros2_ws")
REPORTS = Path("/home/dog/phase4_reports")
AUTHORITY_FILES = {
    "C-P01": REPORTS / "P4E6C_CP01_ATTEMPT2_PROSPECTIVE_AUTHORITY.json",
    "C-P02": REPORTS / "P4E6C_CP02_ATTEMPT3_PROSPECTIVE_AUTHORITY.json",
}
HOOK_NAMES = (
    "authoritative_global_zero", "environment_valid", "launch", "processes_alive",
    "campaign_boot_ready", "route_mission_start", "active_identity_ready", "gate_ready",
    "pre_arm_command_ready", "arm_gate", "active_case_ready", "witness_ready", "runner_ready", "injection_ready", "run_after_marker",
    "terminal_adjudication", "physical_adjudication", "cleanup", "witness_seal",
)

PROGRESS_GRAPH_TARGET_MARKERS = (
    "planner_server", "controller_server", "behavior_server", "bt_navigator",
    "lifecycle_manager", "parking_robot_mission_manager", "mission_manager_node", "mission_manager",
    "guarded_vehicle_cmd_gate", "phase4_vehicle_cmd_fake_base", "wheelchair_cmd_adapter",
    "phase4_p4e6", "phase4_p4b", "p4e6b_",
)


def parse_trigger_success(stdout: str, *, returncode: int = 0) -> bool:
    """Parse one semantic success field from the validated Humble response block."""
    if int(returncode) != 0:
        return False
    response = re.search(r"(?im)^\s*response\s*:\s*", stdout)
    if response is None:
        raise RuntimeError("P4E6C_SERVICE_RESPONSE_UNRECOGNIZED")
    block = stdout[response.end():]
    matches = re.findall(
        r"(?:^|[\s(,{])success\s*[:=]\s*(true|false)\b",
        block, flags=re.IGNORECASE | re.MULTILINE)
    if len(matches) != 1:
        raise RuntimeError("P4E6C_SERVICE_RESPONSE_UNRECOGNIZED")
    return matches[0].lower() == "true"


class _FixedRouteObserver(Node):
    def __init__(self):
        super().__init__("phase4_p4e6c_fixed_route_observer")
        self.route_qos = QoSProfile(depth=10)
        self.route_qos.reliability = ReliabilityPolicy.RELIABLE
        self.route_qos.durability = DurabilityPolicy.VOLATILE
        self.state_qos = QoSProfile(depth=10)
        self.state_qos.reliability = ReliabilityPolicy.RELIABLE
        self.state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(RouteMission, "/mission/route", self.route_qos)
        self.received = []
        self.state_subscription = self.create_subscription(
            MissionState, "/mission/state", self._state, self.state_qos)

    def _state(self, message):
        self.received.append({
            "monotonic_ns": time.monotonic_ns(),
            "state": int(message.state),
            "reason_code": str(message.reason_code),
            "mission_id": str(message.mission_id),
            "route_id": str(message.route_id),
            "active_goal_uuid": str(message.active_goal_uuid),
        })

    def fixed_route(self, mission_id: str) -> RouteMission:
        message = RouteMission()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.mission_id = mission_id
        message.route_id = "p4e3a-phase2-straight-two-waypoint"
        message.topology_version = "v1"
        message.node_ids = ["p4e3a-wp0", "p4e3a-wp1"]
        message.edge_ids = ["p4e3a-edge0"]
        message.edge_directions = [1]
        for x in (8.425, 9.425):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = message.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = -53.725
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message


class _FixedServiceClients(Node):
    """Persistent qualification-owned clients for pre-marker service effects."""

    def __init__(self, case: str):
        super().__init__("phase4_p4e6c_service_client")
        self.case = case
        self.start_client = self.create_client(Trigger, "/mission/start")
        self.gate_client = self.create_client(SetBool, "/vehicle_cmd_safety/arm")
        self.recovery_client = None
        if case == "C-P02":
            self.recovery_client = self.create_client(
                Trigger, "/phase4_p4e6c_recovery_feedback_relay/arm_recovery_injection")
            self.recovery_step_client = self.create_client(
                Trigger, "/phase4_p4e6c_recovery_feedback_relay/advance_recovery_step")
            self.recovery_ack_client = self.create_client(
                Trigger, "/phase4_p4e6c_recovery_feedback_relay/acknowledge_recovery_step")
        else:
            self.recovery_step_client = None
            self.recovery_ack_client = None


def _source(relative: str) -> Path:
    return WORKSPACE / relative


def _paths(case: str) -> dict[str, Path]:
    common = {
        "progress_controller": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_controller.py"),
        "progress_runner": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_progress_runner.py"),
        "progress_evidence": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_progress_evidence.py"),
        "progress_recorder": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_progress_recorder.py"),
        "progress_observer": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_progress_observer.py"),
        "progress_witness": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_progress_witness.py"),
        "block_terminal_adjudicator": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6a_persistent_block_runner.py"),
        "physical_adjudicator": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6b_terminal_closure.py"),
        "global_zero_cleanup": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6b_live_evidence.py"),
        "freeze": _source("src/parking_robot_bringup/test/phase4_p4e6b_s01_attempt4_controller.py"),
        "matrix_launch": _source("src/parking_robot_bringup/launch/phase4_p4e6b_health_matrix.launch.py"),
        "fake_base": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_vehicle_cmd_fake_base.py"),
        "pose_clamp_implementation": _source("src/parking_robot_bringup/parking_robot_bringup/phase2_failure_test_runner.py"),
        "mission_progress_source": _source("src/parking_robot_mission_manager/parking_robot_mission_manager/mission_progress_supervisor_core.py"),
        "mission_manager_node": _source("src/parking_robot_mission_manager/parking_robot_mission_manager/mission_manager_node.py"),
        "mission_manager_progress_checker_source": _source("src/parking_robot_nav2_plugins/src/mission_manager_progress_checker.cpp"),
        "mission_manager_progress_checker_installed": _source("build/parking_robot_nav2_plugins/libmission_manager_progress_checker.so"),
        "recovery_feedback_relay_source": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_recovery_feedback_relay.py"),
        "recovery_feedback_relay_installed": _source("install/parking_robot_bringup/lib/python3.10/site-packages/parking_robot_bringup-0.1.0-py3.10.egg"),
        "progress_live_driver": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_live_driver.py"),
        "pose_clamp_stimulator": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_pose_clamp.py"),
        "c4d_qualification_helpers": _source("src/parking_robot_bringup/parking_robot_bringup/phase4_p4e6c_c4d.py"),
    }
    common["c_p01_qualification_launch"] = _source("src/parking_robot_bringup/launch/phase4_p4e6c_c_p01_full_campaign.launch.py")
    common["c_p02_qualification_launch_remap"] = _source("src/parking_robot_bringup/launch/phase4_p4e6c_c_p02_full_campaign.launch.py")
    return common


def sha256_file(path: Path) -> str:
    if not path.is_file():
        raise RuntimeError(f"P4E6C_IDENTITY_PATH_MISSING:{path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_sealed_authority(case: str) -> dict:
    path = AUTHORITY_FILES.get(str(case).upper())
    if path is None or not path.is_file():
        raise RuntimeError("P4E6C_AUTHORITY_FILE_MISSING")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value.get("required_pinned_identity"), dict):
        raise RuntimeError("P4E6C_AUTHORITY_KEYS_MISSING")
    return value


def compute_actual_identities(case: str) -> dict[str, str]:
    return {key: sha256_file(path) for key, path in _paths(str(case).upper()).items()}


class SealedLiveDriver:
    """Fixed-artifact admission driver; no caller-provided runtime hooks."""

    def __init__(self, case: str, authority_path: Path | None = None):
        self.case = str(case).upper()
        if self.case not in AUTHORITY_FILES:
            raise RuntimeError("P4E6C_CASE_DENIED")
        self.authority = (json.loads(Path(authority_path).read_text(encoding="utf-8"))
                          if authority_path is not None else load_sealed_authority(self.case))
        self.expected = dict(self.authority["required_pinned_identity"])

    def admit(self) -> dict:
        actual = compute_actual_identities(self.case)
        missing = sorted(set(self.expected) - set(actual))
        if missing:
            raise RuntimeError("P4E6C_IDENTITY_PATH_MISSING:" + ",".join(missing))
        mismatches = {key: {"expected": self.expected[key], "actual": actual[key]}
                      for key in self.expected if actual.get(key) != self.expected[key]}
        if mismatches:
            raise RuntimeError("P4E6C_IDENTITY_MISMATCH:" + json.dumps(mismatches, sort_keys=True))
        return {"case": self.case, "identity_match": True, "actual": actual,
                "authority_file": str(AUTHORITY_FILES[self.case])}

    def binding_report(self) -> dict:
        report = self.admit()
        hooks = self._build_fixed_hooks()
        report.update({"effectful_entry": "SealedLiveDriver.execute_live",
                       "caller_supplied_hooks": False,
                       "fixed_hooks": sorted(hooks),
                       "hook_implementations": {
                           name: "_FixedRuntimeBindings." + name for name in sorted(hooks)
                       },
                       "retained_evidence_output": str(
                           REPORTS / f"P4E6C_{self.case.replace('-', '')}_{getattr(self, 'attempt_identity', 'UNBOUND')}"),
                       "runtime_environment": {"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
                                                "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY")}})
        return report

    def _runtime_environment(self) -> dict[str, str]:
        command = ("set -a; unset PYTHONHOME PYTHONPATH; "
                   "source /opt/ros/humble/setup.bash; "
                   "source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash; env")
        result = subprocess.run(["bash", "--noprofile", "--norc", "-c", command],
                                text=True, capture_output=True, check=False)
        if result.returncode != 0:
            raise RuntimeError("P4E6C_ROS_ENVIRONMENT_CONSTRUCTION_FAILED")
        environment = {}
        for line in result.stdout.splitlines():
            key, separator, value = line.partition("=")
            if separator:
                environment[key] = value
        environment["ROS_LOCALHOST_ONLY"] = "1"
        environment["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", environment.get("ROS_DOMAIN_ID", ""))
        if not environment["ROS_DOMAIN_ID"]:
            raise RuntimeError("P4E6C_ROS_DOMAIN_MISSING")
        return environment

    def _build_fixed_hooks(self) -> dict:
        """Construct the complete hook set from fixed, sealed components."""
        bindings = _FixedRuntimeBindings(self)
        hooks = {name: getattr(bindings, name) for name in HOOK_NAMES}
        if set(hooks) != set(HOOK_NAMES):
            raise RuntimeError("P4E6C_FIXED_HOOK_SET_INCOMPLETE")
        return hooks

    def execute_live(self, *, attempt_identity: str, supervisor_authority: str) -> dict:
        """The sole effectful entry; all runtime hooks are internally fixed."""
        if supervisor_authority != self._expected_authority():
            raise RuntimeError("P4E6C_LIVE_AUTHORITY_DENIED")
        authorization = os.environ.get("P4E6C_LIVE_AUTHORIZATION")
        if authorization != "SUPERVISOR_C3_EXPLICIT" and not (
                self.case == "C-P02" and authorization in (
                    "SUPERVISOR_AUTHORIZED_CP02_ATTEMPT2_EXCEPTION",
                    "SUPERVISOR_AUTHORIZED_CP02_ATTEMPT3_FINAL_EXCEPTION")):
            raise RuntimeError("P4E6C_EFFECTFUL_ENTRY_DENIED_UNTIL_C3")
        admission = self.admit()
        self.attempt_identity = attempt_identity
        hooks = self._build_fixed_hooks()
        output = REPORTS / f"P4E6C_{self.case.replace('-', '')}_{attempt_identity}"
        controller = ProgressQualificationController(
            self.case, output,
            supervisor_authority=supervisor_authority,
            attempt_number=int(self.authority.get("attempt", 1)))
        return controller.execute_live(
            attempt_identity=attempt_identity,
            supervisor_authority=supervisor_authority,
            expected_identities=self.expected,
            actual_identities=admission["actual"],
            hooks=hooks)

    def _expected_authority(self) -> str:
        return str(self.authority.get("required_authority", ""))


class _FixedRuntimeBindings:
    """Concrete fixed bindings used only after driver admission succeeds."""

    def __init__(self, driver: SealedLiveDriver):
        self.driver = driver
        self.environment = driver._runtime_environment()
        self.process = None
        identity = getattr(driver, "attempt_identity", "UNBOUND")
        self.output = REPORTS / f"P4E6C_{driver.case.replace('-', '')}_{identity}"
        self.recorder = ProgressEvidenceRecorder(self.output, driver.case,
                                                  self._durable_write)
        self.readiness = None
        self.witness = None
        self.owned_launch_pid = None
        self.owned_pgid = None
        self.signal_chronology = []
        self.boot_observations = []
        self._preflight_done = False
        self.service_node = None
        self.service_clients = {}
        self.service_endpoint_ready = False
        self.gate_arm_ns = None

    def _durable_write(self, path: Path, value: dict):
        controller = ProgressQualificationController(self.driver.case, self.output)
        controller._write_json(path, value)

    @contextmanager
    def _same_environment(self):
        old = dict(os.environ)
        try:
            os.environ.clear()
            os.environ.update(self.environment)
            yield
        finally:
            os.environ.clear()
            os.environ.update(old)

    def _command(self, argv: list[str], *, timeout: float = 5.0):
        try:
            return subprocess.run(argv, env=self.environment, text=True,
                                  capture_output=True, timeout=timeout, check=False)
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout if isinstance(exc.stdout, str) else ""
            stderr = exc.stderr if isinstance(exc.stderr, str) else ""
            return subprocess.CompletedProcess(argv, 124, stdout, stderr + "\nTIMEOUT\n")

    def authoritative_global_zero(self, wait_sec: float = 2.0):
        if not self._preflight_done:
            self._preflight_done = True
            return self.progress_global_zero_preflight(wait_sec=wait_sec)
        from .phase4_p4e6b_live_evidence import authoritative_global_zero
        with self._same_environment():
            result = authoritative_global_zero(wait_sec=wait_sec)
        return result

    def progress_global_zero_preflight(self, wait_sec: float = 2.0):
        """Progress-specific zero authority with immutable raw preflight evidence."""
        from .phase4_p4e6b_live_evidence import global_target_scan
        started_ns = time.monotonic_ns()
        with self._same_environment():
            scan1 = global_target_scan()
            time.sleep(wait_sec)
            scan2 = global_target_scan()
            command = ["ros2", "node", "list", "--no-daemon"]
            try:
                graph_result = subprocess.run(command, env=self.environment, text=True,
                                              capture_output=True, timeout=5.0, check=False)
                graph_stdout = graph_result.stdout
                graph_stderr = graph_result.stderr
                graph_returncode = graph_result.returncode
            except subprocess.TimeoutExpired as exc:
                graph_stdout = exc.stdout if isinstance(exc.stdout, str) else ""
                graph_stderr = (exc.stderr if isinstance(exc.stderr, str) else "") + "\nTIMEOUT\n"
                graph_returncode = None
        nodes = [line.strip() for line in graph_stdout.splitlines() if line.strip()]
        target_nodes = [node for node in nodes if any(marker in node for marker in PROGRESS_GRAPH_TARGET_MARKERS)]
        graph = {"command": command, "ROS_DOMAIN_ID": self.environment.get("ROS_DOMAIN_ID"),
                 "ROS_LOCALHOST_ONLY": self.environment.get("ROS_LOCALHOST_ONLY"),
                 "returncode": graph_returncode, "stdout": graph_stdout, "stderr": graph_stderr,
                 "nodes": nodes, "target_nodes": target_nodes,
                 "clean": graph_returncode == 0 and not target_nodes,
                 "target_markers": list(PROGRESS_GRAPH_TARGET_MARKERS)}
        passed = (scan1 == [] and scan2 == [] and graph["returncode"] == 0
                  and graph["target_nodes"] == [] and graph["clean"] is True)
        result = {"case": self.driver.case, "logical_attempt": 1,
                  "attempt_identity": getattr(self.driver, "attempt_identity", ""),
                  "ROS_DOMAIN_ID": self.environment.get("ROS_DOMAIN_ID"),
                  "ROS_LOCALHOST_ONLY": self.environment.get("ROS_LOCALHOST_ONLY"),
                  "started_monotonic_ns": started_ns,
                  "completed_monotonic_ns": time.monotonic_ns(), "scan1": scan1,
                  "scan2": scan2, "wait_sec": wait_sec, "graph": graph, "pass": passed}
        self._durable_write(self.output / "global_zero_preflight.json", result)
        return result

    def environment_valid(self):
        if self.environment.get("ROS_LOCALHOST_ONLY") != "1":
            return False
        probe = self._command(["/usr/bin/python3", "-c",
                               "import importlib.metadata as m; m.distribution('ros2cli')"])
        if probe.returncode != 0:
            return False
        graph = self._command(["ros2", "node", "list", "--no-daemon"])
        return graph.returncode == 0

    def launch(self):
        launch = _paths(self.driver.case)[
            "c_p01_qualification_launch" if self.driver.case == "C-P01"
            else "c_p02_qualification_launch_remap"]
        launch_environment = dict(self.environment)
        launch_environment["P4E6C_EVIDENCE_OUTPUT"] = str(self.output)
        launch_environment["P4E6C_ATTEMPT_IDENTITY"] = str(self.driver.attempt_identity)
        self.output.mkdir(parents=True, exist_ok=True)
        stdout_path = self.output / "campaign_stdout.log"
        stderr_path = self.output / "campaign_stderr.log"
        self.stdout_file = stdout_path.open("ab")
        self.stderr_file = stderr_path.open("ab")
        self.process = subprocess.Popen(["ros2", "launch", str(launch)],
                                        env=launch_environment,
                                        stdout=self.stdout_file, stderr=self.stderr_file,
                                        text=True, start_new_session=True)
        self.owned_launch_pid = int(self.process.pid)
        self.owned_pgid = int(os.getpgid(self.owned_launch_pid))
        if self.owned_pgid == int(os.getpgrp()) or self.owned_pgid <= 0:
            raise RuntimeError("P4E6C_INVALID_OWNED_PROCESS_GROUP")
        self.recorder.record("process_group_events.jsonl", {
            "event": "LAUNCH_OWNERSHIP_CAPTURED",
            "pid": self.owned_launch_pid,
            "pgid": self.owned_pgid,
            "stdout": str(stdout_path),
            "stderr": str(stderr_path),
            "monotonic_ns": time.monotonic_ns(),
        })
        return self.process

    def processes_alive(self, process):
        return process is self.process and process.poll() is None

    def _owned_process_inventory(self) -> list[dict]:
        """Return only processes in this launch's registered process group."""
        if self.owned_pgid is None:
            return []
        rows = []
        for proc in Path("/proc").glob("[0-9]*"):
            try:
                pid = int(proc.name)
                stat = (proc / "stat").read_text().split()
                pgid = int(stat[4])
                if pgid != int(self.owned_pgid):
                    continue
                command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
                rows.append({"pid": pid, "pgid": pgid, "command": command,
                             "executable": command.split(" ", 1)[0] if command else "",
                             "monotonic_ns": time.monotonic_ns()})
            except (FileNotFoundError, PermissionError, IndexError, ValueError):
                continue
        return sorted(rows, key=lambda row: row["pid"])

    def _current_transaction_rows(self, filename: str) -> list[dict]:
        path = self.output / filename
        if not path.is_file():
            return []
        rows = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if row.get("transaction_id") == self.driver.attempt_identity:
                rows.append(row)
        return rows

    def campaign_boot_ready(self, process):
        required_services = {"/mission/start"}
        if self.driver.case == "C-P02":
            required_services.add("/phase4_p4e6c_recovery_feedback_relay/arm_recovery_injection")
        deadline = time.monotonic() + 15.0
        last = None
        stable_services = 0
        previous_heartbeat = -1
        heartbeat_advanced = False
        while time.monotonic() < deadline:
            if not self.processes_alive(process):
                raise RuntimeError("P4E6C_CAMPAIGN_NOT_ALIVE")
            nodes = self._command(["ros2", "node", "list", "--no-daemon"])
            services = self._command(["ros2", "service", "list"])
            inventory = self._owned_process_inventory()
            mission_process = any("mission_manager_node" in row["command"] for row in inventory)
            observer_process = any("phase4_p4e6c_progress_observer" in row["command"] for row in inventory)
            service_lines = set(services.stdout.splitlines()) if services.returncode == 0 else set()
            service_ready = services.returncode == 0 and required_services.issubset(service_lines)
            stable_services = stable_services + 1 if service_ready else 0
            physical_rows = self._current_transaction_rows("physical_evidence.jsonl")
            heartbeat_rows = list(physical_rows)
            if self.driver.case == "C-P02":
                heartbeat_rows += self._current_transaction_rows("recovery_feedback_events.jsonl")
            heartbeat = max((int(row.get("sequence", -1)) for row in heartbeat_rows), default=-1)
            if heartbeat > previous_heartbeat:
                if previous_heartbeat >= 0:
                    heartbeat_advanced = True
                previous_heartbeat = heartbeat
            # Mission Manager policy/state evidence is meaningful only after
            # RouteMission/start.  Boot authority is deliberately limited to
            # owned processes, interface availability, and observer liveness.
            manager_rows = (self._current_transaction_rows("mission_policy_diagnostics.jsonl")
                            + self._current_transaction_rows("mission_state_events.jsonl"))
            relay_rows = self._current_transaction_rows("recovery_feedback_events.jsonl")
            last = {"monotonic_ns": time.monotonic_ns(),
                    "node_list": {"returncode": nodes.returncode, "stdout": nodes.stdout,
                                   "stderr": nodes.stderr},
                    "service_list": {"returncode": services.returncode, "stdout": services.stdout,
                                     "stderr": services.stderr},
                    "launch_returncode": process.poll(),
                    "owned_processes": inventory,
                    "mission_manager_owned": mission_process,
                    "observer_owned": observer_process,
                    "service_ready": service_ready,
                    "stable_service_observations": stable_services,
                    "observer_heartbeat_sequence": heartbeat,
                    "observer_heartbeat_advanced": heartbeat_advanced,
                    "mission_manager_evidence_present": bool(manager_rows),
                    "recovery_relay_ready": any(
                        row.get("relay_state") in {"READY_FORWARD", "READY"}
                        and int(row.get("injection_count") or 0) == 0
                        and int(row.get("canonical_recovery_count") or 0) == 0
                        for row in relay_rows)}
            self.boot_observations.append(last)
            if (mission_process and observer_process and stable_services >= 2
                    and heartbeat_advanced):
                if self.driver.case != "C-P02" or last["recovery_relay_ready"]:
                    self._durable_write(self.output / "boot_readiness.json", {
                        "pass": True, "case": self.driver.case,
                        "attempt_identity": self.driver.attempt_identity,
                        "boot_authority": "OWNED_PROCESS_INTERFACE_HEARTBEAT_EVIDENCE",
                        "observations": self.boot_observations,
                        "launch_stdout": str(self.output / "campaign_stdout.log"),
                        "launch_stderr": str(self.output / "campaign_stderr.log"),
                    })
                    return last
            time.sleep(0.10)
        self._durable_write(self.output / "boot_readiness.json", {
            "pass": False, "case": self.driver.case,
            "attempt_identity": self.driver.attempt_identity,
            "boot_authority": "OWNED_PROCESS_INTERFACE_HEARTBEAT_EVIDENCE",
            "observations": self.boot_observations,
            "final": last,
            "launch_stdout": str(self.output / "campaign_stdout.log"),
            "launch_stderr": str(self.output / "campaign_stderr.log"),
        })
        raise RuntimeError("P4E6C_CAMPAIGN_BOOT_NOT_READY")

    def prepare_service_clients(self):
        """Discover all effectful service servers before route publication."""
        if not rclpy.ok():
            rclpy.init(args=None)
        self.service_node = _FixedServiceClients(self.driver.case)
        specs = {
            "/mission/start": (self.service_node.start_client, "std_srvs/srv/Trigger"),
            "/vehicle_cmd_safety/arm": (self.service_node.gate_client, "std_srvs/srv/SetBool"),
        }
        if self.driver.case == "C-P02":
            specs["/phase4_p4e6c_recovery_feedback_relay/arm_recovery_injection"] = (
                self.service_node.recovery_client, "std_srvs/srv/Trigger")
            specs["/phase4_p4e6c_recovery_feedback_relay/advance_recovery_step"] = (
                self.service_node.recovery_step_client, "std_srvs/srv/Trigger")
            specs["/phase4_p4e6c_recovery_feedback_relay/acknowledge_recovery_step"] = (
                self.service_node.recovery_ack_client, "std_srvs/srv/Trigger")
        deadline = time.monotonic() + 10.0
        observations = []
        while time.monotonic() < deadline:
            available = {}
            for name, (client, service_type) in specs.items():
                available[name] = {
                    "service_type": service_type,
                    "available": bool(client.service_is_ready()),
                    "discovery_monotonic_ns": time.monotonic_ns(),
                }
            observations.append(available)
            if all(item["available"] for item in available.values()):
                evidence = {
                    "transaction_id": self.driver.attempt_identity,
                    "services": available,
                    "expected_server_identity": "not exposed by rclpy service client API",
                    "observations": observations,
                    "pass": True,
                }
                self._durable_write(self.output / "service_endpoint_readiness.json", evidence)
                self.service_clients = {name: client for name, (client, _) in specs.items()}
                self.service_endpoint_ready = True
                return evidence
            rclpy.spin_once(self.service_node, timeout_sec=0.05)
            time.sleep(0.05)
        evidence = {
            "transaction_id": self.driver.attempt_identity,
            "services": observations[-1] if observations else {},
            "observations": observations,
            "pass": False,
        }
        self._durable_write(self.output / "service_endpoint_readiness.json", evidence)
        self._teardown_service_clients()
        raise RuntimeError("P4E6C_SERVICE_ENDPOINTS_NOT_READY")

    def _call_fixed_service(self, name: str, request):
        if not self.service_endpoint_ready or self.service_node is None:
            raise RuntimeError("P4E6C_SERVICE_CLIENT_NOT_READY")
        client = self.service_clients[name]
        future = client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.service_node, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError("P4E6C_SERVICE_RESPONSE_TIMEOUT:" + name)
        response = future.result()
        if response is None:
            raise RuntimeError("P4E6C_SERVICE_RESPONSE_EMPTY:" + name)
        return response

    def route_mission_start(self, process):
        self.prepare_service_clients()
        self.publish_route_and_wait_received()
        self.teardown_route_helper()
        self.route_start_ns = time.monotonic_ns()
        self.recorder.record("mission_service_events.jsonl", {
            "event": "START_REQUEST", "transaction_id": self.driver.attempt_identity,
            "request_monotonic_ns": self.route_start_ns})
        request_ns = self.route_start_ns
        try:
            response = self._call_fixed_service("/mission/start", Trigger.Request())
            success = bool(response.success)
            message = str(response.message)
            exception = None
        except Exception as exc:
            response = None
            success = False
            message = ""
            exception = str(exc)
        response_ns = time.monotonic_ns()
        self.recorder.record("mission_service_events.jsonl", {
            "event": "START_RESPONSE", "transaction_id": self.driver.attempt_identity,
            "request_monotonic_ns": self.route_start_ns,
            "response_monotonic_ns": response_ns, "round_trip_sec": (response_ns - request_ns) / 1e9,
            "success": success, "message": message, "exception": exception,
            "transport": "rclpy_fixed_client"})
        if not success:
            raise RuntimeError("P4E6C_ROUTE_MISSION_START_FAILED")

    def publish_route_and_wait_received(self):
        if getattr(self, "route_node", None) is not None:
            raise RuntimeError("P4E6C_ROUTEMISSION_REPUBLISH_FORBIDDEN")
        if not rclpy.ok():
            rclpy.init(args=None)
        self.route_node = _FixedRouteObserver()
        mission_id = "p4e6c-" + str(self.driver.attempt_identity).lower()
        route = self.route_node.fixed_route(mission_id)
        self._wait_for_route_endpoints()
        publish_ns = time.monotonic_ns()
        try:
            self.route_node.publisher.publish(route)
        except Exception as exc:
            raise RuntimeError("P4E6C_ROUTEMISSION_PUBLICATION_FAILED") from exc
        self.route_publication_count = 1
        self.route_publication_ns = publish_ns
        self.recorder.record("route_mission_events.jsonl", {
            "event": "ROUTE_MISSION_PUBLISHED", "transaction_id": self.driver.attempt_identity,
            "mission_id": route.mission_id, "route_id": route.route_id,
            "topology_version": route.topology_version, "node_ids": list(route.node_ids),
            "edge_ids": list(route.edge_ids), "edge_directions": list(route.edge_directions),
            "poses": [[pose.pose.position.x, pose.pose.position.y,
                        pose.pose.orientation.z, pose.pose.orientation.w]
                       for pose in route.poses],
            "publish_request_monotonic_ns": publish_ns,
            "publish_completion_monotonic_ns": time.monotonic_ns(), "publication_count": 1})
        deadline = time.monotonic() + 10.0
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self.route_node, timeout_sec=0.05)
                received = [row for row in self.route_node.received
                            if row["monotonic_ns"] > publish_ns
                            and row["state"] == MissionState.RECEIVED
                            and row["reason_code"] == "MISSION_RECEIVED"]
                if received:
                    row = received[-1]
                    if row["mission_id"] != route.mission_id or row["route_id"] != route.route_id \
                            or row["active_goal_uuid"]:
                        raise RuntimeError("P4E6C_ROUTEMISSION_IDENTITY_MISMATCH")
                    self.recorder.record("route_mission_events.jsonl", {
                        "event": "ROUTE_MISSION_RECEIVED", "transaction_id": self.driver.attempt_identity,
                        **row, "expected_mission_id": route.mission_id, "expected_route_id": route.route_id})
                    self._wait_for_observer_received(route, publish_ns)
                    self.route_identity = {"mission_id": route.mission_id, "route_id": route.route_id}
                    return row
            raise RuntimeError("P4E6C_ROUTEMISSION_DIRECT_RECEIVED_TIMEOUT")
        except Exception:
            try:
                self.teardown_route_helper()
            except Exception:
                pass
            raise

    def teardown_route_helper(self):
        node = getattr(self, "route_node", None)
        if node is None:
            return {"pass": True, "already_destroyed": True}
        destroy_ns = time.monotonic_ns()
        try:
            before = self._route_endpoint_snapshot()
        except Exception:
            before = {"endpoint_observation_failed": True}
        publisher_destroyed = False
        subscription_destroyed = False
        node_destroyed = False
        try:
            node.destroy_publisher(node.publisher)
            publisher_destroyed = True
        finally:
            try:
                node.destroy_subscription(node.state_subscription)
                subscription_destroyed = True
            finally:
                try:
                    node.destroy_node()
                    node_destroyed = True
                finally:
                    self.route_node = None
                    if getattr(self, "service_node", None) is None and rclpy.ok():
                        rclpy.shutdown()
        evidence = {
            "transaction_id": self.driver.attempt_identity,
            "destroy_request_monotonic_ns": destroy_ns,
            "publisher_destroyed": publisher_destroyed,
            "subscription_destroyed": subscription_destroyed,
            "node_destroyed": node_destroyed,
            "endpoint_observations": {"before": before, "after": {
                "route_publisher": 0, "direct_state_subscription": 0}},
            "pass": publisher_destroyed and subscription_destroyed and node_destroyed,
        }
        self._durable_write(self.output / "route_helper_teardown.json", evidence)
        if not evidence["pass"]:
            raise RuntimeError("P4E6C_ROUTE_HELPER_TEARDOWN_FAILED")
        return evidence

    def _wait_for_observer_received(self, route, publish_ns: int):
        path = self.output / "mission_state_events.jsonl"
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if path.is_file():
                for line in path.read_text(encoding="utf-8").splitlines():
                    if not line.strip():
                        continue
                    try:
                        row = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if row.get("transaction_id") != self.driver.attempt_identity:
                        continue
                    observation_ns = int(row.get("observation_monotonic_ns", -1))
                    if (observation_ns > publish_ns
                            and row.get("state") in ("RECEIVED", MissionState.RECEIVED, 1)
                            and row.get("reason_code") == "MISSION_RECEIVED"
                            and row.get("mission_id") == route.mission_id
                            and row.get("route_id") == route.route_id
                            and not row.get("active_goal_uuid")):
                        self.recorder.record("route_mission_events.jsonl", {
                            "event": "ROUTE_MISSION_OBSERVER_RECEIVED",
                            "transaction_id": self.driver.attempt_identity,
                            "observation": row})
                        return row
            rclpy.spin_once(self.route_node, timeout_sec=0.05)
            time.sleep(0.05)
        raise RuntimeError("P4E6C_OBSERVER_MISSION_STATE_CHANNEL_NOT_READY")

    @staticmethod
    def _qos_dict(info):
        qos = getattr(info, "qos_profile", None)
        if qos is None:
            return {}
        def value(name):
            item = getattr(qos, name, None)
            return getattr(item, "name", str(item))
        return {"reliability": value("reliability"),
                "durability": value("durability"),
                "history": value("history"),
                "depth": int(getattr(qos, "depth", 0))}

    @staticmethod
    def _endpoint_dict(info):
        return {"node_name": str(getattr(info, "node_name", "")),
                "node_namespace": str(getattr(info, "node_namespace", "")),
                "topic_type": str(getattr(info, "topic_type", "")),
                "qos": _FixedRuntimeBindings._qos_dict(info)}

    def _route_endpoint_snapshot(self):
        route_subscribers = self.route_node.get_subscriptions_info_by_topic(
            "/mission/route")
        state_publishers = self.route_node.get_publishers_info_by_topic(
            "/mission/state")
        state_subscribers = self.route_node.get_subscriptions_info_by_topic(
            "/mission/state")
        route_subscribers_json = [self._endpoint_dict(info) for info in route_subscribers]
        state_publishers_json = [self._endpoint_dict(info) for info in state_publishers]
        state_subscribers_json = [self._endpoint_dict(info) for info in state_subscribers]
        route_count = int(self.route_node.publisher.get_subscription_count())
        state_count = len(state_publishers)
        manager_route = [row for row in route_subscribers_json
                         if row["node_name"] in {"mission_manager", "/mission_manager"}]
        manager_state = [row for row in state_publishers_json
                         if row["node_name"] in {"mission_manager", "/mission_manager"}]
        observer_state = [row for row in state_subscribers_json
                          if "phase4_p4e6c_progress_observer" in row["node_name"]]
        return {
            "monotonic_ns": time.monotonic_ns(),
            "route_publisher_subscription_count": route_count,
            "route_subscribers": route_subscribers_json,
            "state_subscription_publisher_count": state_count,
            "state_publishers": state_publishers_json,
            "state_subscribers": state_subscribers_json,
            "mission_manager_route_match": bool(route_count and manager_route),
            "mission_manager_state_match": bool(state_count and manager_state),
            "progress_observer_state_match": bool(observer_state),
            "qos": {"route_publisher": self._qos_dict(self.route_node.publisher.qos_profile),
                    "state_subscription": self._qos_dict(self.route_node.state_subscription.qos_profile)},
        }

    def _wait_for_route_endpoints(self):
        deadline = time.monotonic() + 10.0
        stable = []
        last = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.route_node, timeout_sec=0.05)
            last = self._route_endpoint_snapshot()
            stable.append(last)
            stable = stable[-2:]
            if len(stable) == 2 and all(
                row["mission_manager_route_match"]
                and row["mission_manager_state_match"]
                and row["progress_observer_state_match"] for row in stable):
                evidence = {"transaction_id": self.driver.attempt_identity,
                            "pass": True, "stable_observations": stable,
                            "final": last}
                self._durable_write(self.output / "route_endpoint_readiness.json", evidence)
                return evidence
            time.sleep(0.10)
        evidence = {"transaction_id": self.driver.attempt_identity,
                    "pass": False, "stable_observations": stable,
                    "final": last}
        self._durable_write(self.output / "route_endpoint_readiness.json", evidence)
        raise RuntimeError("P4E6C_ROUTEMISSION_ENDPOINTS_NOT_READY")

    def active_identity_ready(self, process):
        path = self.output / "mission_state_events.jsonl"
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if path.is_file():
                for line in path.read_text(encoding="utf-8").splitlines():
                    if not line.strip():
                        continue
                    row = json.loads(line)
                    if row.get("transaction_id") != self.driver.attempt_identity:
                        continue
                    if int(row.get("observation_monotonic_ns", -1)) <= self.route_start_ns:
                        continue
                    if row.get("state") not in ("NAVIGATING", 4):
                        continue
                    identity = {key: row.get(key) for key in ("mission_id", "route_id", "active_goal_uuid")}
                    if not all(identity.values()):
                        continue
                    self.active_identity = identity
                    self.active_identity_ns = int(row.get("observation_monotonic_ns", time.monotonic_ns()))
                    return identity
            time.sleep(0.05)
        raise RuntimeError("P4E6C_ACTIVE_MISSION_IDENTITY_TIMEOUT")

    def gate_ready(self, process):
        if not self.service_endpoint_ready or "/vehicle_cmd_safety/arm" not in self.service_clients:
            raise RuntimeError("P4E6C_GATE_INTERFACE_NOT_READY")
        return {"interface_ready": True, "service_client_ready": True,
                "monotonic_ns": time.monotonic_ns()}

    @staticmethod
    def _as_bool(value):
        return value is True or str(value).strip().lower() == "true"

    @staticmethod
    def _finite(value):
        try:
            number = float(value)
            return number if number == number and number not in (float("inf"), float("-inf")) else None
        except (TypeError, ValueError):
            return None

    def pre_arm_command_ready(self, process):
        """Prove causal Nav2 raw/safe acquisition before Gate arm."""
        active_ns = int(getattr(self, "active_identity_ns", time.monotonic_ns()))
        deadline_ns = active_ns + 1_600_000_000
        evidence = {"transaction_id": self.driver.attempt_identity,
                    "active_goal_uuid": self.active_identity["active_goal_uuid"],
                    "active_uuid_timestamp": active_ns, "deadline_sec": 1.6,
                    "pass": False, "observations": []}
        while time.monotonic_ns() < deadline_ns:
            terminal = self._premarker_terminal(after=active_ns)
            if terminal:
                evidence.update({"failure": terminal, "failure_class": "COMPETING_TERMINAL"})
                self._durable_write(self.output / "pre_arm_command_readiness.json", evidence)
                raise RuntimeError("P4E6C_PREMARKER_TERMINAL:" + terminal)
            gates = [row for row in self._current_transaction_rows("gate_state_events.jsonl")
                     if int(row.get("observation_monotonic_ns", -1)) > active_ns]
            policies = [row for row in self._current_transaction_rows("mission_policy_diagnostics.jsonl")
                        if int(row.get("observation_monotonic_ns", row.get("monotonic_ns", -1))) > active_ns]
            physical = [row for row in self._current_transaction_rows("physical_evidence.jsonl")
                        if int(row.get("monotonic_ns", row.get("observation_monotonic_ns", -1))) > active_ns]
            raw = [row for row in physical if row.get("source") == "raw_command"
                   and bool(row.get("movement_intent"))]
            safe = [row for row in physical if row.get("source") == "canonical_command"
                    and (abs(float(row.get("linear_x", 0.0))) > 1e-6
                         or abs(float(row.get("angular_z", 0.0))) > 1e-6)]
            pairs = []
            for safe_row in safe:
                safe_ns = int(safe_row.get("monotonic_ns", safe_row.get("observation_monotonic_ns", -1)))
                candidates = [row for row in raw if int(row.get("monotonic_ns", row.get("observation_monotonic_ns", -1))) <= safe_ns
                              and safe_ns - int(row.get("monotonic_ns", row.get("observation_monotonic_ns", -1))) <= 75_000_000]
                if not candidates:
                    continue
                raw_row = candidates[-1]
                delta = max(abs(float(raw_row.get(key, 0.0)) - float(safe_row.get(key, 0.0)))
                            for key in ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"))
                if delta <= 0.05:
                    raw_ns = int(raw_row.get("monotonic_ns", raw_row.get("observation_monotonic_ns", -1)))
                    pairs.append({"raw": raw_row, "safe": safe_row, "skew_ns": safe_ns - raw_ns, "max_delta": delta})
            gate = gates[-1] if gates else {}
            gate_values = gate.get("values", gate)
            policy = policies[-1].get("values", policies[-1]) if policies else {}
            safe_age = self._finite(gate_values.get("safe_twist_age_sec"))
            gate_ok = (gate_values.get("state") == "DISARMED"
                       and not self._as_bool(gate_values.get("fault_latched"))
                       and all(self._as_bool(gate_values.get(key)) for key in
                               ("controller_valid", "localization_valid", "collision_monitor_valid"))
                       and all(int(gate_values.get(key, 0)) == 1 for key in
                               ("controller_publisher_count", "localization_publisher_count", "collision_valid_publisher_count", "safe_input_publisher_count"))
                       and safe_age is not None and safe_age < 0.250)
            stream_ok = (policy.get("command_stream_phase") == "STREAM_ESTABLISHED"
                         and self._finite(policy.get("raw_command_age_sec")) is not None
                         and self._finite(policy.get("safe_command_age_sec")) is not None)
            entry = {"monotonic_ns": time.monotonic_ns(), "gate": gate_values,
                     "policy": policy, "safe_twist_age_sec": safe_age,
                     "raw_command_age_sec": self._finite(policy.get("raw_command_age_sec")),
                     "safe_command_age_sec": self._finite(policy.get("safe_command_age_sec")),
                     "pair_count": len(pairs), "pairs": pairs[-4:]}
            evidence["observations"].append(entry)
            if gate_ok and stream_ok and len(pairs) >= 2:
                evidence.update({"gate_state": gate_values, "command_stream_phase": policy.get("command_stream_phase"),
                                 "raw_command_age_sec": entry["raw_command_age_sec"],
                                 "safe_command_age_sec": entry["safe_command_age_sec"],
                                 "movement_producing": True, "causal_pairs": pairs[-4:],
                                 "readiness_monotonic_ns": time.monotonic_ns(), "pass": True})
                self._durable_write(self.output / "pre_arm_command_readiness.json", evidence)
                self.pre_arm_readiness = evidence
                return evidence
            time.sleep(0.03)
        evidence.update({"failure": "PRE_COMMAND_ACQUISITION_TIMEOUT", "failure_class": "PRE_ARM_COMMAND_ACQUISITION_TIMEOUT"})
        self._durable_write(self.output / "pre_arm_command_readiness.json", evidence)
        raise RuntimeError("PRE_ARM_COMMAND_ACQUISITION_TIMEOUT")

    def arm_gate(self, process):
        if not getattr(self, "pre_arm_readiness", {}).get("pass"):
            raise RuntimeError("P4E6C_PRE_ARM_COMMAND_NOT_READY")
        request_ns = time.monotonic_ns()
        self.recorder.record("gate_service_events.jsonl", {
            "event": "ACTIVE_IDENTITY_ACCEPTED", "transaction_id": self.driver.attempt_identity,
            "monotonic_ns": request_ns, **self.active_identity})
        try:
            response = self._call_fixed_service("/vehicle_cmd_safety/arm", SetBool.Request(data=True))
            success = bool(response.success)
            message = str(response.message)
            exception = None
        except Exception as exc:
            success = False
            message = ""
            exception = str(exc)
        response_ns = time.monotonic_ns()
        self.recorder.record("gate_service_events.jsonl", {
            "event": "GATE_ARM_RESPONSE", "transaction_id": self.driver.attempt_identity,
            "request_monotonic_ns": request_ns, "response_monotonic_ns": response_ns,
            "round_trip_sec": (response_ns - request_ns) / 1e9, "success": success,
            "message": message, "exception": exception, **self.active_identity})
        if not success:
            raise RuntimeError("P4E6C_GATE_ARM_FAILED")
        self.gate_arm_ns = request_ns

    def _premarker_terminal(self, after=None):
        for filename in ("mission_state_events.jsonl", "progress_events.jsonl",
                          "mission_policy_diagnostics.jsonl"):
            for row in self._current_transaction_rows(filename):
                stamp = int(row.get("observation_monotonic_ns", row.get("monotonic_ns", -1)))
                boundary = self.gate_arm_ns if after is None else after
                if boundary is None or stamp <= boundary:
                    continue
                values = row.get("values", row)
                state = str(values.get("state", ""))
                reason = str(values.get("reason_code", values.get("progress_supervisor_event", "")))
                if state in {"CANCELLING", "CANCELLED", "BLOCKED", "FAILED", "SUCCEEDED"}:
                    return reason or state
                if reason in {"GATE_DISARMED", "GATE_FAULT", "COLLISION_MONITOR_INVALID",
                              "LOCALIZATION_INVALID", "CONTROLLER_INVALID",
                              "INITIAL_COMMAND_ACQUISITION_TIMEOUT", "CANCELLED", "BLOCKED", "FAILED", "SUCCEEDED"}:
                    return reason
        return None

    def _wait_post_arm_witness(self):
        deadline = time.monotonic() + 10.0
        gate_seen = False
        policy_seen = False
        while time.monotonic() < deadline:
            terminal = self._premarker_terminal()
            if terminal:
                raise RuntimeError("P4E6C_PREMARKER_TERMINAL:" + terminal)
            for row in self._current_transaction_rows("gate_state_events.jsonl"):
                if int(row.get("observation_monotonic_ns", -1)) > int(self.gate_arm_ns or -1):
                    if row.get("state") == "ARMED" and row.get("active_goal_uuid") == self.active_identity["active_goal_uuid"]:
                        gate_seen = True
            for row in self._current_transaction_rows("mission_policy_diagnostics.jsonl"):
                stamp = int(row.get("observation_monotonic_ns", row.get("monotonic_ns", -1)))
                values = row.get("values", row)
                if stamp <= int(self.gate_arm_ns or -1):
                    continue
                if (values.get("state") == "NAVIGATING"
                        and values.get("mission_id") == self.active_identity["mission_id"]
                        and values.get("route_id") == self.active_identity["route_id"]
                        and values.get("active_goal_uuid") == self.active_identity["active_goal_uuid"]
                        and values.get("progress_supervisor_event", "") != "GATE_DISARMED"):
                    policy_seen = True
            if gate_seen and policy_seen:
                return {"gate_armed": True, "post_arm_policy": True,
                        "monotonic_ns": time.monotonic_ns()}
            time.sleep(0.05)
        raise RuntimeError("P4E6C_POST_ARM_WITNESS_TIMEOUT")

    def active_case_ready(self, process):
        self._wait_post_arm_witness()
        deadline = time.monotonic() + 10.0
        seen_policy = set()
        last_reason = "SOURCE_EVIDENCE_STALE"
        while time.monotonic() < deadline:
            policies = self._current_transaction_rows("mission_policy_diagnostics.jsonl")
            policies = [row for row in policies if int(row.get("observation_monotonic_ns", row.get("monotonic_ns", -1))) > int(self.gate_arm_ns or -1)]
            if policies:
                row = policies[-1]
                sequence = int(row.get("sequence", -1))
                if sequence not in seen_policy:
                    seen_policy.add(sequence)
                    candidate, reason = self._source_readiness_candidate(row)
                    self.recorder.record("readiness_candidate_events.jsonl", candidate)
                    last_reason = reason or "SOURCE_EVIDENCE_STALE"
                    if candidate["pass"]:
                        values = candidate["readiness"]
                        self.readiness = readiness_from_mapping(values, case=self.driver.case,
                                                                require_active_case=True)
                        self.recorder.write_readiness(values)
                        return self.readiness
            terminal = self._premarker_terminal(after=int(self.gate_arm_ns or -1))
            if terminal:
                raise RuntimeError("P4E6C_PREMARKER_TERMINAL:" + terminal)
            time.sleep(0.05)
        raise RuntimeError("P4E6C_ACTIVE_CASE_READINESS_TIMEOUT:" + last_reason)

    def _source_readiness_candidate(self, policy_row):
        values = policy_row.get("values", policy_row)
        stamp = int(policy_row.get("observation_monotonic_ns", policy_row.get("monotonic_ns", -1)))
        candidate = {"transaction_id": self.driver.attempt_identity,
                     "candidate_monotonic_ns": time.monotonic_ns(),
                     "mission_policy_sequence": policy_row.get("sequence"),
                     "mission_policy_monotonic_ns": stamp,
                     "mission_id": values.get("mission_id", ""),
                     "route_id": values.get("route_id", ""),
                     "active_goal_uuid": values.get("active_goal_uuid", ""),
                     "policy_activation_state": values.get("progress_policy_activation_state"),
                     "policy_activation_latched": values.get("progress_policy_activation_latched"),
                     "command_stream_phase": values.get("command_stream_phase"),
                     "command_pair_state": values.get("command_pair_state"),
                     "collision_classification": values.get("collision_classification"),
                     "pass": False}
        reason = None
        expected = self.active_identity
        if values.get("state") != "NAVIGATING": reason = "MISSION_NOT_ACTIVE"
        elif any(values.get(key) != expected[key] for key in ("mission_id", "route_id", "active_goal_uuid")):
            reason = "IDENTITY_MISMATCH"
        elif values.get("progress_policy_activation_state") != "ACTIVE" or str(values.get("progress_policy_activation_latched", "")).lower() != "true":
            reason = "POLICY_NOT_ACTIVE"
        elif values.get("command_stream_phase") != "STREAM_ESTABLISHED": reason = "COMMAND_STREAM_NOT_ESTABLISHED"
        else:
            pair = command_pair_ready({**values, "pre_arm_causal_proof": bool(getattr(self, "pre_arm_readiness", {}).get("pass"))})
            candidate.update(pair)
            reason = pair.get("rejection_reason")
        gate_rows = [row for row in self._current_transaction_rows("gate_state_events.jsonl")
                     if int(row.get("observation_monotonic_ns", -1)) > int(self.gate_arm_ns or -1)
                     and row.get("mission_id") == expected["mission_id"]
                     and row.get("route_id") == expected["route_id"]
                     and row.get("active_goal_uuid") == expected["active_goal_uuid"]]
        gate = gate_rows[-1] if gate_rows else {}
        gate_values = gate.get("values", gate)
        gate_stamp = int(gate.get("observation_monotonic_ns", -1))
        candidate.update({"gate_sequence": gate.get("sequence"), "gate_monotonic_ns": gate_stamp,
                          "gate_armed": gate.get("gate_armed"), "gate_state": gate.get("state"),
                          "gate_fault_latched": gate_values.get("fault_latched"),
                          "safe_twist_age_sec": gate_values.get("safe_twist_age_sec")})
        if reason is None and (gate.get("gate_armed") is not True or gate.get("state") != "ARMED"):
            reason = "GATE_NOT_ARMED"
        elif reason is None and str(gate_values.get("fault_latched", "")).lower() == "true": reason = "GATE_FAULTED"
        safe_age = self._finite(gate_values.get("safe_twist_age_sec"))
        if reason is None and (safe_age is None or safe_age >= .250): reason = "SAFE_INPUT_NOT_CURRENT"
        if reason is None and values.get("collision_classification") != "CLEAR": reason = "COLLISION_NOT_CLEAR"
        def current(name, limit):
            number = self._finite(values.get(name + "_age_sec"))
            return number is not None and number <= limit
        if reason is None and not current("localization", .5): reason = "HEALTH_NOT_CURRENT"
        if reason is None and not current("controller", .5): reason = "HEALTH_NOT_CURRENT"
        if reason is None and not current("odometry", .5): reason = "ODOM_NOT_CURRENT"
        if reason is None and not current("tf", .5): reason = "TF_NOT_CURRENT"
        if reason is None and not current("feedback", 2.0): reason = "FEEDBACK_NOT_CURRENT"
        if reason is None and str(values.get("state")) in {"CANCELLED", "BLOCKED", "FAILED", "SUCCEEDED"}: reason = "ACTION_TERMINAL"
        physical = [row for row in self._current_transaction_rows("physical_evidence.jsonl")
                    if int(row.get("monotonic_ns", row.get("observation_monotonic_ns", -1))) > int(self.gate_arm_ns or -1)]
        movement = [row for row in physical if row.get("source") == "raw_command" and row.get("movement_intent")]
        safe = [row for row in physical if row.get("source") == "canonical_command"]
        latest_movement = movement[-1] if movement else {}
        latest_safe = safe[-1] if safe else {}
        if reason is None and not latest_movement: reason = "MOVEMENT_INTENT_MISSING"
        raw_recovery_delta = values.get("recovery_delta")
        recovery_delta = 999 if raw_recovery_delta is None else int(raw_recovery_delta)
        if reason is None and recovery_delta >= 6: reason = "RECOVERY_NOT_READY"
        relay_ready = True
        if self.driver.case == "C-P02":
            relay_rows = [row for row in self._current_transaction_rows("recovery_feedback_events.jsonl")
                          if int(row.get("monotonic_ns", row.get("observation_monotonic_ns", -1)))
                          > int(self.gate_arm_ns or -1)
                          and row.get("transaction_id") == self.driver.attempt_identity]
            relay = relay_rows[-1] if relay_rows else {}
            relay_ready = (relay.get("relay_state") in {"READY_FORWARD", "READY"}
                           and int(relay.get("injection_count", -1)) == 0
                           and int(relay.get("canonical_recovery_count", -1)) == 0)
            if reason is None and not relay_ready: reason = "RECOVERY_NOT_READY"
        candidate.update({"movement_intent": bool(latest_movement),
                          "command_raw_monotonic_ns": latest_movement.get("monotonic_ns", latest_movement.get("observation_monotonic_ns")),
                          "command_safe_monotonic_ns": latest_safe.get("monotonic_ns", latest_safe.get("observation_monotonic_ns")),
                          "mission_active": values.get("state") == "NAVIGATING",
                          "action_terminal": False, "recovery_delta": recovery_delta,
                          "health_current": reason not in {"HEALTH_NOT_CURRENT"},
                          "odom_current": current("odometry", .5), "tf_current": current("tf", .5),
                          "feedback_current": current("feedback", 2.0),
                          "collision_clear": values.get("collision_classification") == "CLEAR",
                          "recovery_relay_ready": relay_ready})
        if reason is not None:
            candidate["rejection_reason"] = reason
            return candidate, reason
        readiness = {"mission_active": True, "policy_active": True, "gate_armed": True,
                     "collision_clear": True, "health_current": True,
                     "odom_current": True, "tf_current": True, "feedback_current": True,
                     "command_pair_current": True, "command_pair_branch": candidate.get("command_pair_branch"),
                     "pair_liveness_age_sec": candidate.get("pair_liveness_age_sec"),
                     "raw_command_age_sec": candidate.get("raw_command_age_sec"),
                     "safe_command_age_sec": candidate.get("safe_command_age_sec"),
                     "movement_intent": True,
                     "action_terminal": False, "recovery_delta": recovery_delta,
                     "mission_id": expected["mission_id"], "route_id": expected["route_id"],
                     "active_goal_uuid": expected["active_goal_uuid"],
                     "transaction_id": self.driver.attempt_identity,
                     "observation_monotonic_ns": stamp,
                     "policy_activation_state": values.get("progress_policy_activation_state"),
                     "policy_activation_latched": values.get("progress_policy_activation_latched"),
                     "command_stream_phase": values.get("command_stream_phase"),
                     "command_pair_state": values.get("command_pair_state"),
                     "gate_state": gate.get("state"),
                     "recovery_relay_ready": relay_ready,
                     "collision_classification": values.get("collision_classification")}
        candidate["readiness"] = readiness
        candidate["pass"] = True
        return candidate, None

    def _fresh_json(self, filename: str, *, after: int, timeout_sec: float) -> dict:
        deadline = time.monotonic() + timeout_sec
        path = self.output / filename
        while time.monotonic() < deadline:
            if path.is_file():
                value = json.loads(path.read_text(encoding="utf-8"))
                if int(value.get("observation_monotonic_ns", -1)) > after:
                    if value.get("transaction_id") != getattr(self.driver, "attempt_identity", None):
                        raise RuntimeError("P4E6C_ACTIVE_IDENTITY_TRANSACTION_MISMATCH")
                    return value
            time.sleep(0.05)
        raise RuntimeError("P4E6C_FRESH_EVIDENCE_TIMEOUT:" + filename)

    def witness_ready(self, process):
        from .phase4_p4e6c_progress_witness import ProgressWitness
        self.witness = ProgressWitness(self.output, case=self.driver.case)
        identity = {"mission_id": self.readiness.mission_id,
                    "route_id": self.readiness.route_id,
                    "active_goal_uuid": self.readiness.active_goal_uuid}
        return self.witness.mark_ready(identity=identity, mechanism={"ready": True})

    def runner_ready(self, process):
        from .phase4_p4e6c_progress_runner import LiveProgressRunner
        return LiveProgressRunner(self.driver.case)

    def injection_ready(self, process):
        stimulator = self._pose_clamp_ready()
        if self.driver.case == "C-P01":
            return stimulator
        services = self._command(["ros2", "service", "list"])
        if "arm_recovery_injection" not in services.stdout:
            raise RuntimeError("P4E6C_RECOVERY_RELAY_NOT_READY")
        path = self.output / "recovery_feedback_events.jsonl"
        ready = []
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not ready:
            if path.is_file():
                rows = [json.loads(line) for line in path.read_text().splitlines()
                        if line.strip() and json.loads(line).get("transaction_id") == self.driver.attempt_identity]
                ready = [row for row in rows if row.get("relay_state") == "READY_FORWARD"
                         and int(row.get("injection_count", 0)) == 0
                         and int(row.get("canonical_recovery_count") or 0) == 0]
            if not ready:
                time.sleep(0.05)
        if not ready:
            raise RuntimeError("P4E6C_RECOVERY_RELAY_NOT_READY")
        return {"ready": True, "mechanism": "shadow_feedback_relay",
                "pose_clamp_stimulator": stimulator}

    def _pose_clamp_ready(self):
        if getattr(self, "pose_clamp", None) is not None:
            raise RuntimeError("STIMULATOR_DUPLICATE_INSTANCE")
        self.pose_clamp = PersistentPoseClamp(
            transaction_id=self.driver.attempt_identity,
            output=self.output,
            event_callback=self.recorder.record_clamp,
            acceptance_callback=lambda row: self.recorder.record(
                "clamp_acceptance_events.jsonl", row))
        self.pose_clamp.start_executor()
        anchor = self.pose_clamp.capture_anchor(timeout_sec=5.0)
        readiness = self.pose_clamp.ready()
        readiness.update({"transaction_id": self.driver.attempt_identity,
                          "readiness": "POSE_CLAMP_STIMULATOR_READY",
                          "captured_anchor": anchor,
                          "evidence_recorder_ready": True})
        self._durable_write(self.output / "pose_clamp_stimulator_ready.json", readiness)
        return readiness

    def run_after_marker(self, process):
        from .phase4_p4e6c_progress_runner import LiveProgressRunner
        runner = LiveProgressRunner(self.driver.case)
        pose_command = ["ros2", "topic", "pub", "--once", "/initialpose",
                        "geometry_msgs/msg/PoseWithCovarianceStamped",
                        "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}"]

        if getattr(self, "pose_clamp", None) is None:
            raise RuntimeError("STIMULATOR_NOT_READY")
        clamp_duration = 25.0 if self.driver.case == "C-P01" else 20.0
        self.pose_clamp.run(clamp_duration)
        self.pose_clamp.wait_first_tick(timeout_sec=0.2)
        if self.driver.case == "C-P02":
            self._wait_clamp_warmup(required=5, timeout_sec=1.0)

        # The persistent scheduler owns actual publication cadence.  This
        # callback deliberately does no I/O; runner observation remains
        # concurrent with the publisher.
        def publish_initialpose():
            return None

        def latest(path_name):
            path = self.output / path_name
            if not path.is_file():
                raise RuntimeError("P4E6C_OBSERVATION_FILE_MISSING:" + path_name)
            rows = [line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
            if not rows:
                raise RuntimeError("P4E6C_OBSERVATION_FILE_EMPTY:" + path_name)
            return json.loads(rows[-1])

        progress_reader = self._fresh_reader(self.output / "progress_events.jsonl")

        def observe_progress():
            return progress_reader.next()

        if self.driver.case == "C-P01":
            try:
                return runner.run_cp01(
                    evidence=self.readiness.admission,
                    publish_initialpose=publish_initialpose,
                    observe=observe_progress,
                    marker_present=True,
                    duration_sec=clamp_duration,
                    record_clamp=None,
                    record_progress=None)
            finally:
                stimulator = self.pose_clamp.stop()
                validate_clamp_snapshot(stimulator)
                self._durable_write(self.output / "pose_clamp_stimulator.json", stimulator)
                for event in stimulator["events"]:
                    self.recorder.record_clamp(event)
                for event in stimulator["acceptance_events"]:
                    self.recorder.record("clamp_acceptance_events.jsonl", event)

        recovery_reader = self._fresh_reader(self.output / "recovery_feedback_events.jsonl")
        source_reader = self._fresh_reader(self.output / "mission_policy_diagnostics.jsonl")
        receipts_path = self.output / "feedback_receipts.jsonl"
        receipts = [int(row["monotonic_ns"]) / 1e9 for row in
                    (json.loads(line) for line in receipts_path.read_text().splitlines()
                     if line.strip()) if row.get("transaction_id") == self.driver.attempt_identity]
        receipts = receipts[-6:]

        def observe_recovery():
            return recovery_reader.next(timeout_sec=0.25)

        def arm_recovery():
            request_ns = time.monotonic_ns()
            try:
                response = self._call_fixed_service(
                    "/phase4_p4e6c_recovery_feedback_relay/arm_recovery_injection",
                    Trigger.Request())
                success = bool(response.success)
                message = str(response.message)
                exception = None
            except Exception as exc:
                success = False
                message = ""
                exception = str(exc)
            response_ns = time.monotonic_ns()
            self.recorder.record("mission_service_events.jsonl", {
                "event": "RECOVERY_ARM_RESPONSE", "transaction_id": self.driver.attempt_identity,
                "request_monotonic_ns": request_ns, "response_monotonic_ns": response_ns,
                "round_trip_sec": (response_ns - request_ns) / 1e9,
                "success": success, "message": message, "exception": exception,
                "transport": "rclpy_fixed_client"})
            if not success:
                raise RuntimeError("P4E6C_RECOVERY_INJECTION_ARM_FAILED")
            self.recorder.record("block_cancel_ack_events.jsonl", {
                "event": "RECOVERY_INJECTION_ARMED", "transaction_id": self.driver.attempt_identity,
                "message": message, "monotonic_ns": time.monotonic_ns(),
                "relay_state": "ARMED"})

        def advance_recovery(step):
            request_ns = time.monotonic_ns()
            response = self._call_fixed_service(
                "/phase4_p4e6c_recovery_feedback_relay/advance_recovery_step",
                Trigger.Request())
            response_ns = time.monotonic_ns()
            if not response.success:
                raise RuntimeError("P4E6C_RECOVERY_STEP_REQUEST_FAILED")
            self.recorder.record("recovery_feedback_events.jsonl", {
                "event": "RECOVERY_STEP_REQUESTED",
                "transaction_id": self.driver.attempt_identity,
                "requested_shadow_recovery_count": int(step),
                "request_monotonic_ns": request_ns,
                "response_monotonic_ns": response_ns,
                "relay_state": "INJECTION_ACTIVE",
                "message": str(response.message)})

        def observe_source_ack(step):
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    row = source_reader.next(timeout_sec=0.25)
                except RuntimeError as exc:
                    if "STALE_EVIDENCE_ROW" in str(exc):
                        continue
                    raise
                if (int(row.get("recovery_delta", -1)) == int(step)
                        and row.get("mission_id") == self.readiness.mission_id
                        and row.get("route_id") == self.readiness.route_id
                        and row.get("active_goal_uuid") == self.readiness.active_goal_uuid
                        and row.get("measurable_progress") is False
                        and row.get("collision_classification") == "CLEAR"
                        and (int(step) == 6 or not row.get("progress_event"))):
                    return row
            raise RuntimeError("P4E6C_MISSION_MANAGER_RECOVERY_STEP_ACK_TIMEOUT")

        def acknowledge_recovery(source_row):
            response = self._call_fixed_service(
                "/phase4_p4e6c_recovery_feedback_relay/acknowledge_recovery_step",
                Trigger.Request())
            if not response.success:
                raise RuntimeError("P4E6C_RECOVERY_SOURCE_ACK_FORWARD_FAILED")
            self.recorder.record_recovery({
                **source_row, "event": "MISSION_MANAGER_STEP_ACKNOWLEDGED",
                "source_acknowledgement_monotonic_ns": time.monotonic_ns()})

        try:
            return runner.run_cp02(
                evidence=self.readiness.admission,
                publish_initialpose=publish_initialpose,
                arm_recovery=arm_recovery,
                observe_feedback=observe_recovery,
                canonical_receipt_probe=lambda: receipts,
                marker_present=True,
                duration_sec=clamp_duration,
                record_clamp=None,
                record_recovery=self.recorder.record_recovery,
                step_recovery=advance_recovery,
                acknowledge_recovery=acknowledge_recovery,
                observe_source_ack=observe_source_ack)
        finally:
            pending = None
            stimulator = self.pose_clamp.stop()
            try:
                validate_clamp_snapshot(stimulator, minimum_publications=None, duration_aware=True)
            except Exception as exc:
                pending = exc
            self._durable_write(self.output / "pose_clamp_stimulator.json", stimulator)
            if self.driver.case == "C-P02":
                try:
                    self._terminal_drain(expected_reason="RECOVERY_EXHAUSTED_NO_PROGRESS")
                except Exception as exc:
                    if pending is None:
                        pending = exc
            if pending is not None:
                raise pending

    def _fresh_reader(self, path: Path) -> FreshEvidenceReader:
        sequence = -1; monotonic_ns = -1
        if path.is_file():
            for line in path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                row = json.loads(line)
                if row.get("transaction_id") != self.driver.attempt_identity:
                    continue
                sequence = max(sequence, int(row.get("sequence", -1)))
                monotonic_ns = max(monotonic_ns, int(row.get("monotonic_ns", row.get("observation_monotonic_ns", -1))))
        return FreshEvidenceReader(path, baseline_sequence=sequence,
                                    baseline_monotonic_ns=monotonic_ns,
                                    transaction_id=self.driver.attempt_identity)

    def _wait_clamp_warmup(self, *, required: int, timeout_sec: float):
        """Wait while scheduler runs; recovery is not reachable before PASS."""
        start_ns = time.monotonic_ns()
        first_ns = None
        last_count = 0
        self.recorder.record("clamp_warmup_events.jsonl", {
            "event": "START_CLAMP", "transaction_id": self.driver.attempt_identity,
            "scheduler_start_monotonic_ns": start_ns, "required_publications": required})
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            with self.pose_clamp._lock:
                events = list(self.pose_clamp.events)
            count = len(events)
            if count and first_ns is None:
                first_ns = int(events[0]["actual_publish_monotonic_ns"])
            if count != last_count:
                last_count = count
                current_hz = self.pose_clamp.snapshot().get("effective_frequency_hz", 0.0)
                self.recorder.record("clamp_warmup_events.jsonl", {
                    "event": "PUBLICATION_OBSERVED", "transaction_id": self.driver.attempt_identity,
                    "publication_count": count, "publication_sequence": count - 1,
                    "first_publication_monotonic_ns": first_ns,
                    "current_actual_frequency_hz": current_hz,
                    "monotonic_ns": time.monotonic_ns()})
            if clamp_warmup_status(count, required=required) == "CLAMP_WARMUP_READY":
                self.recorder.record("clamp_warmup_events.jsonl", {
                    "event": "CLAMP_WARMUP_READY", "transaction_id": self.driver.attempt_identity,
                    "warmup_ready_monotonic_ns": time.monotonic_ns(),
                    "final_warmup_count": count, "pass": True})
                return {"pass": True, "publication_count": count}
            time.sleep(0.01)
        self.recorder.record("clamp_warmup_events.jsonl", {
            "event": "CLAMP_WARMUP_PUBLICATION_TIMEOUT", "transaction_id": self.driver.attempt_identity,
            "final_warmup_count": last_count, "pass": False})
        raise RuntimeError("CLAMP_WARMUP_PUBLICATION_TIMEOUT")

    def _terminal_drain(self, *, expected_reason: str, timeout_sec: float = 5.0):
        """Keep observers/campaign alive until the complete same-UUID block tail."""
        deadline = time.monotonic() + float(timeout_sec)
        seen = set()
        path = self.output / "terminal_drain_events.jsonl"
        def rows(name):
            source = self.output / name
            if not source.is_file():
                return []
            return [json.loads(line) for line in source.read_text(encoding="utf-8").splitlines()
                    if line.strip() and json.loads(line).get("transaction_id") == self.driver.attempt_identity]
        while time.monotonic() < deadline:
            policy = rows("mission_policy_diagnostics.jsonl")
            states = rows("mission_state_events.jsonl")
            acks = rows("block_cancel_ack_events.jsonl")
            actions = rows("navigate_action_status_events.jsonl")
            physical = rows("physical_evidence.jsonl")
            if any(r.get("reason_code") == expected_reason or r.get("progress_supervisor_event") == expected_reason for r in policy + states):
                seen.add(expected_reason)
                self.recorder.record("terminal_drain_events.jsonl", {"event": expected_reason,
                    "transaction_id": self.driver.attempt_identity, "monotonic_ns": time.monotonic_ns()})
            if any(r.get("event") == "BLOCK_CANCEL_ACK_ACCEPTED" for r in acks):
                seen.add("BLOCK_CANCEL_ACK_ACCEPTED")
            if any(r.get("event") == "CANCELING" or r.get("status") == 3 for r in acks + actions):
                seen.add("CANCELING_3")
            if any(r.get("event") == "CANCELED" or r.get("status") == 5 for r in acks + actions):
                seen.add("CANCELED_5")
            if any(r.get("state") == "BLOCKED" or r.get("reason_code") == expected_reason for r in states):
                seen.add("BLOCKED")
            if any(r.get("source") == "applied_command" and r.get("zero") is True for r in physical):
                seen.add("APPLIED_ZERO_PERSISTENT")
            if {expected_reason, "BLOCK_CANCEL_ACK_ACCEPTED", "CANCELING_3", "CANCELED_5", "BLOCKED", "APPLIED_ZERO_PERSISTENT"}.issubset(seen):
                self.recorder.record("terminal_drain_events.jsonl", {"event": "TERMINAL_DRAIN_COMPLETE",
                    "transaction_id": self.driver.attempt_identity, "monotonic_ns": time.monotonic_ns()})
                return {"pass": True, "drain_end_ns": time.monotonic_ns(), "events": sorted(seen)}
            time.sleep(0.05)
        self.recorder.record("terminal_drain_events.jsonl", {"event": "TERMINAL_DRAIN_TIMEOUT",
            "transaction_id": self.driver.attempt_identity, "monotonic_ns": time.monotonic_ns(),
            "events": sorted(seen)})
        raise RuntimeError("P4E6C_TERMINAL_DRAIN_TIMEOUT")

    def terminal_adjudication(self, runtime):
        from .phase4_p4e6c_progress_evidence import validate_block_terminal
        path = self.output / "block_cancel_ack_events.jsonl"
        if not path.is_file():
            raise RuntimeError("P4E6C_BLOCK_EVIDENCE_MISSING")
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        expected = self.driver.authority["expected_reason"]
        aggregate = {"originating_reason": None, "block_cancel_ack": None,
                     "cancel_request_count": 0, "canceling_count": 0,
                     "canceled_count": 0, "duplicate_cancel": 0,
                     "terminal_state": None, "mission_id": None, "route_id": None,
                     "active_goal_uuid": None}
        for row in rows:
            aggregate.update({key: row[key] for key in aggregate if key in row})
            if row.get("event") == "BLOCK_CANCEL_REQUEST": aggregate["cancel_request_count"] += 1
            if row.get("event") == "CANCELING" or row.get("status") == 3: aggregate["canceling_count"] += 1
            if row.get("event") == "CANCELED" or row.get("status") == 5: aggregate["canceled_count"] += 1
            if row.get("event") == "DUPLICATE_CANCEL": aggregate["duplicate_cancel"] += 1
        states_path = self.output / "mission_state_events.jsonl"
        if states_path.is_file():
            for line in states_path.read_text().splitlines():
                if line.strip():
                    state = json.loads(line); aggregate.update({key: state[key] for key in
                        ("mission_id", "route_id", "active_goal_uuid") if key in state})
                    if state.get("state") == "BLOCKED" or state.get("state") == 10:
                        aggregate["terminal_state"] = "BLOCKED"
                    if state.get("reason_code"): aggregate["originating_reason"] = state["reason_code"]
        status_path = self.output / "navigate_action_status_events.jsonl"
        if status_path.is_file():
            for line in status_path.read_text().splitlines():
                if not line.strip():
                    continue
                status_row = json.loads(line)
                for status in status_row.get("statuses", []):
                    code = int(status.get("status", -1))
                    if code == 3:
                        aggregate["canceling_count"] += 1
                    elif code == 5:
                        aggregate["canceled_count"] += 1
        return validate_block_terminal(aggregate, expected)

    def physical_adjudication(self, runtime):
        path = self.output / "physical_evidence.jsonl"
        if not path.is_file():
            raise RuntimeError("P4E6C_PHYSICAL_EVIDENCE_MISSING")
        values = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if not values:
            raise RuntimeError("P4E6C_PHYSICAL_EVIDENCE_EMPTY")
        from .phase4_p4e6b_terminal_closure import physical_closure_from_rows
        vehicle = [row for row in values if row.get("source") == "canonical_command"]
        applied = [row for row in values if row.get("source") == "applied_command"]
        odometry = [row for row in values if row.get("source") == "odometry"]
        result = physical_closure_from_rows(vehicle_rows=vehicle, applied_rows=applied,
                                            odometry_rows=odometry,
                                            terminal_ns=int(runtime.get("terminal_ns", 0)),
                                            drain_end_ns=int(runtime.get("drain_end_ns", time.monotonic_ns())))
        self.recorder.record("physical_evidence.jsonl", {"event": "PHYSICAL_ADJUDICATION", **result})
        if not result.get("pass"):
            raise RuntimeError("P4E6C_PHYSICAL_ADJUDICATION_FAILED")
        return result

    def cleanup(self, process):
        if process is not self.process:
            return {"pass": False, "classification": "PROCESS_OWNERSHIP_FAILURE"}
        pgid = self.owned_pgid
        if pgid is None or pgid <= 0 or pgid == os.getpgrp():
            return {"pass": False, "classification": "PROCESS_GROUP_OWNERSHIP_FAILURE"}
        def group_exists():
            try:
                os.killpg(pgid, 0)
                return True
            except ProcessLookupError:
                return False
            except PermissionError:
                return True
        if group_exists():
            term_ns = time.monotonic_ns()
            os.killpg(pgid, signal.SIGTERM)
            self.signal_chronology.append({"signal": "SIGTERM", "pgid": pgid,
                                           "monotonic_ns": term_ns})
            deadline = time.monotonic() + 5.0
            while group_exists() and time.monotonic() < deadline:
                time.sleep(0.05)
            if group_exists():
                kill_ns = time.monotonic_ns()
                os.killpg(pgid, signal.SIGKILL)
                self.signal_chronology.append({"signal": "SIGKILL", "pgid": pgid,
                                               "monotonic_ns": kill_ns})
                deadline = time.monotonic() + 5.0
                while group_exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
        try:
            process.wait(timeout=1.0)
        except (subprocess.TimeoutExpired, ChildProcessError):
            pass
        for stream_name in ("stdout_file", "stderr_file"):
            stream = getattr(self, stream_name, None)
            if stream is not None and not stream.closed:
                stream.close()
        initial = self.authoritative_global_zero()
        result = initial
        settle_used = False
        graph_convergence = []
        owned = {int(pgid)}
        first = initial.get("scan1") or []
        second = initial.get("scan2") or []
        if not initial.get("pass", False) and first and not second and all(int(row.get("pgid", -1)) in owned for row in first):
            result = self.authoritative_global_zero()
            settle_used = True
        elif (not initial.get("pass", False) and not first and not second
              and not bool((initial.get("graph") or {}).get("clean"))):
            target_nodes = (initial.get("graph") or {}).get("target_nodes") or []
            allowed_markers = (
                "planner_server", "controller_server", "behavior_server", "bt_navigator",
                "collision_monitor", "guarded_vehicle_cmd_gate", "phase4_vehicle_cmd_fake_base",
                "wheelchair_cmd_adapter", "parking_robot_mission_manager", "p4e6b_",
            )
            graph_only_owned = bool(target_nodes) and all(
                any(marker in str(node) for marker in allowed_markers) for node in target_nodes
            )
            if graph_only_owned:
                convergence_start = time.monotonic()
                convergence_failed = None
                while time.monotonic() - convergence_start <= 30.0:
                    observation = self.authoritative_global_zero(wait_sec=1.0)
                    graph = observation.get("graph") or {}
                    targets = graph.get("target_nodes") or []
                    inventory = self._owned_process_inventory()
                    process_zero = not observation.get("scan1") and not observation.get("scan2")
                    group_gone = not group_exists()
                    topology_ok = all(any(marker in str(node) for marker in allowed_markers)
                                      for node in targets)
                    entry = {"label": "DDS_GRAPH_CONVERGENCE_WAIT",
                             "elapsed_sec": time.monotonic() - convergence_start,
                             "scan1": observation.get("scan1"),
                             "scan2": observation.get("scan2"),
                             "target_nodes": targets,
                             "process_zero": process_zero,
                             "owned_group_absent": group_gone,
                             "topology_ok": topology_ok,
                             "graph": graph}
                    graph_convergence.append(entry)
                    if not group_gone or not process_zero:
                        convergence_failed = "PROCESS_RESIDUE_DURING_GRAPH_CONVERGENCE"
                        break
                    if not topology_ok:
                        convergence_failed = "FOREIGN_GRAPH_TARGET_DURING_CONVERGENCE"
                        break
                    if graph.get("returncode") == 0 and not targets and graph.get("clean"):
                        final = self.authoritative_global_zero()
                        result = final
                        settle_used = bool(final.get("pass"))
                        if not final.get("pass"):
                            convergence_failed = "FINAL_GRAPH_ZERO_FAILED"
                        break
                    time.sleep(1.0)
                else:
                    convergence_failed = "DDS_GRAPH_CONVERGENCE_TIMEOUT"
                if convergence_failed:
                    result = dict(result)
                    result["pass"] = False
                    result["graph_convergence_failure"] = convergence_failed
        result = dict(result)
        result["initial"] = initial
        result["settle_observation_used"] = settle_used
        result["owned_launch_pid"] = self.owned_launch_pid
        result["owned_pgid"] = pgid
        result["signal_chronology"] = self.signal_chronology
        result["owned_group_disappeared"] = not group_exists()
        result["graph_convergence"] = graph_convergence
        self.recorder.write_cleanup(result)
        if getattr(self, "pose_clamp", None) is not None:
            try:
                self.pose_clamp.destroy()
            finally:
                self.pose_clamp = None
        if getattr(self, "route_node", None) is not None:
            try:
                self.teardown_route_helper()
            except Exception as exc:
                result["pass"] = False
                result["route_helper_teardown_error"] = str(exc)
        self._teardown_service_clients()
        return result

    def _teardown_service_clients(self):
        node = getattr(self, "service_node", None)
        if node is None:
            return
        try:
            node.destroy_client(node.start_client)
            node.destroy_client(node.gate_client)
            if node.recovery_client is not None:
                node.destroy_client(node.recovery_client)
            node.destroy_node()
        finally:
            self.service_node = None
            self.service_clients = {}
            self.service_endpoint_ready = False
            if rclpy.ok():
                rclpy.shutdown()

    def witness_seal(self, runtime, terminal, physical, cleanup):
        if self.witness is None:
            raise RuntimeError("P4E6C_WITNESS_TRANSACTION_MISSING")
        return self.witness.seal(identity=runtime.get("identity", {}), outcome={
            "terminal_evidence_complete": bool(terminal.get("pass")),
            "physical_evidence_complete": bool(physical.get("pass")),
            "cleanup": cleanup})


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", required=True, choices=("C-P01", "C-P02"))
    parser.add_argument("--attempt-identity", required=True)
    parser.add_argument("--supervisor-authority", required=True)
    args = parser.parse_args()
    return SealedLiveDriver(args.case).execute_live(
        attempt_identity=args.attempt_identity,
        supervisor_authority=args.supervisor_authority)
