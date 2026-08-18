"""Bounded no-route Collision Monitor activation probe for B2O.

This tool observes a fresh matrix generation only.  It never sends lifecycle
transitions, starts a mission, publishes a route, arms a gate, or injects
health.  Launch stdout/stderr and token-matched process snapshots are retained
so PROCESS_RUNNING, NODE_PRESENT, and NODE_ACTIVE remain distinct.
"""
import argparse
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def processes(token):
    rows = []
    for p in Path("/proc").glob("[0-9]*"):
        try:
            env = (p / "environ").read_bytes().split(b"\0")
            if ("P4E6B_EPISODE_TOKEN=" + token).encode() not in env:
                continue
            stat = (p / "stat").read_text().split()
            cmd = (p / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            rows.append({"pid": int(p.name), "ppid": int(stat[3]), "pgid": int(stat[4]),
                         "state": stat[2], "starttime": stat[21], "cmdline": cmd})
        except (FileNotFoundError, PermissionError, IndexError, ValueError):
            pass
    return sorted(rows, key=lambda x: x["pid"])


def stop_group(proc):
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=4.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=4.0)


class ProbeNode:
    def __init__(self, out):
        self.node = rclpy.create_node("p4e6b_activation_probe")
        self.out = out
        self.rows = []
        self.diags = []
        self.bools = []
        self.scans = []
        self.state_client = self.node.create_client(GetState, "/collision_monitor/get_state")
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self.diag, 50)
        self.node.create_subscription(Bool, "/system/collision_monitor_valid", self.boolean, 50)
        self.node.create_subscription(LaserScan, "/phase4/synthetic_scan", self.scan, 50)

    def diag(self, msg):
        now = time.monotonic_ns()
        for status in msg.status:
            if status.name != "vehicle_cmd_safety/collision_monitor_validity_monitor":
                continue
            values = {x.key: x.value for x in status.values}
            level = status.level[0] if isinstance(status.level, (bytes, bytearray)) else int(status.level)
            row = {"monotonic_ns": now, "level": level,
                   "message": status.message, "values": values,
                   "state": values.get("state"), "reason_code": values.get("reason_code")}
            self.diags.append(row)

    def boolean(self, msg):
        self.bools.append({"monotonic_ns": time.monotonic_ns(), "value": bool(msg.data)})

    def scan(self, msg):
        self.scans.append({"monotonic_ns": time.monotonic_ns(), "frame_id": msg.header.frame_id,
                           "range_count": len(msg.ranges)})

    def poll(self, token, phase):
        now = time.monotonic_ns()
        names = sorted(self.node.get_node_names_and_namespaces())
        services = self.node.get_service_names_and_types()
        state = None
        service_ready = self.state_client.service_is_ready()
        if service_ready:
            future = self.state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=.15)
            if future.done() and future.exception() is None:
                state = {"id": int(future.result().current_state.id),
                         "label": future.result().current_state.label}
        row = {"monotonic_ns": now, "phase": phase,
               "node_present": any(n == "collision_monitor" for n, _ in names),
               "node_names": names,
               "get_state_service_available": service_ready,
               "get_state": state,
               "get_state_service_listed": any("/collision_monitor/get_state" == n for n, _ in services),
               "processes": processes(token),
               "diag_count": len(self.diags), "bool_count": len(self.bools),
               "scan_count": len(self.scans)}
        self.rows.append(row)

    def close(self):
        self.node.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--domain", type=int, required=True)
    ap.add_argument("--observe-sec", type=float, default=15.0)
    a = ap.parse_args()
    out = Path(a.output_dir); out.mkdir(parents=True, exist_ok=True)
    token = "b2o-" + uuid.uuid4().hex
    env = dict(os.environ, ROS_DOMAIN_ID=str(a.domain), ROS_LOCALHOST_ONLY="1", P4E6B_EPISODE_TOKEN=token)
    (out / "identity.json").write_text(json.dumps({"domain": a.domain, "token": token,
                                                     "route_mission": 0, "mission_start": 0,
                                                     "gate_arm": 0, "health_injection": 0}, indent=2) + "\n")
    os.environ.update(ROS_DOMAIN_ID=str(a.domain), ROS_LOCALHOST_ONLY="1")
    rclpy.init(args=[])
    probe = ProbeNode(out)
    matrix = subprocess.Popen(["ros2", "launch", "parking_robot_bringup",
                                "phase4_p4e6b_health_matrix.launch.py",
                                "enable_health_runner:=false"], env=env,
                               start_new_session=True,
                               stdout=(out / "matrix.stdout").open("w"),
                               stderr=(out / "matrix.stderr").open("w"))
    start = time.monotonic_ns(); deadline = time.monotonic() + a.observe_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(probe.node, timeout_sec=.02)
            probe.poll(token, "OBSERVING")
            time.sleep(.08)
    finally:
        probe.poll(token, "PRE_CLEANUP")
        stop_group(matrix)
        probe.poll(token, "POST_CLEANUP")
        payload = {"probe": "B2O_NO_ROUTE_ACTIVATION", "domain": a.domain, "token": token,
                   "start_ns": start, "end_ns": time.monotonic_ns(),
                   "route_mission": 0, "mission_start": 0, "gate_arm": 0, "health_injection": 0,
                   "lifecycle_observations": probe.rows, "diagnostics": probe.diags,
                   "bools": probe.bools, "scans": probe.scans,
                   "process_exit_code": matrix.returncode,
                   "active_observed": any((r.get("get_state") or {}).get("label") == "active" for r in probe.rows),
                   "valid_observed": any(r.get("reason_code") == "VALID" for r in probe.diags),
                   "bool_true_observed": any(r.get("value") is True for r in probe.bools)}
        (out / "probe.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        probe.close(); rclpy.shutdown()
        subprocess.run(["ros2", "daemon", "stop"], env=env, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)


if __name__ == "__main__":
    main()
