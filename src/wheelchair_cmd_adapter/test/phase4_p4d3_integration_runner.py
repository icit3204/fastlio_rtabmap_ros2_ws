from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import platform
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool

from vehicle_cmd_safety.guarded_vehicle_cmd_gate import GuardedVehicleCmdGate
from wheelchair_cmd_adapter.mock_wheelchair_cmd_adapter import MockWheelchairCmdAdapter


SAFE = "/cmd_vel_nav_safe"
GATE_OUT = "/vehicle_cmd_safe"
MOCK_OUT = "/wheelchair_control_command_mock"
PERMISSIONS = [
    "/system/localization_valid",
    "/system/controller_valid",
    "/system/collision_monitor_valid",
]
FORBIDDEN = [
    "/cmd_vel_nav_raw", "/cmd_vel", "/wheelchair_control_command",
    "/wheelchair_control_command_raw",
]
ZERO_ARRAY = [0.0, 0.0, 0.0]
STRAIGHT_ARRAY = [10000.0, 100.0, 0.0]


def now_record(node: Node, info=None) -> dict:
    gid = list(info.publisher_gid) if info is not None else []
    return {"monotonic_ns": time.monotonic_ns(), "ros_receipt_ns": node.get_clock().now().nanoseconds,
            "publisher_gid": gid}


class Fixtures(Node):
    def __init__(self):
        super().__init__("p4d3_synthetic_fixtures")
        self.safe_pub = self.create_publisher(Twist, SAFE, 10)
        self.permission_pubs = {topic: self.create_publisher(Bool, topic, 10) for topic in PERMISSIONS}
        self.safe = (0.10, 0.0)
        self.collision_enabled = True
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self._tick, clock=self.clock)
        self.events = []
        self.last_collision_publish_ns = None

    def command(self, v: float, w: float):
        self.safe = (v, w)
        self.events.append({"event": "safe_command", "monotonic_ns": time.monotonic_ns(), "linear_x": v, "angular_z": w})

    def _tick(self):
        msg = Twist(); msg.linear.x = self.safe[0]; msg.angular.z = self.safe[1]
        self.safe_pub.publish(msg)
        valid = Bool(); valid.data = True
        for topic, pub in self.permission_pubs.items():
            if topic == PERMISSIONS[2] and not self.collision_enabled:
                continue
            pub.publish(valid)
            if topic == PERMISSIONS[2]:
                self.last_collision_publish_ns = time.monotonic_ns()


class Evidence(Node):
    def __init__(self):
        super().__init__("p4d3_evidence_monitor")
        self.safe = []; self.gate = []; self.mock = []; self.permissions = []
        self.gate_state = []; self.gate_diag = []; self.adapter_diag = []
        self.create_subscription(Twist, SAFE, self._safe, 200)
        self.create_subscription(TwistStamped, GATE_OUT, self._gate, 200)
        self.create_subscription(Float32MultiArray, MOCK_OUT, self._mock, 200)
        for topic in PERMISSIONS:
            self.create_subscription(Bool, topic, lambda msg, t=topic: self._permission(t, msg), 200)
        self.create_subscription(DiagnosticStatus, "/vehicle_cmd_safety/state", self._state, 200)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._gate_diagnostic, 200)
        self.create_subscription(DiagnosticArray, "/wheelchair_cmd_adapter/diagnostics", self._adapter_diagnostic, 200)

    def _safe(self, msg):
        row = now_record(self); row.update({"linear_x": float(msg.linear.x), "linear_y": float(msg.linear.y),
            "linear_z": float(msg.linear.z), "angular_x": float(msg.angular.x), "angular_y": float(msg.angular.y),
            "angular_z": float(msg.angular.z), "frame": ""}); self.safe.append(row)

    def _gate(self, msg):
        row = now_record(self); row.update({"linear_x": float(msg.twist.linear.x), "linear_y": float(msg.twist.linear.y),
            "linear_z": float(msg.twist.linear.z), "angular_x": float(msg.twist.angular.x), "angular_y": float(msg.twist.angular.y),
            "angular_z": float(msg.twist.angular.z), "frame": msg.header.frame_id,
            "header_stamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec}); self.gate.append(row)

    def _mock(self, msg):
        row = now_record(self); row.update({"array": [float(x) for x in msg.data], "frame": ""}); self.mock.append(row)

    def _permission(self, topic, msg):
        row = now_record(self); row.update({"topic": topic, "value": bool(msg.data)}); self.permissions.append(row)

    @staticmethod
    def _status(msg, node):
        raw_level = msg.level
        level = raw_level[0] if isinstance(raw_level, (bytes, bytearray, memoryview)) else int(raw_level)
        row = now_record(node); row.update({"name": msg.name, "level": level, "message": msg.message,
            "hardware_id": msg.hardware_id, "values": {x.key: x.value for x in msg.values}}); return row

    def _state(self, msg): self.gate_state.append(self._status(msg, self))
    def _gate_diagnostic(self, msg):
        for status in msg.status:
            if status.name == "vehicle_cmd_safety/guarded_vehicle_cmd_gate": self.gate_diag.append(self._status(status, self))
    def _adapter_diagnostic(self, msg):
        for status in msg.status:
            if status.name == "mock_wheelchair_cmd_adapter": self.adapter_diag.append(self._status(status, self))


class Probe(Node):
    def __init__(self): super().__init__("p4d3_graph_probe")
    def snapshot(self):
        topics = {n: t for n, t in self.get_topic_names_and_types()}
        watched = [SAFE, GATE_OUT, MOCK_OUT] + PERMISSIONS
        return {"monotonic_ns": time.monotonic_ns(), "publishers": {t: self._infos(t, True) for t in watched},
                "subscriptions": {t: self._infos(t, False) for t in watched},
                "forbidden": {t: topics.get(t, []) for t in FORBIDDEN}}
    def _infos(self, topic, publishers):
        infos = self.get_publishers_info_by_topic(topic) if publishers else self.get_subscriptions_info_by_topic(topic)
        return [{"node_name": x.node_name, "node_namespace": x.node_namespace, "topic_type": x.topic_type,
                 "gid": list(x.endpoint_gid)} for x in infos]


def spin(executor, seconds, predicate=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if predicate and predicate(): return True
    return bool(predicate and predicate())


def close(a, b, eps=1e-4): return abs(a - b) <= eps
def gate_matches(row, v, w): return close(row["linear_x"], v) and close(row["angular_z"], w)
def array_matches(row, expected): return len(row["array"]) == 3 and all(close(a, b) for a, b in zip(row["array"], expected))


def first(rows, start_ns, predicate): return next((x for x in rows if x["monotonic_ns"] >= start_ns and predicate(x)), None)
def last(rows, end_ns, predicate):
    found = [x for x in rows if x["monotonic_ns"] <= end_ns and predicate(x)]
    return found[-1] if found else None


def rate_window(rows, start_ns, predicate, minimum_sec, label):
    matched = [x for x in rows if x["monotonic_ns"] >= start_ns and predicate(x)]
    if len(matched) < 2: raise RuntimeError(f"{label}: insufficient samples")
    duration = (matched[-1]["monotonic_ns"] - matched[0]["monotonic_ns"]) / 1e9
    rate = (len(matched) - 1) / duration
    if duration < minimum_sec or not 18.0 <= rate <= 22.0:
        raise RuntimeError(f"{label}: duration={duration:.6f} rate={rate:.6f}")
    return {"label": label, "start_ns": matched[0]["monotonic_ns"], "end_ns": matched[-1]["monotonic_ns"],
            "count": len(matched), "duration_sec": duration, "rate_hz": rate}


def call_arm(executor, client, enable, events):
    if not client.wait_for_service(timeout_sec=1.0): raise RuntimeError("arm service unavailable")
    request = SetBool.Request(); request.data = enable; request_ns = time.monotonic_ns()
    future = client.call_async(request)
    if not spin(executor, 2.0, future.done): raise RuntimeError("arm call timeout")
    response = future.result(); event = {"monotonic_ns": request_ns, "data": enable, "success": response.success,
                                        "message": response.message}; events.append(event)
    return event


def require_authority(snapshot, gate_present=True):
    expected = {SAFE: 1, GATE_OUT: 1 if gate_present else 0, MOCK_OUT: 1, **{x: 1 for x in PERMISSIONS}}
    for topic, count in expected.items():
        actual = len(snapshot["publishers"][topic])
        if actual != count: raise RuntimeError(f"{topic}: expected {count} publishers, got {actual}")
    present = {k: v for k, v in snapshot["forbidden"].items() if v}
    if present: raise RuntimeError(f"forbidden topics present: {present}")
    mock_subscribers = snapshot["subscriptions"][MOCK_OUT]
    if [(x["node_name"], x["node_namespace"]) for x in mock_subscribers] != [("p4d3_evidence_monitor", "/")]:
        raise RuntimeError(f"unexpected mock output subscribers: {mock_subscribers}")


def write_tsv(path, rows):
    if not rows: path.write_text(""); return
    keys = []
    for row in rows:
        for key in row:
            if key not in keys: keys.append(key)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys, delimiter="\t", extrasaction="ignore"); writer.writeheader()
        for row in rows: writer.writerow({k: json.dumps(v, sort_keys=True) if isinstance(v, (list, dict)) else v for k, v in row.items()})


def write_jsonl(path, rows):
    path.write_text("".join(json.dumps(x, sort_keys=True) + "\n" for x in rows))


def persist(out_dir, evidence, fixture, graphs, authority, scenario_events, arm_events, metrics, start_ns):
    end_ns = time.monotonic_ns()
    env = [{"key": k, "value": os.environ.get(k, "")} for k in ["ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"]]
    env += [{"key": "python", "value": platform.python_version()}, {"key": "hostname", "value": platform.node()}]
    latest = graphs[-1]["snapshot"] if graphs else {"publishers": {}}
    topic_rows = [(SAFE, evidence.safe), (GATE_OUT, evidence.gate), (MOCK_OUT, evidence.mock)]
    topic_rows += [(topic, [x for x in evidence.permissions if x["topic"] == topic]) for topic in PERMISSIONS]
    for topic, rows in topic_rows:
        infos = latest["publishers"].get(topic, [])
        if len(infos) == 1:
            for row in rows:
                row["publisher_gid"] = infos[0]["gid"]
                row["attributed_node"] = infos[0]["node_name"]
                row["attribution_basis"] = "unique_graph_authority"
        elif topic == GATE_OUT and len(infos) == 0:
            prior = next((g["snapshot"]["publishers"][topic][0] for g in reversed(graphs[:-1])
                          if len(g["snapshot"]["publishers"][topic]) == 1), None)
            if prior:
                for row in rows:
                    row["publisher_gid"] = prior["gid"]; row["attributed_node"] = prior["node_name"]
                    row["attribution_basis"] = "last_unique_graph_authority_before_loss"
    write_tsv(out_dir / "process_environment.tsv", env)
    write_tsv(out_dir / "process_lifetime.tsv", [{"process": "in_process_integration_campaign", "start_ns": start_ns, "end_ns": end_ns,
                                                    "exit_expected": True}])
    write_tsv(out_dir / "graph_timeline.tsv", graphs)
    write_tsv(out_dir / "safe_twist_timeline.tsv", evidence.safe)
    write_tsv(out_dir / "vehicle_cmd_safe_timeline.tsv", evidence.gate)
    write_tsv(out_dir / "mock_wheelchair_output_timeline.tsv", evidence.mock)
    write_tsv(out_dir / "gate_state_timeline.tsv", evidence.gate_state)
    write_jsonl(out_dir / "gate_diagnostics.jsonl", evidence.gate_diag)
    write_jsonl(out_dir / "adapter_diagnostics.jsonl", evidence.adapter_diag)
    write_tsv(out_dir / "permission_timeline.tsv", evidence.permissions)
    write_jsonl(out_dir / "arm_service_events.jsonl", arm_events)
    provenance = []
    for graph in graphs:
        snap = graph["snapshot"]
        for topic, infos in snap["publishers"].items():
            for x in infos: provenance.append({"snapshot": graph["label"], "topic": topic, **x})
    write_tsv(out_dir / "publisher_provenance.tsv", provenance)
    write_tsv(out_dir / "authority_timeline.tsv", authority)
    write_jsonl(out_dir / "scenario_events.jsonl", scenario_events + fixture.events)
    (out_dir / "terminal_metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")


def run(scenario, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True); start_ns = time.monotonic_ns()
    rclpy.init(args=["--ros-args", "-p", "max_forward_velocity:=0.20", "-p", "max_angular_velocity:=0.50",
                     "-p", "max_linear_increase_rate:=0.50", "-p", "max_angular_increase_rate:=1.00"])
    gate = GuardedVehicleCmdGate(); adapter = MockWheelchairCmdAdapter(); fixture = Fixtures(); evidence = Evidence(); probe = Probe()
    nodes = [gate, adapter, fixture, evidence, probe]; executor = SingleThreadedExecutor()
    for node in nodes: executor.add_node(node)
    client = evidence.create_client(SetBool, "/vehicle_cmd_safety/arm")
    graphs = []; authority = []; events = []; arm_events = []; windows = []; timing = {}; statuses = []
    metrics = {"scenario": scenario, "status": "PASS", "required_statuses": statuses, "timing": timing, "windows": windows}
    try:
        spin(executor, 2.4)
        snap = probe.snapshot(); require_authority(snap)
        graphs.append({"label": "readiness", "monotonic_ns": snap["monotonic_ns"], "snapshot": snap})
        authority.append({"label": "readiness", "monotonic_ns": snap["monotonic_ns"],
                          **{t: len(snap["publishers"][t]) for t in [SAFE, GATE_OUT, MOCK_OUT] + PERMISSIONS}})
        readiness_start = time.monotonic_ns() - 2_000_000_000
        windows.append(rate_window(evidence.gate, readiness_start, lambda x: gate_matches(x, 0.0, 0.0), 1.0, "disarmed_gate_zero"))
        windows.append(rate_window(evidence.mock, readiness_start, lambda x: array_matches(x, ZERO_ARRAY), 1.0, "disarmed_mock_zero"))
        if not any(x["values"].get("condition") == "VALID" and x["values"].get("input_age_sec") not in (None, "None") for x in evidence.adapter_diag[-30:]):
            raise RuntimeError("adapter did not classify fresh gate zero VALID")
        statuses.append("P4D3_GATE_ADAPTER_CHAIN_READINESS_PASS")
        if scenario == "readiness":
            persist(out_dir, evidence, fixture, graphs, authority, events, arm_events, metrics, start_ns); return metrics

        arm = call_arm(executor, client, True, arm_events)
        if not arm["success"]: raise RuntimeError(f"arm failed: {arm}")
        timing["arm_request_ns"] = arm["monotonic_ns"]
        spin(executor, 2.6)
        first_gate = first(evidence.gate, arm["monotonic_ns"], lambda x: x["linear_x"] > 0.0)
        first_mock = first(evidence.mock, arm["monotonic_ns"], lambda x: array_matches(x, STRAIGHT_ARRAY))
        if not first_gate or not first_mock: raise RuntimeError("straight conversion did not start")
        timing["first_nonzero_gate_ns"] = first_gate["monotonic_ns"]; timing["first_nonzero_mock_ns"] = first_mock["monotonic_ns"]
        timing["gate_to_mock_straight_sec"] = (first_mock["monotonic_ns"] - first_gate["monotonic_ns"]) / 1e9
        stable = time.monotonic_ns() - 2_200_000_000
        windows.append(rate_window(evidence.gate, stable, lambda x: gate_matches(x, .1, 0), 2.0, "straight_gate"))
        windows.append(rate_window(evidence.mock, stable, lambda x: array_matches(x, STRAIGHT_ARRAY), 2.0, "straight_mock"))

        for label, w, expected in [("left", .05, [-2000.0, 100.0, 0.0]), ("right", -.05, [2000.0, 100.0, 0.0])]:
            fixture.command(.1, w); command_ns = fixture.events[-1]["monotonic_ns"]; timing[f"{label}_command_ns"] = command_ns
            spin(executor, 1.4)
            stable_gate = first(evidence.gate, command_ns, lambda x, ww=w: gate_matches(x, .1, ww))
            output = first(evidence.mock, stable_gate["monotonic_ns"] if stable_gate else command_ns,
                           lambda x, exp=expected: array_matches(x, exp))
            if not stable_gate or not output: raise RuntimeError(f"{label} stable conversion absent")
            timing[f"{label}_stable_gate_ns"] = stable_gate["monotonic_ns"]
            timing[f"{label}_first_mock_ns"] = output["monotonic_ns"]
            timing[f"{label}_gate_to_mock_sec"] = (output["monotonic_ns"] - stable_gate["monotonic_ns"]) / 1e9
            windows.append(rate_window(evidence.mock, output["monotonic_ns"], lambda x, exp=expected: array_matches(x, exp), .7, f"{label}_mock"))
        fixture.command(.1, 0.0); spin(executor, 1.0)
        statuses.append("P4D3_GATE_ADAPTER_VALID_CONVERSION_PASS")

        if scenario == "gate_loss":
            final_gate = evidence.gate[-1]; timing["final_vehicle_cmd_safe_receipt_ns"] = final_gate["monotonic_ns"]
            executor.remove_node(gate); gate.destroy_node(); nodes.remove(gate)
            loss_start = time.monotonic_ns(); events.append({"event": "gate_terminated", "monotonic_ns": loss_start})
            spin(executor, 2.8)
            snap = probe.snapshot(); require_authority(snap, gate_present=False)
            graphs.append({"label": "gate_lost", "monotonic_ns": snap["monotonic_ns"], "snapshot": snap})
            first_invalid = first(evidence.adapter_diag, loss_start, lambda x: x["message"] == "INPUT_AUTHORITY_INVALID")
            zero = first(evidence.mock, final_gate["monotonic_ns"], lambda x: array_matches(x, ZERO_ARRAY))
            if not first_invalid or not zero: raise RuntimeError("gate loss was not detected and zeroed")
            latency = (zero["monotonic_ns"] - final_gate["monotonic_ns"]) / 1e9
            if latency > .35: raise RuntimeError(f"gate loss zero latency {latency}")
            timing["publisher_loss_detection_ns"] = first_invalid["monotonic_ns"]
            timing["first_adapter_deadman_zero_ns"] = zero["monotonic_ns"]
            timing["gate_loss_final_receipt_to_zero_sec"] = latency
            post = [x for x in evidence.mock if x["monotonic_ns"] >= zero["monotonic_ns"]]
            if any(not array_matches(x, ZERO_ARRAY) for x in post): raise RuntimeError("mock nonzero after gate loss")
            windows.append(rate_window(post, zero["monotonic_ns"], lambda x: array_matches(x, ZERO_ARRAY), 2.0, "gate_loss_mock_zero"))
            statuses.append("P4D3_GATE_LOSS_ADAPTER_DEADMAN_PASS")
            persist(out_dir, evidence, fixture, graphs, authority, events, arm_events, metrics, start_ns); return metrics

        fixture.collision_enabled = False
        final_collision = fixture.last_collision_publish_ns; timing["final_collision_valid_receipt_ns"] = final_collision
        events.append({"event": "collision_permission_stopped", "monotonic_ns": time.monotonic_ns()})
        spin(executor, 2.8)
        fault = first(evidence.gate_state, final_collision, lambda x: x["values"].get("state") == "FAULT")
        gate_zero = first(evidence.gate, final_collision, lambda x: gate_matches(x, 0, 0))
        mock_zero = first(evidence.mock, gate_zero["monotonic_ns"] if gate_zero else final_collision, lambda x: array_matches(x, ZERO_ARRAY))
        if not fault or fault["message"] != "COLLISION_MONITOR_VALID_INVALID" or fault["values"].get("fault_latched") != "true":
            raise RuntimeError(f"collision fault contract failed: {fault}")
        if not gate_zero or not mock_zero: raise RuntimeError("fault zero propagation absent")
        if (mock_zero["monotonic_ns"] - gate_zero["monotonic_ns"]) / 1e9 > .10: raise RuntimeError("gate-to-mock fault zero exceeded .10 s")
        timing.update({"gate_fault_ns": fault["monotonic_ns"], "first_gate_zero_ns": gate_zero["monotonic_ns"],
                       "first_mock_zero_ns": mock_zero["monotonic_ns"],
                       "last_nonzero_gate_ns": last(evidence.gate, gate_zero["monotonic_ns"], lambda x: not gate_matches(x,0,0))["monotonic_ns"],
                       "last_nonzero_mock_ns": last(evidence.mock, mock_zero["monotonic_ns"], lambda x: not array_matches(x,ZERO_ARRAY))["monotonic_ns"]})
        timing["gate_fault_to_gate_zero_sec"] = (gate_zero["monotonic_ns"] - fault["monotonic_ns"]) / 1e9
        timing["gate_zero_to_mock_zero_sec"] = (mock_zero["monotonic_ns"] - gate_zero["monotonic_ns"]) / 1e9
        timing["final_permission_to_mock_zero_sec"] = (mock_zero["monotonic_ns"] - final_collision) / 1e9
        post = [x for x in evidence.mock if x["monotonic_ns"] >= mock_zero["monotonic_ns"]]
        if any(not array_matches(x, ZERO_ARRAY) for x in post): raise RuntimeError("mock nonzero after gate fault")
        windows.append(rate_window(evidence.gate, gate_zero["monotonic_ns"], lambda x: gate_matches(x,0,0), 2.0, "fault_gate_zero"))
        windows.append(rate_window(evidence.mock, mock_zero["monotonic_ns"], lambda x: array_matches(x,ZERO_ARRAY), 2.0, "fault_mock_zero"))
        statuses.append("P4D3_GATE_FAULT_TO_MOCK_ZERO_PASS")

        fixture.collision_enabled = True; recovery_permission_ns = time.monotonic_ns(); spin(executor, 1.0)
        if any(x["linear_x"] != 0.0 for x in evidence.gate if x["monotonic_ns"] >= recovery_permission_ns):
            raise RuntimeError("permission recovery restored motion without clear")
        clear = call_arm(executor, client, False, arm_events); timing["fault_clear_request_ns"] = clear["monotonic_ns"]
        if not clear["success"]: raise RuntimeError(f"fault clear failed: {clear}")
        spin(executor, 1.25)
        rearm = call_arm(executor, client, True, arm_events); timing["rearm_request_ns"] = rearm["monotonic_ns"]
        if not rearm["success"]: raise RuntimeError(f"rearm failed: {rearm}")
        spin(executor, 2.6)
        recovered_gate = first(evidence.gate, rearm["monotonic_ns"], lambda x: gate_matches(x,.1,0))
        recovered_mock = first(evidence.mock, rearm["monotonic_ns"], lambda x: array_matches(x,STRAIGHT_ARRAY))
        if not recovered_gate or not recovered_mock: raise RuntimeError("explicit recovery absent")
        timing["first_recovered_gate_nonzero_ns"] = recovered_gate["monotonic_ns"]
        timing["first_recovered_mock_nonzero_ns"] = recovered_mock["monotonic_ns"]
        windows.append(rate_window(evidence.mock, recovered_mock["monotonic_ns"], lambda x: array_matches(x,STRAIGHT_ARRAY), 2.0, "recovered_mock"))
        statuses.append("P4D3_EXPLICIT_RECOVERY_AND_REARM_PASS")
        snap = probe.snapshot(); require_authority(snap); graphs.append({"label":"final", "monotonic_ns":snap["monotonic_ns"], "snapshot":snap})
        persist(out_dir, evidence, fixture, graphs, authority, events, arm_events, metrics, start_ns); return metrics
    finally:
        for node in reversed(nodes):
            try: executor.remove_node(node); node.destroy_node()
            except Exception: pass
        if rclpy.ok(): rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("scenario", choices=["readiness", "full", "gate_loss"])
    parser.add_argument("--out-dir", type=Path, required=True); args = parser.parse_args()
    print(json.dumps(run(args.scenario, args.out_dir), indent=2, sort_keys=True))


if __name__ == "__main__": main()
