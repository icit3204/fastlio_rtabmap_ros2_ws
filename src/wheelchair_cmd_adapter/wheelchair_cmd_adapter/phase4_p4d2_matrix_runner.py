from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import Float32MultiArray

from wheelchair_cmd_adapter.mock_wheelchair_cmd_adapter import MockWheelchairCmdAdapter


INPUT = "/vehicle_cmd_safe"
OUTPUT = "/wheelchair_control_command_mock"
DIAGNOSTICS = "/wheelchair_cmd_adapter/diagnostics"
FORBIDDEN = [
    "/wheelchair_control_command", "/wheelchair_control_command_raw",
    "/cmd_vel_nav_raw", "/cmd_vel_nav_safe", "/cmd_vel",
]
STRAIGHT = [10000.0, 100.0, 0.0]
ZERO = [0.0, 0.0, 0.0]


class Fixture(Node):
    def __init__(self, name: str = "p4d2_fixture"):
        super().__init__(name)
        self.pub = self.create_publisher(TwistStamped, INPUT, 10)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self._tick, clock=self.clock)
        self.enabled = True
        self.values = {"frame": "base_footprint", "lx": 0.1, "ly": 0.0, "lz": 0.0,
                       "ax": 0.0, "ay": 0.0, "az": 0.0}
        self.last_publish = None

    def _tick(self):
        if not self.enabled:
            return
        msg = TwistStamped()
        msg.header.frame_id = self.values["frame"]
        msg.twist.linear.x = self.values["lx"]
        msg.twist.linear.y = self.values["ly"]
        msg.twist.linear.z = self.values["lz"]
        msg.twist.angular.x = self.values["ax"]
        msg.twist.angular.y = self.values["ay"]
        msg.twist.angular.z = self.values["az"]
        self.pub.publish(msg)
        self.last_publish = time.monotonic()


class RogueOutput(Node):
    def __init__(self):
        super().__init__("p4d2_rogue_mock_output")
        self.pub = self.create_publisher(Float32MultiArray, OUTPUT, 10)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self._tick, clock=self.clock)

    def _tick(self):
        msg = Float32MultiArray()
        msg.data = [4242.0, 313.0, 99.0]
        self.pub.publish(msg)


class ClockFixture(Node):
    def __init__(self):
        super().__init__("p4d2_clock_fixture", parameter_overrides=[Parameter("use_sim_time", value=False)])
        self.pub = self.create_publisher(ClockMsg, "/clock", 10)
        msg = ClockMsg()
        msg.clock.sec = 1234
        msg.clock.nanosec = 567000000
        self.pub.publish(msg)


class Monitor(Node):
    def __init__(self):
        super().__init__("p4d2_evidence_monitor", parameter_overrides=[Parameter("use_sim_time", value=False)])
        self.outputs = []
        self.diagnostics = []
        self.create_subscription(Float32MultiArray, OUTPUT, self._out, 200)
        self.create_subscription(DiagnosticArray, DIAGNOSTICS, self._diag, 200)

    def _out(self, msg):
        self.outputs.append({"t": time.monotonic(), "array": [float(x) for x in msg.data]})

    def _diag(self, msg):
        for status in msg.status:
            if status.name == "mock_wheelchair_cmd_adapter":
                self.diagnostics.append({"t": time.monotonic(), "reason": status.message,
                                         "values": {v.key: v.value for v in status.values}})


class Probe(Node):
    def __init__(self):
        super().__init__("p4d2_graph_probe", parameter_overrides=[Parameter("use_sim_time", value=False)])

    def snapshot(self):
        topics = {n: t for n, t in self.get_topic_names_and_types()}
        return {"input": self._info(INPUT), "output": self._info(OUTPUT),
                "forbidden": {x: topics.get(x, []) for x in FORBIDDEN}}

    def _info(self, topic):
        return [{"node_name": x.node_name, "node_namespace": x.node_namespace,
                 "topic_type": x.topic_type, "gid": list(x.endpoint_gid)}
                for x in self.get_publishers_info_by_topic(topic)]


def spin(executor, seconds, predicate=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if predicate and predicate():
            return True
    return bool(predicate and predicate())


def samples_between(samples, start, end=None):
    end = end if end is not None else math.inf
    return [x for x in samples if start <= x["t"] <= end]


def window(samples, expected, minimum, label):
    matches = [x for x in samples if x["array"] == expected]
    if len(matches) < 2:
        raise RuntimeError(f"{label}: insufficient {expected} samples")
    duration = matches[-1]["t"] - matches[0]["t"]
    hz = (len(matches) - 1) / duration
    if duration < minimum or not 18.0 <= hz <= 22.0:
        raise RuntimeError(f"{label}: duration={duration:.6f}, rate={hz:.6f}")
    return {"label": label, "count": len(matches), "duration_sec": duration, "rate_hz": hz,
            "start": matches[0]["t"], "end": matches[-1]["t"]}


def first_after(samples, start, expected):
    return next((x for x in samples if x["t"] >= start and x["array"] == expected), None)


INVALID = {
    "empty_frame": ({"frame": ""}, "FRAME_INVALID"),
    "wrong_frame": ({"frame": "map"}, "FRAME_INVALID"),
    "nonfinite": ({"lx": float("nan")}, "NUMERICAL_INVALID"),
    "unsupported_axis": ({"ly": 0.01}, "UNSUPPORTED_AXES"),
    "forward_over_limit": ({"lx": 0.200001}, "OVER_LIMIT"),
    "angular_over_limit": ({"az": 0.500001}, "OVER_LIMIT"),
    "reverse": ({"lx": -0.01}, "REVERSE_UNSUPPORTED"),
    "in_place": ({"lx": 0.0, "az": 0.1}, "IN_PLACE_ROTATION_UNSUPPORTED"),
    "tight_radius": ({"lx": 0.1, "az": 0.2}, "TURN_RADIUS_UNSUPPORTED"),
}


def run(scenario: str, out_dir: Path, sim_time: bool):
    out_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    adapter = MockWheelchairCmdAdapter()
    if sim_time:
        adapter.set_parameters([Parameter("use_sim_time", value=True)])
    fixture = Fixture()
    monitor = Monitor()
    probe = Probe()
    nodes = [adapter, fixture, monitor, probe]
    if sim_time:
        nodes.append(ClockFixture())
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    result = {"scenario": scenario, "sim_time": sim_time, "status": "PASS", "windows": [],
              "graphs": {}, "latencies": {}, "diagnostic_history": []}
    try:
        spin(executor, 1.2)
        result["graphs"]["initial"] = probe.snapshot()
        if scenario == "missing_input":
            fixture.enabled = False
            fixture.destroy_publisher(fixture.pub)
            spin(executor, 0.5)
            start = time.monotonic()
            spin(executor, 2.3)
            result["windows"].append(window(samples_between(monitor.outputs, start), ZERO, 2.0, scenario))
            expected_reason = "INPUT_AUTHORITY_INVALID"
        else:
            spin(executor, 1.0)
            baseline = samples_between(monitor.outputs, time.monotonic() - 0.8)
            window(baseline, STRAIGHT, 0.5, "baseline")
            expected_reason = None

        if scenario in INVALID:
            changes, expected_reason = INVALID[scenario]
            fixture.values.update(changes)
            transition = time.monotonic()
            spin(executor, 2.35)
            first = first_after(monitor.outputs, transition, ZERO)
            if first is None or first["t"] - transition > 0.10:
                raise RuntimeError(f"invalid zero latency {None if first is None else first['t'] - transition}")
            post = samples_between(monitor.outputs, first["t"])
            if any(x["array"] != ZERO for x in post):
                raise RuntimeError("adapter emitted nonzero after invalid-state zero")
            result["latencies"]["invalid_to_zero_sec"] = first["t"] - transition
            result["windows"].append(window(post, ZERO, 2.0, scenario))
        elif scenario == "input_duplicate" or scenario == "frozen_authority":
            rogue = Fixture("p4d2_rogue_input")
            nodes.append(rogue); executor.add_node(rogue)
            spin(executor, 0.25)
            graph_visible = time.monotonic(); result["graphs"]["conflict"] = probe.snapshot()
            if len(result["graphs"]["conflict"]["input"]) != 2:
                raise RuntimeError("duplicate input not graph-visible")
            spin(executor, 2.3)
            first = first_after(monitor.outputs, graph_visible, ZERO)
            if first is None or first["t"] - graph_visible > 0.75:
                raise RuntimeError("input conflict detection exceeded 0.75 s")
            result["latencies"]["graph_conflict_to_zero_sec"] = first["t"] - graph_visible
            result["windows"].append(window(samples_between(monitor.outputs, first["t"]), ZERO, 2.0, scenario))
            expected_reason = "INPUT_AUTHORITY_INVALID"
            executor.remove_node(rogue); rogue.destroy_node(); nodes.remove(rogue)
            spin(executor, 0.5); fixture.values = {"frame": "base_footprint", "lx": 0.1, "ly": 0.0, "lz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0}
            recovery = time.monotonic(); spin(executor, 1.2)
            window(samples_between(monitor.outputs, recovery), STRAIGHT, 0.7, "automatic_recovery")
            result["graphs"]["recovered"] = probe.snapshot()
        elif scenario == "output_duplicate":
            rogue = RogueOutput(); nodes.append(rogue); executor.add_node(rogue)
            spin(executor, 0.25); graph_visible = time.monotonic(); result["graphs"]["conflict"] = probe.snapshot()
            if len(result["graphs"]["conflict"]["output"]) != 2:
                raise RuntimeError("duplicate output not graph-visible")
            spin(executor, 2.3)
            adapter_zero = next((d for d in monitor.diagnostics if d["t"] >= graph_visible and d["reason"] == "OUTPUT_AUTHORITY_INVALID"), None)
            if adapter_zero is None or adapter_zero["t"] - graph_visible > 0.75:
                raise RuntimeError("output conflict detection exceeded 0.75 s")
            owned = [x for x in samples_between(monitor.outputs, adapter_zero["t"]) if x["array"] != [4242.0, 313.0, 99.0]]
            if any(x["array"] != ZERO for x in owned):
                raise RuntimeError("adapter-owned nonzero after output conflict")
            result["latencies"]["graph_conflict_to_zero_sec"] = adapter_zero["t"] - graph_visible
            result["windows"].append(window(owned, ZERO, 2.0, scenario))
            result["aggregate_topic_safe"] = False
            expected_reason = "OUTPUT_AUTHORITY_INVALID"
            executor.remove_node(rogue); rogue.destroy_node(); nodes.remove(rogue)
            spin(executor, 0.5); recovery = time.monotonic(); spin(executor, 1.2)
            window(samples_between(monitor.outputs, recovery), STRAIGHT, 0.7, "automatic_recovery")
            result["graphs"]["recovered"] = probe.snapshot()
        elif scenario == "stale" or scenario == "frozen_deadman":
            fixture.enabled = False
            final_receipt_ns = adapter._latest_receipt_ns
            final_publish = fixture.last_publish
            spin(executor, 2.65)
            first = first_after(monitor.outputs, final_publish, ZERO)
            if first is None or first["t"] - final_publish > 0.35:
                raise RuntimeError("deadman exceeded 0.35 s from final receipt")
            result["latencies"]["final_receipt_to_zero_sec"] = first["t"] - final_publish
            stale_diag = next(x for x in monitor.diagnostics if x["t"] >= final_publish and x["reason"] == "INPUT_STALE")
            result["latencies"]["core_input_age_sec"] = float(stale_diag["values"]["input_age_sec"])
            result["latencies"]["final_receipt_steady_ns"] = final_receipt_ns
            result["windows"].append(window(samples_between(monitor.outputs, first["t"]), ZERO, 2.0, scenario))
            expected_reason = "INPUT_STALE"
            fixture.enabled = True; recovery = time.monotonic(); spin(executor, 2.3)
            result["windows"].append(window(samples_between(monitor.outputs, recovery), STRAIGHT, 2.0, "recovered_valid"))
        elif scenario == "frozen_liveness":
            start = time.monotonic(); spin(executor, 1.3)
            result["windows"].append(window(samples_between(monitor.outputs, start), STRAIGHT, 1.0, scenario))
            expected_reason = "VALID"

        result["graphs"]["final"] = probe.snapshot()
        result["diagnostic_history"] = monitor.diagnostics
        if expected_reason and not any(x["reason"] == expected_reason for x in monitor.diagnostics):
            raise RuntimeError(f"missing diagnostic reason {expected_reason}")
        present = {k: v for k, v in result["graphs"]["final"]["forbidden"].items() if v}
        if present:
            raise RuntimeError(f"forbidden topics present: {present}")
        with (out_dir / "output.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["t", "array"]); writer.writeheader(); writer.writerows(monitor.outputs)
        (out_dir / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        return result
    finally:
        for node in reversed(nodes):
            try:
                executor.remove_node(node); node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=list(INVALID) + ["missing_input", "input_duplicate", "output_duplicate", "stale",
                                                          "frozen_liveness", "frozen_deadman", "frozen_authority"])
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.scenario, args.out_dir, args.scenario.startswith("frozen_")), indent=2))


if __name__ == "__main__":
    main()
