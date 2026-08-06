from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import time
from typing import Callable

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray

from wheelchair_cmd_adapter.mock_wheelchair_cmd_adapter import MockWheelchairCmdAdapter


INPUT_TOPIC = "/vehicle_cmd_safe"
OUTPUT_TOPIC = "/wheelchair_control_command_mock"
FORBIDDEN_TOPICS = [
    "/wheelchair_control_command",
    "/wheelchair_control_command_raw",
    "/cmd_vel_nav_raw",
    "/cmd_vel_nav_safe",
]


class SyntheticFixture(Node):
    def __init__(self):
        super().__init__("phase4_p4d1_synthetic_vehicle_cmd_fixture")
        self.pub = self.create_publisher(TwistStamped, INPUT_TOPIC, 10)
        self.current: tuple[float, float] | None = None
        self.timer = self.create_timer(0.05, self._timer_cb)
        self.last_publish_ns: int | None = None

    def set_command(self, v: float | None, w: float | None) -> None:
        self.current = None if v is None or w is None else (float(v), float(w))

    def _timer_cb(self) -> None:
        if self.current is None:
            return
        msg = TwistStamped()
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.current[0]
        msg.twist.angular.z = self.current[1]
        self.pub.publish(msg)
        self.last_publish_ns = self.get_clock().now().nanoseconds


class EvidenceMonitor(Node):
    def __init__(self):
        super().__init__("phase4_p4d1_mock_evidence_monitor")
        self.samples: list[dict] = []
        qos = QoSProfile(depth=200)
        self.create_subscription(Float32MultiArray, OUTPUT_TOPIC, self._output_cb, qos)

    def _output_cb(self, msg: Float32MultiArray) -> None:
        self.samples.append({
            "t": time.monotonic(),
            "array": [float(x) for x in msg.data],
        })


class GraphProbe(Node):
    def __init__(self):
        super().__init__("phase4_p4d1_graph_probe")

    def snapshot(self) -> dict:
        topics = {name: types for name, types in self.get_topic_names_and_types()}
        return {
            "input_publishers": self._infos(INPUT_TOPIC),
            "mock_output_publishers": self._infos(OUTPUT_TOPIC),
            "forbidden_topics": {topic: topics.get(topic, []) for topic in FORBIDDEN_TOPICS},
        }

    def _infos(self, topic: str) -> list[dict]:
        return [
            {
                "node_name": info.node_name,
                "node_namespace": info.node_namespace,
                "topic_type": info.topic_type,
                "gid": list(info.endpoint_gid),
            }
            for info in self.get_publishers_info_by_topic(topic)
        ]


def spin_until(executor: SingleThreadedExecutor, deadline: float, predicate: Callable[[], bool] | None = None) -> bool:
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate is not None and predicate():
            return True
    return predicate() if predicate is not None else True


def select_samples(samples: list[dict], start: float, end: float) -> list[dict]:
    return [s for s in samples if start <= s["t"] <= end]


def rate_hz(samples: list[dict]) -> float:
    if len(samples) < 2:
        return 0.0
    duration = samples[-1]["t"] - samples[0]["t"]
    if duration <= 0.0:
        return 0.0
    return (len(samples) - 1) / duration


def require_window(samples: list[dict], expected: list[float], min_duration: float, label: str) -> dict:
    if not samples:
        raise RuntimeError(f"{label}: no samples")
    matching = [s for s in samples if s["array"] == expected]
    if len(matching) < 2:
        raise RuntimeError(f"{label}: insufficient matching samples for {expected}")
    duration = matching[-1]["t"] - matching[0]["t"]
    hz = rate_hz(matching)
    if duration < min_duration:
        raise RuntimeError(f"{label}: matching duration {duration:.3f}s < {min_duration:.3f}s")
    if not 18.0 <= hz <= 22.0:
        raise RuntimeError(f"{label}: heartbeat {hz:.3f} Hz outside 18-22")
    return {"label": label, "expected": expected, "count": len(matching), "duration_sec": duration, "rate_hz": hz}


def run(out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    adapter = MockWheelchairCmdAdapter()
    fixture = SyntheticFixture()
    monitor = EvidenceMonitor()
    graph = GraphProbe()
    executor = SingleThreadedExecutor()
    for node in [adapter, fixture, monitor, graph]:
        executor.add_node(node)

    timeline: list[dict] = []
    sample_csv = out_dir / "mock_output_samples.csv"
    result = {
        "status": "P4D1_MOCK_ADAPTER_BASIC_PREFLIGHT_PASS",
        "timeline": timeline,
        "authority": {},
        "forbidden_topics": {},
        "stale": {},
        "windows": [],
    }
    try:
        spin_until(executor, time.monotonic() + 1.0)
        graph_start = graph.snapshot()
        result["authority"]["startup"] = graph_start

        fixture.set_command(None, None)
        t0 = time.monotonic()
        spin_until(executor, t0 + 2.4)
        result["windows"].append(require_window(select_samples(monitor.samples, t0, time.monotonic()), [0.0, 0.0, 0.0], 2.0, "startup_zero"))
        timeline.append({"step": "startup_silent", "start": t0, "end": time.monotonic()})

        scenarios = [
            ("straight", 0.10, 0.0, [10000.0, 100.0, 0.0], 2.0),
            ("left_curve", 0.10, 0.05, [-2000.0, 100.0, 0.0], 0.8),
            ("right_curve", 0.10, -0.05, [2000.0, 100.0, 0.0], 0.8),
            ("explicit_zero", 0.0, 0.0, [0.0, 0.0, 0.0], 0.8),
        ]
        for label, v, w, expected, min_duration in scenarios:
            fixture.set_command(v, w)
            start = time.monotonic()
            spin_until(executor, start + max(2.35, min_duration + 0.35))
            result["windows"].append(require_window(select_samples(monitor.samples, start, time.monotonic()), expected, min_duration, label))
            timeline.append({"step": label, "start": start, "end": time.monotonic(), "command": [v, w], "expected": expected})

        fixture.set_command(0.10, 0.0)
        restore_start = time.monotonic()
        spin_until(executor, restore_start + 1.0)
        fixture.set_command(None, None)
        final_input_receipt = adapter._latest_receipt_ns
        stop_start = time.monotonic()

        first_zero_time: float | None = None
        core_detection_latency = None
        evidence_latency = None

        def stale_observed() -> bool:
            nonlocal first_zero_time, core_detection_latency, evidence_latency
            now = time.monotonic()
            if core_detection_latency is None and adapter._last_result is not None and adapter._last_result.reason.value == "INPUT_STALE":
                if final_input_receipt is not None:
                    age_sec = (adapter._receipt_clock.now().nanoseconds - final_input_receipt) / 1e9
                    core_detection_latency = age_sec
            for sample in monitor.samples:
                if sample["t"] >= stop_start and sample["array"] == [0.0, 0.0, 0.0]:
                    first_zero_time = sample["t"]
                    evidence_latency = sample["t"] - stop_start
                    return True
            return now - stop_start > 0.35

        spin_until(executor, stop_start + 0.35, stale_observed)
        if first_zero_time is None:
            raise RuntimeError("stale: no mock zero within 0.35 s after final input publication stopped")
        spin_until(executor, time.monotonic() + 2.2)
        stale_samples = select_samples(monitor.samples, first_zero_time, time.monotonic())
        result["windows"].append(require_window(stale_samples, [0.0, 0.0, 0.0], 2.0, "stale_zero"))
        result["stale"] = {
            "stop_start_monotonic": stop_start,
            "first_zero_monotonic": first_zero_time,
            "evidence_callback_latency_sec": evidence_latency,
            "core_detection_latency_sec": core_detection_latency,
            "required_max_zero_latency_sec": 0.35,
            "configured_deadman_sec": 0.25,
        }
        timeline.append({"step": "stale_deadman", "start": stop_start, "end": time.monotonic()})

        final_graph = graph.snapshot()
        result["authority"]["final"] = final_graph
        result["forbidden_topics"] = final_graph["forbidden_topics"]
        if len(final_graph["input_publishers"]) != 1:
            raise RuntimeError(f"expected one input publisher, got {final_graph['input_publishers']}")
        if len(final_graph["mock_output_publishers"]) != 1:
            raise RuntimeError(f"expected one mock output publisher, got {final_graph['mock_output_publishers']}")
        present_forbidden = {k: v for k, v in final_graph["forbidden_topics"].items() if v}
        if present_forbidden:
            raise RuntimeError(f"forbidden topics present: {present_forbidden}")

        with sample_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["t", "array"])
            writer.writeheader()
            writer.writerows(monitor.samples)
        result["sample_csv"] = str(sample_csv)
        (out_dir / "preflight_result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        return result
    finally:
        for node in [adapter, fixture, monitor, graph]:
            executor.remove_node(node)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    result = run(args.out_dir)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
