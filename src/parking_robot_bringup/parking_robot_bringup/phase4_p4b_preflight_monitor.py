"""Fresh-source Phase 4 P4-B Collision Monitor preflight monitor."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter


@dataclass
class CommandSample:
    monotonic_ns: int
    ros_ns: int
    topic: str
    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float

    @property
    def finite(self) -> bool:
        return all(math.isfinite(value) for value in self.values)

    @property
    def nonzero(self) -> bool:
        return any(abs(value) > 1.0e-6 for value in self.values)

    @property
    def values(self) -> tuple[float, float, float, float, float, float]:
        return (
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        )


class Phase4P4BPreflightMonitor(Node):
    """Collect raw and safe Twist samples while driving synthetic modes."""

    def __init__(self, out_dir: Path) -> None:
        super().__init__("phase4_p4b_preflight_monitor")
        self.out_dir = out_dir
        self.raw: list[CommandSample] = []
        self.safe: list[CommandSample] = []
        self.events: list[dict] = []
        self._raw_sub = self.create_subscription(Twist, "/cmd_vel_nav_raw", self._raw_cb, 50)
        self._safe_sub = self.create_subscription(Twist, "/cmd_vel_nav_safe", self._safe_cb, 50)
        self._param_client = self.create_client(
            SetParameters, "/phase4_p4b_synthetic_obstacles/set_parameters"
        )

    def _sample(self, topic: str, msg: Twist) -> CommandSample:
        return CommandSample(
            monotonic_ns=time.monotonic_ns(),
            ros_ns=self.get_clock().now().nanoseconds,
            topic=topic,
            linear_x=float(msg.linear.x),
            linear_y=float(msg.linear.y),
            linear_z=float(msg.linear.z),
            angular_x=float(msg.angular.x),
            angular_y=float(msg.angular.y),
            angular_z=float(msg.angular.z),
        )

    def _raw_cb(self, msg: Twist) -> None:
        self.raw.append(self._sample("/cmd_vel_nav_raw", msg))

    def _safe_cb(self, msg: Twist) -> None:
        self.safe.append(self._sample("/cmd_vel_nav_safe", msg))

    def set_mode(self, mode: str) -> None:
        if not self._param_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Synthetic obstacle parameter service unavailable")
        request = SetParameters.Request()
        request.parameters = [Parameter("mode", Parameter.Type.STRING, mode).to_parameter_msg()]
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Timed out setting synthetic mode {mode}")
        result = future.result().results[0]
        if not result.successful:
            raise RuntimeError(f"Failed to set synthetic mode {mode}: {result.reason}")
        self.events.append({"monotonic_ns": time.monotonic_ns(), "mode": mode})

    def spin_for(self, seconds: float) -> None:
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)

    def write_timelines(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._write_tsv(self.out_dir / "raw_cmd_timeline.tsv", self.raw)
        self._write_tsv(self.out_dir / "safe_cmd_timeline.tsv", self.safe)
        with (self.out_dir / "obstacle_mode_events.jsonl").open("w", encoding="utf-8") as handle:
            for event in self.events:
                handle.write(json.dumps(event, sort_keys=True) + "\n")

    def _write_tsv(self, path: Path, samples: list[CommandSample]) -> None:
        fields = [
            "monotonic_ns",
            "ros_ns",
            "topic",
            "linear_x",
            "linear_y",
            "linear_z",
            "angular_x",
            "angular_y",
            "angular_z",
            "finite",
            "nonzero",
        ]
        with path.open("w", encoding="utf-8") as handle:
            handle.write("\t".join(fields) + "\n")
            for sample in samples:
                row = asdict(sample)
                row["finite"] = sample.finite
                row["nonzero"] = sample.nonzero
                handle.write("\t".join(str(row[field]) for field in fields) + "\n")

    def metrics(self, source: str) -> dict:
        mode_windows: dict[str, tuple[int, int]] = {}
        for index, event in enumerate(self.events):
            start = int(event["monotonic_ns"])
            end = int(self.events[index + 1]["monotonic_ns"]) if index + 1 < len(self.events) else time.monotonic_ns()
            mode_windows[str(event["mode"])] = (start, end)

        def samples_for(samples: list[CommandSample], mode: str) -> list[CommandSample]:
            start, end = mode_windows[mode]
            return [sample for sample in samples if start <= sample.monotonic_ns < end]

        clear_raw = [s for s in samples_for(self.raw, "CLEAR") if s.nonzero]
        clear_safe = [s for s in samples_for(self.safe, "CLEAR") if s.nonzero]
        slow_raw = []
        slow_safe = []
        if "SLOW" in mode_windows:
            slow_raw = [s for s in samples_for(self.raw, "SLOW") if abs(s.linear_x) > 1.0e-6]
            slow_safe = [s for s in samples_for(self.safe, "SLOW") if abs(s.linear_x) > 1.0e-6]
        stop_safe = samples_for(self.safe, "STOP")
        recovery_safe = [s for s in samples_for(self.safe, "CLEAR_RECOVERY") if s.nonzero]

        slowdown_ratio = None
        if slow_raw and slow_safe:
            slowdown_ratio = statistics.median(abs(s.linear_x) for s in slow_safe) / statistics.median(
                abs(s.linear_x) for s in slow_raw
            )

        stop_event_ns = mode_windows["STOP"][0]
        first_stop_zero_ns = next((s.monotonic_ns for s in stop_safe if not s.nonzero), None)
        recovery_event_ns = mode_windows["CLEAR_RECOVERY"][0]
        first_recovery_ns = next((s.monotonic_ns for s in recovery_safe), None)

        return {
            "source": source,
            "raw_count": len(self.raw),
            "safe_count": len(self.safe),
            "clear_raw_nonzero_count": len(clear_raw),
            "clear_safe_nonzero_count": len(clear_safe),
            "slowdown_ratio": slowdown_ratio,
            "stop_latency_sec": None if first_stop_zero_ns is None else (first_stop_zero_ns - stop_event_ns) / 1.0e9,
            "recovery_latency_sec": None
            if first_recovery_ns is None
            else (first_recovery_ns - recovery_event_ns) / 1.0e9,
            "stop_safe_zero_count": sum(1 for s in stop_safe if not s.nonzero),
            "all_raw_finite": all(s.finite for s in self.raw),
            "all_safe_finite": all(s.finite for s in self.safe),
        }


def run_preflight(out_dir: Path, source: str) -> dict:
    rclpy.init()
    node = Phase4P4BPreflightMonitor(out_dir)
    try:
        node.spin_for(2.0)
        sequence = ("CLEAR", "STOP", "CLEAR") if source == "points" else ("CLEAR", "SLOW", "STOP", "CLEAR")
        for mode in sequence:
            recorded_mode = "CLEAR_RECOVERY" if mode == "CLEAR" and node.events else mode
            node.set_mode(mode)
            node.events[-1]["mode"] = recorded_mode
            node.spin_for(2.6)
        node.write_timelines()
        metrics = node.metrics(source)
        with (out_dir / "terminal_metrics.json").open("w", encoding="utf-8") as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True)
        return metrics
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--source", required=True, choices=["scan", "points"])
    args = parser.parse_args(argv)
    print(json.dumps(run_preflight(args.out_dir, args.source), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
