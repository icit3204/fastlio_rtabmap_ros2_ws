"""Standalone evidence monitor for P4-C runtime scenarios."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


def _diagnostic_level(value) -> int:
    if isinstance(value, memoryview):
        value = value.tobytes()
    if isinstance(value, bytearray):
        value = bytes(value)
    if isinstance(value, bytes):
        if len(value) != 1:
            raise ValueError(f"diagnostic level byte length must be 1, got {len(value)}")
        return int.from_bytes(value, "little")
    return int(value)


def _classify(values) -> tuple[str, str]:
    finite = "finite" if all(math.isfinite(value) for value in values) else "nonfinite"
    zero = "zero" if all(abs(value) <= 1e-9 for value in values) else "nonzero"
    return finite, zero


class P4CEvidenceMonitor(Node):
    def __init__(self) -> None:
        super().__init__("phase4_p4c_evidence_monitor")
        self.declare_parameter("output_dir", "/tmp/p4c_evidence")
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.files = {
            "safe": (self.output_dir / "safe_input_timeline.tsv").open("w"),
            "vehicle": (self.output_dir / "vehicle_output_timeline.tsv").open("w"),
            "permission": (self.output_dir / "permission_timeline.tsv").open("w"),
            "collision_valid": (self.output_dir / "collision_valid_timeline.tsv").open("w"),
            "state": (self.output_dir / "gate_state_events.jsonl").open("w"),
            "diag": (self.output_dir / "diagnostics.jsonl").open("w"),
            "heartbeat": (self.output_dir / "monitor_heartbeat.tsv").open("w"),
        }
        self.counts = {"safe": 0, "vehicle": 0, "loc": 0, "ctrl": 0, "collision": 0, "state": 0}
        self.create_subscription(Twist, "/cmd_vel_nav_safe", self._safe_cb, 10)
        self.create_subscription(TwistStamped, "/vehicle_cmd_safe", self._vehicle_cb, 10)
        self.create_subscription(Bool, "/system/localization_valid", lambda m: self._perm_cb("localization", m), 10)
        self.create_subscription(Bool, "/system/controller_valid", lambda m: self._perm_cb("controller", m), 10)
        self.create_subscription(Bool, "/system/collision_monitor_valid", self._collision_cb, 10)
        self.create_subscription(DiagnosticStatus, "/vehicle_cmd_safety/state", self._state_cb, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diag_cb, 10)
        self.create_timer(0.1, self._heartbeat_cb)

    def _mono_ns(self) -> int:
        return time.monotonic_ns()

    def _safe_cb(self, msg: Twist) -> None:
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        finite, zero = _classify(values)
        self.files["safe"].write(f"{self._mono_ns()}\t{','.join(str(v) for v in values)}\t{finite}\t{zero}\n")
        self.files["safe"].flush()
        self.counts["safe"] += 1

    def _vehicle_cb(self, msg: TwistStamped) -> None:
        values = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        ]
        finite, zero = _classify(values)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1_000_000_000.0
        self.files["vehicle"].write(f"{self._mono_ns()}\t{stamp:.9f}\t{msg.header.frame_id}\t{','.join(str(v) for v in values)}\t{finite}\t{zero}\n")
        self.files["vehicle"].flush()
        self.counts["vehicle"] += 1

    def _perm_cb(self, name: str, msg: Bool) -> None:
        self.files["permission"].write(f"{self._mono_ns()}\t{name}\t{str(msg.data).lower()}\n")
        self.files["permission"].flush()
        self.counts["loc" if name == "localization" else "ctrl"] += 1

    def _collision_cb(self, msg: Bool) -> None:
        self.files["collision_valid"].write(f"{self._mono_ns()}\t{str(msg.data).lower()}\n")
        self.files["collision_valid"].flush()
        self.counts["collision"] += 1

    def _state_cb(self, msg: DiagnosticStatus) -> None:
        payload = {
            "monotonic_ns": self._mono_ns(),
            "level": int(msg.level),
            "message": msg.message,
            "values": {item.key: item.value for item in msg.values},
        }
        self.files["state"].write(json.dumps(payload, sort_keys=True) + "\n")
        self.files["state"].flush()
        self.counts["state"] += 1

    def _diag_cb(self, msg: DiagnosticArray) -> None:
        statuses = []
        for status in msg.status:
            row = {"name": status.name, "message": status.message}
            try:
                row["level"] = _diagnostic_level(status.level)
            except (TypeError, ValueError) as exc:
                row["level"] = None
                row["level_error"] = str(exc)
            statuses.append(row)
        payload = {"monotonic_ns": self._mono_ns(), "status": statuses}
        self.files["diag"].write(json.dumps(payload, sort_keys=True) + "\n")
        self.files["diag"].flush()

    def _heartbeat_cb(self) -> None:
        self.files["heartbeat"].write(
            f"{self._mono_ns()}\t{self.counts['safe']}\t{self.counts['vehicle']}\t{self.counts['loc']}\t{self.counts['ctrl']}\t{self.counts['collision']}\t{self.counts['state']}\n"
        )
        self.files["heartbeat"].flush()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = P4CEvidenceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for handle in node.files.values():
            handle.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
