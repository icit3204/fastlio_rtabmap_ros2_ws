"""Passive, qualification-only collision-validity evidence observer.

This deliberately does not use ``ros2 topic echo`` or create any publisher.
The READY file is the handshake: it is written only after all subscriptions
exist and is independent of stdout pipes owned by a launch parent.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


VALIDITY_DIAGNOSTIC = "vehicle_cmd_safety/collision_monitor_validity_monitor"


def _gid(info) -> str:
    """Best-effort writer GID; absence is recorded rather than fabricated."""
    try:
        return bytes(info.publisher_gid).hex()
    except (AttributeError, TypeError):
        return ""


def _status_level(value) -> int:
    """ROS Humble may expose uint8 fields as a one-byte buffer."""
    if isinstance(value, (bytes, bytearray)):
        return int(value[0]) if value else 0
    return int(value)


def authority_rows(node) -> list[dict]:
    """Return, without publishing, the complete authority picture we observed."""
    rows = []
    for topic in ("/system/collision_monitor_valid", "/system/localization_valid",
                  "/system/controller_valid", "/phase4/synthetic_scan"):
        publishers = []
        for endpoint in node.get_publishers_info_by_topic(topic):
            publishers.append({"node": endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name,
                               "gid": bytes(endpoint.endpoint_gid).hex(), "topic_type": endpoint.topic_type})
        rows.append({"topic": topic, "publishers": publishers})
    return rows


class CollisionReasonObserver(Node):
    def __init__(self, output: Path) -> None:
        super().__init__("p4e6b_collision_reason_observer")
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self._events = (output / "observer.jsonl").open("a", encoding="utf-8")
        self._stopping = False
        self.create_subscription(Bool, "/system/collision_monitor_valid", self._validity, 50)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics, 50)
        self.create_subscription(LaserScan, "/phase4/synthetic_scan", self._scan, 50)
        self.create_timer(0.5, self._authority_snapshot)
        self._write("observer_started", pid=os.getpid(), ros_domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
                    ros_localhost_only=os.environ.get("ROS_LOCALHOST_ONLY", ""))
        self._write_ready()

    def _write(self, kind: str, **fields) -> None:
        row = {"receipt_monotonic_ns": time.monotonic_ns(), "event": kind, **fields}
        self._events.write(json.dumps(row, sort_keys=True) + "\n")
        self._events.flush()

    def _write_ready(self) -> None:
        ready = {"event": "READY", "receipt_monotonic_ns": time.monotonic_ns(), "pid": os.getpid(),
                 "node": self.get_fully_qualified_name(),
                 "topics": ["/system/collision_monitor_valid", "/diagnostics", "/phase4/synthetic_scan"]}
        path = self.output / "READY.json"
        path.write_text(json.dumps(ready, sort_keys=True) + "\n", encoding="utf-8")
        self._write("READY", **ready)

    def _validity(self, msg: Bool, info=None) -> None:
        self._write("collision_validity", value=bool(msg.data), publisher_gid=_gid(info))

    def _diagnostics(self, msg: DiagnosticArray, info=None) -> None:
        for status in msg.status:
            if status.name != VALIDITY_DIAGNOSTIC:
                continue
            values = {item.key: item.value for item in status.values}
            self._write("collision_validity_diagnostic", publisher_gid=_gid(info), level=_status_level(status.level),
                        message=status.message, reason_code=values.get("reason_code", status.message), values=values)

    def _scan(self, msg: LaserScan, info=None) -> None:
        structurally_valid = bool(msg.angle_increment > 0.0 and msg.range_max > msg.range_min > 0.0 and msg.ranges)
        finite_ranges = sum(1 for value in msg.ranges if msg.range_min <= value <= msg.range_max)
        self._write("synthetic_scan", publisher_gid=_gid(info), frame_id=msg.header.frame_id,
                    range_count=len(msg.ranges), finite_range_count=finite_ranges,
                    structurally_valid=structurally_valid)

    def _authority_snapshot(self) -> None:
        for row in authority_rows(self):
            self._write("publisher_authority", **row)

    def close(self) -> None:
        if not self._stopping:
            self._stopping = True
            self._write("observer_stopping")
            self._events.flush()
            os.fsync(self._events.fileno())
            self._events.close()


def main(args: list[str] | None = None) -> None:
    raw = list(sys.argv[1:] if args is None else args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parsed = parser.parse_args(remove_ros_args(raw))
    rclpy.init(args=raw)
    node = CollisionReasonObserver(Path(parsed.output_dir))
    deadline = time.monotonic() + parsed.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
