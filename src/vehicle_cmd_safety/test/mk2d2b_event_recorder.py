#!/usr/bin/env python3
"""Passive single-clock timing recorder for P5A-MK2D2B.

The recorder subscribes only.  It has no services, publishers, or connection
to the safety decision path.  Each received event is timestamped at callback
receipt with ``time.monotonic_ns()``, thereby keeping all comparison points on
one local monotonic clock.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool


def _stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class EventRecorder(Node):
    def __init__(self) -> None:
        super().__init__("mk2d2b_passive_event_recorder")
        self.events: list[dict] = []
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/cloud_registered_body", self._cloud, qos_profile_sensor_data)
        self.create_subscription(Bool, "/phase5/perception/mid360_valid", self._bool("mid360_valid"), 10)
        self.create_subscription(Bool, "/phase5/perception/tmini_valid", self._bool("tmini_valid"), 10)
        self.create_subscription(Bool, "/system/collision_monitor_valid", self._bool("required_valid"), 10)
        self.create_subscription(Twist, "/phase5/cm_test/cmd_vel_in", self._twist("cm_input"), 10)
        self.create_subscription(Twist, "/phase5/cm_test/cmd_vel_out", self._twist("cm_output"), 10)
        self.create_subscription(TwistStamped, "/phase5/gate_test/cmd_vel_safe", self._safe, 10)
        self.create_subscription(DiagnosticStatus, "/phase5/gate_test/state", self._state, 10)

    @staticmethod
    def _now() -> int:
        return time.monotonic_ns()

    def _append(self, topic: str, **fields) -> None:
        self.events.append({"monotonic_ns": self._now(), "topic": topic, **fields})

    def _scan(self, msg: LaserScan) -> None:
        self._append("scan", header_stamp_ns=_stamp_ns(msg), frame_id=msg.header.frame_id, count=len(msg.ranges))

    def _cloud(self, msg: PointCloud2) -> None:
        self._append("cloud", header_stamp_ns=_stamp_ns(msg), frame_id=msg.header.frame_id, width=msg.width, height=msg.height)

    def _bool(self, topic: str):
        def callback(msg: Bool) -> None:
            self._append(topic, value=bool(msg.data))
        return callback

    def _twist(self, topic: str):
        def callback(msg: Twist) -> None:
            self._append(topic, linear_x=float(msg.linear.x), angular_z=float(msg.angular.z))
        return callback

    def _safe(self, msg: TwistStamped) -> None:
        self._append("gate_safe", header_stamp_ns=_stamp_ns(msg), linear_x=float(msg.twist.linear.x), angular_z=float(msg.twist.angular.z))

    def _state(self, msg: DiagnosticStatus) -> None:
        level = msg.level if isinstance(msg.level, int) else int.from_bytes(msg.level, byteorder="little")
        self._append(
            "gate_state",
            level=level,
            reason=msg.message,
            values={entry.key: entry.value for entry in msg.values},
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-sec", required=True, type=float)
    args = parser.parse_args()
    if args.duration_sec <= 0:
        raise SystemExit("duration must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = EventRecorder()
    started = time.monotonic_ns()
    try:
        while rclpy.ok() and (time.monotonic_ns() - started) / 1e9 < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        ended = time.monotonic_ns()
        args.output.write_text(json.dumps({
            "recorder": "mk2d2b_passive_event_recorder",
            "clock": "CLOCK_MONOTONIC",
            "started_monotonic_ns": started,
            "ended_monotonic_ns": ended,
            "events": node.events,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
