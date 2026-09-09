#!/usr/bin/env python3
"""Passive, monotonic-clock recorder for the P5A-MK2E1 dry run.

It is deliberately observation-only: no publishers, services, transports, or
CAN interfaces.  The adapter diagnostic exposes fields decoded from the real
codec frame held by its in-memory MockTransport.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class Recorder(Node):
    def __init__(self) -> None:
        super().__init__("mk2e1_passive_command_chain_recorder")
        self.events: list[dict] = []
        self.create_subscription(Twist, "/phase5/cm_test/cmd_vel_in", self._twist("cm_input"), 10)
        self.create_subscription(Twist, "/phase5/cm_test/cmd_vel_out", self._twist("cm_output"), 10)
        self.create_subscription(TwistStamped, "/phase5/gate_test/cmd_vel_safe", self._safe, 10)
        self.create_subscription(DiagnosticStatus, "/phase5/gate_test/state", self._state("gate_state"), 10)
        self.create_subscription(
            DiagnosticStatus,
            "/phase5/mkmini_dry_run/adapter_state",
            self._state("adapter_state"),
            10,
        )

    def _append(self, topic: str, **fields) -> None:
        self.events.append({"monotonic_ns": time.monotonic_ns(), "topic": topic, **fields})

    def _twist(self, topic: str):
        def callback(msg: Twist) -> None:
            self._append(topic, linear_x=float(msg.linear.x), angular_z=float(msg.angular.z))
        return callback

    def _safe(self, msg: TwistStamped) -> None:
        self._append(
            "gate_safe",
            linear_x=float(msg.twist.linear.x),
            angular_z=float(msg.twist.angular.z),
            stamp_ns=int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
        )

    def _state(self, topic: str):
        def callback(msg: DiagnosticStatus) -> None:
            level = msg.level if isinstance(msg.level, int) else int.from_bytes(msg.level, byteorder="little")
            self._append(
                topic,
                level=level,
                message=msg.message,
                name=msg.name,
                hardware_id=msg.hardware_id,
                values={item.key: item.value for item in msg.values},
            )
        return callback


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-sec", required=True, type=float)
    args = parser.parse_args()
    if args.duration_sec <= 0:
        raise SystemExit("duration must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Recorder()
    started = time.monotonic_ns()
    try:
        while rclpy.ok() and (time.monotonic_ns() - started) / 1e9 < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        ended = time.monotonic_ns()
        args.output.write_text(json.dumps({
            "recorder": "mk2e1_passive_command_chain_recorder",
            "clock": "CLOCK_MONOTONIC",
            "started_monotonic_ns": started,
            "ended_monotonic_ns": ended,
            "events": node.events,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
