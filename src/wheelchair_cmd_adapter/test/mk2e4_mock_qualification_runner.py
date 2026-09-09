#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import statistics
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Runner(Node):
    def __init__(self) -> None:
        super().__init__("mk2e4_mock_qualification_runner")
        self.publisher = self.create_publisher(TwistStamped, "/vehicle_cmd_safe", 10)
        self.bridge_output = None
        self.decoded_output = None
        self.frame_times = []
        self.create_subscription(
            Float32MultiArray,
            "/wheelchair_control_command",
            self._bridge_callback,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            "/phase5/mk2e4/mock_can_decoded",
            self._decoded_callback,
            100,
        )

    def _bridge_callback(self, message) -> None:
        self.bridge_output = [round(float(value), 4) for value in message.data]

    def _decoded_callback(self, message) -> None:
        self.decoded_output = [round(float(value), 4) for value in message.data]
        self.frame_times.append(time.monotonic())

    def send(self, v: float, w: float, duration: float = 0.45, frame="base_footprint"):
        self.bridge_output = None
        self.decoded_output = None
        deadline = time.monotonic() + duration
        next_publish = time.monotonic()
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                message = TwistStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = frame
                message.twist.linear.x = v
                message.twist.angular.z = w
                self.publisher.publish(message)
                next_publish += 0.04
            rclpy.spin_once(self, timeout_sec=0.005)
        drain_deadline = time.monotonic() + 0.08
        while time.monotonic() < drain_deadline:
            rclpy.spin_once(self, timeout_sec=0.005)
        return {"bridge": self.bridge_output, "backend": self.decoded_output}

    def silence(self, duration: float = 0.85):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        return {"bridge": self.bridge_output, "backend": self.decoded_output}


def main() -> None:
    rclpy.init()
    node = Runner()
    try:
        discovery_deadline = time.monotonic() + 2.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        results = {
            "forward_straight": node.send(0.25, 0.0),
            "forward_left": node.send(0.20, 0.10),
            "forward_right": node.send(0.20, -0.10),
            "reverse_straight": node.send(-0.25, 0.0),
            "reverse_left": node.send(-0.20, 0.10),
            "reverse_right": node.send(-0.20, -0.10),
            "zero": node.send(0.0, 0.0),
            "invalid_nan": node.send(math.nan, 0.0),
            "wrong_frame": node.send(0.20, 0.0, frame="base_link"),
            "in_place": node.send(0.0, 0.10),
        }
        node.send(0.25, 0.0, duration=0.25)
        results["stale"] = node.silence()
        gaps = [b - a for a, b in zip(node.frame_times, node.frame_times[1:])]
        results["heartbeat"] = {
            "samples": len(node.frame_times),
            "median_period_sec": round(statistics.median(gaps), 6),
            "max_gap_sec": round(max(gaps), 6),
        }
        print(json.dumps(results, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
