#!/usr/bin/env python3
"""Direct FollowPath qualification runner for the stationary MK2F1 mock chain."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


def _median(values):
    return statistics.median(values) if values else None


def _range(values):
    return {
        "count": len(values),
        "min": min(values) if values else None,
        "median": _median(values),
        "max": max(values) if values else None,
    }


class MppiStationaryRunner(Node):
    def __init__(self) -> None:
        super().__init__("mk2f1_mppi_stationary_runner")
        self.action = ActionClient(self, FollowPath, "/follow_path")
        self.arm_client = self.create_client(SetBool, "/phase5/gate_test/arm")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest = {}
        self.events = []
        self.create_subscription(Twist, "/cmd_vel_nav", self._twist_cb("mppi"), 100)
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb("cm"), 100)
        self.create_subscription(TwistStamped, "/vehicle_cmd_safe", self._gate_cb, 100)
        self.create_subscription(
            Float32MultiArray, "/wheelchair_control_command", self._array_cb("bridge"), 100
        )
        self.create_subscription(
            Float32MultiArray, "/phase5/mk2e4/mock_can_decoded",
            self._array_cb("backend"), 100
        )
        self.create_subscription(
            DiagnosticArray, "/gate_to_labmate_bridge/diagnostics", self._bridge_diag_cb, 100
        )
        self.create_subscription(
            DiagnosticStatus, "/phase5/gate_test/state", self._gate_state_cb, 100
        )
        self.create_subscription(
            Bool, "/system/collision_monitor_valid", self._valid_cb, 100
        )

    def _record(self, topic, value):
        event = {"monotonic_ns": time.monotonic_ns(), "topic": topic, "value": value}
        self.latest[topic] = event
        self.events.append(event)

    def _twist_cb(self, topic):
        def callback(msg):
            self._record(topic, [msg.linear.x, msg.angular.z])
        return callback

    def _array_cb(self, topic):
        def callback(msg):
            self._record(topic, list(msg.data))
        return callback

    def _gate_cb(self, msg):
        self._record(
            "gate", [msg.twist.linear.x, msg.twist.angular.z, msg.header.frame_id]
        )

    def _bridge_diag_cb(self, msg):
        if msg.status:
            self._record("bridge_reason", msg.status[0].message)

    def _gate_state_cb(self, msg):
        self._record("gate_state", msg.message)

    def _valid_cb(self, msg):
        self._record("required_valid", bool(msg.data))

    def spin_until(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {description}")

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call_arm(self, enabled):
        self.spin_until(self.arm_client.service_is_ready, 10.0, "Gate arm service")
        request = SetBool.Request()
        request.data = enabled
        future = self.arm_client.call_async(request)
        self.spin_until(future.done, 10.0, f"Gate arm={enabled} response")
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"Gate arm={enabled} failed: {message}")
        return response.message

    def current_pose(self):
        def available():
            return self.tf_buffer.can_transform(
                "odom_chassis", "base_footprint", Time(), timeout=Duration()
            )

        self.spin_until(available, 15.0, "odom_chassis<-base_footprint TF")
        transform = self.tf_buffer.lookup_transform(
            "odom_chassis", "base_footprint", Time()
        ).transform
        q = transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return transform.translation.x, transform.translation.y, yaw

    def make_path(self, scenario):
        x0, y0, yaw0 = self.current_pose()
        message = PathMessage()
        message.header.frame_id = "odom_chassis"
        message.header.stamp = self.get_clock().now().to_msg()
        count = 41
        if scenario == "straight" or scenario == "override":
            local = [(1.5 * i / (count - 1), 0.0, 0.0) for i in range(count)]
        else:
            sign = 1.0 if scenario == "left" else -1.0
            radius = 2.5
            end_angle = 0.55
            local = []
            for i in range(count):
                angle = end_angle * i / (count - 1)
                local.append((
                    radius * math.sin(angle),
                    sign * radius * (1.0 - math.cos(angle)),
                    sign * angle,
                ))
        cy, sy = math.cos(yaw0), math.sin(yaw0)
        for lx, ly, local_yaw in local:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x0 + cy * lx - sy * ly
            pose.pose.position.y = y0 + sy * lx + cy * ly
            yaw = yaw0 + local_yaw
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            message.poses.append(pose)
        return message

    def send_goal(self, scenario):
        goal = FollowPath.Goal()
        goal.path = self.make_path(scenario)
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"
        future = self.action.send_goal_async(goal)
        self.spin_until(future.done, 10.0, f"{scenario} goal acceptance")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{scenario} FollowPath goal rejected")
        return handle

    def wait_for_nonzero_mppi(self, after_monotonic_ns):
        self.spin_until(
            lambda: "mppi" in self.latest
            and self.latest["mppi"]["monotonic_ns"] > after_monotonic_ns
            and abs(self.latest["mppi"]["value"][0]) >= 0.002,
            10.0,
            "new nonzero MPPI output",
        )
        # Collision Monitor publishes from the command callback.  Require a
        # correspondingly new downstream sample before clearing a stale-input
        # latch left by the deliberately idle interval between test actions.
        self.spin_until(
            lambda: "cm" in self.latest
            and self.latest["cm"]["monotonic_ns"] > after_monotonic_ns,
            2.0,
            "new Collision Monitor output",
        )

    def prepare_armed_action(self, scenario):
        action_start_ns = time.monotonic_ns()
        handle = self.send_goal(scenario)
        self.wait_for_nonzero_mppi(action_start_ns)
        self.call_arm(False)
        self.spin_for(1.1)
        self.call_arm(True)
        self.spin_until(
            lambda: self.latest.get("gate_state", {}).get("value") == "ARMED_COMMAND"
            and abs(self.latest.get("gate", {}).get("value", [0.0])[0]) >= 0.002,
            10.0,
            "armed nonzero Gate output",
        )
        self.spin_for(0.5)
        return handle

    def cancel(self, handle):
        future = handle.cancel_goal_async()
        self.spin_until(future.done, 10.0, "FollowPath cancel")

    def summarize(self, start_index, end_index=None):
        events = self.events[start_index:end_index]
        by_topic = {}
        for event in events:
            by_topic.setdefault(event["topic"], []).append(event)
        mppi = [event["value"] for event in by_topic.get("mppi", [])]
        cm = [event["value"] for event in by_topic.get("cm", [])]
        gate = [event["value"] for event in by_topic.get("gate", [])]
        backend = [event["value"] for event in by_topic.get("backend", [])]
        radii = []
        in_place = 0
        nonfinite = 0
        for velocity, yaw_rate in mppi:
            if not math.isfinite(velocity) or not math.isfinite(yaw_rate):
                nonfinite += 1
                continue
            if abs(velocity) < 0.001 and abs(yaw_rate) >= 0.001:
                in_place += 1
            if abs(velocity) >= 0.001:
                radii.append(10.0 if abs(yaw_rate) < 0.001 else abs(velocity / yaw_rate))
        reasons = [event["value"] for event in by_topic.get("bridge_reason", [])]
        return {
            "samples": {topic: len(items) for topic, items in by_topic.items()},
            "mppi_linear": _range([value[0] for value in mppi]),
            "mppi_angular": _range([value[1] for value in mppi]),
            "derived_radius_abs_m": _range(radii),
            "cm_linear": _range([value[0] for value in cm]),
            "gate_linear": _range([value[0] for value in gate]),
            "backend_speed_mmps": _range([value[1] for value in backend]),
            "backend_steering_deg": _range([value[2] for value in backend]),
            "bridge_reject_count": sum(reason != "VALID" for reason in reasons),
            "bridge_reasons": sorted(set(reasons)),
            "in_place_request_count": in_place,
            "nonfinite_count": nonfinite,
            "gate_states": sorted({event["value"] for event in by_topic.get("gate_state", [])}),
            "backend_gears": sorted({int(value[0]) for value in backend}),
            "checksum_all_valid": all(value[5] == 1.0 for value in backend),
            "reserved_all_zero": all(value[6] == 1.0 for value in backend),
        }

    def run(self):
        self.spin_until(self.action.server_is_ready, 30.0, "FollowPath action server")
        self.spin_until(
            lambda: self.latest.get("required_valid", {}).get("value") is True,
            30.0,
            "required perception validity",
        )
        results = {"started_monotonic_ns": time.monotonic_ns(), "scenarios": {}}
        for scenario in ("straight", "left", "right"):
            handle = self.prepare_armed_action(scenario)
            start = len(self.events)
            self.spin_for(3.0)
            results["scenarios"][scenario] = self.summarize(start)
            self.cancel(handle)
            self.spin_for(0.8)

        handle = self.prepare_armed_action("override")
        pre_disarm = len(self.events)
        self.spin_for(1.0)
        results["scenarios"]["override_before_disarm"] = self.summarize(pre_disarm)
        response = self.call_arm(False)
        disarm_start = len(self.events)
        self.spin_for(1.0)
        results["scenarios"]["override_disarmed"] = self.summarize(disarm_start)
        results["disarm_response"] = response
        self.cancel(handle)
        cancel_start = len(self.events)
        self.spin_for(1.0)
        results["scenarios"]["after_cancel"] = self.summarize(cancel_start)
        results["finished_monotonic_ns"] = time.monotonic_ns()
        return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json")
    args = parser.parse_args()
    rclpy.init()
    node = MppiStationaryRunner()
    try:
        results = node.run()
        rendered = json.dumps(results, indent=2, sort_keys=True)
        print("MK2F1_RESULT " + rendered, flush=True)
        if args.output_json:
            Path(args.output_json).write_text(rendered + "\n", encoding="utf-8")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
