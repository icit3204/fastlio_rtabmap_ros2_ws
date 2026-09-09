#!/usr/bin/env python3
"""Real NavigateToPose orchestration recorder for stationary MK2F3."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def value_range(values):
    finite = [value for value in values if math.isfinite(value)]
    return {
        "count": len(finite),
        "min": min(finite) if finite else None,
        "median": statistics.median(finite) if finite else None,
        "max": max(finite) if finite else None,
    }


def path_metrics(path):
    distances = []
    reverse = inplace = nonfinite = 0
    for pose in path.poses:
        values = (pose.pose.position.x, pose.pose.position.y, yaw_of(pose.pose))
        nonfinite += int(not all(math.isfinite(value) for value in values))
    for first, second in zip(path.poses, path.poses[1:]):
        dx = second.pose.position.x - first.pose.position.x
        dy = second.pose.position.y - first.pose.position.y
        distance = math.hypot(dx, dy)
        yaw_delta = abs(wrap(yaw_of(second.pose) - yaw_of(first.pose)))
        if distance < 0.005:
            inplace += int(yaw_delta > 0.10)
            continue
        distances.append(distance)
        projection = dx * math.cos(yaw_of(first.pose)) + dy * math.sin(yaw_of(first.pose))
        reverse += int(projection < -0.01)
    return {
        "frame": path.header.frame_id,
        "pose_count": len(path.poses),
        "length_m": sum(distances),
        "reverse_segment_count": reverse,
        "in_place_segment_count": inplace,
        "nonfinite_pose_count": nonfinite,
    }


class NavigateToPoseStationaryRunner(Node):
    def __init__(self):
        super().__init__("mk2f3_navigate_to_pose_stationary_runner")
        self.navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.arm = self.create_client(SetBool, "/phase5/gate_test/arm")
        self.lifecycle = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in ("planner_server", "controller_server", "bt_navigator", "collision_monitor")
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.events = []
        self.latest = {}
        self.map = None
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, latched)
        self.create_subscription(PathMessage, "/plan", self.path_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self.twist_cb("mppi"), 100)
        self.create_subscription(Twist, "/cmd_vel", self.twist_cb("cm"), 100)
        self.create_subscription(TwistStamped, "/vehicle_cmd_safe", self.gate_cb, 100)
        self.create_subscription(Float32MultiArray, "/wheelchair_control_command", self.array_cb("bridge"), 100)
        self.create_subscription(Float32MultiArray, "/phase5/mk2e4/mock_can_decoded", self.array_cb("backend"), 100)
        self.create_subscription(DiagnosticArray, "/gate_to_labmate_bridge/diagnostics", self.bridge_diag_cb, 100)
        self.create_subscription(DiagnosticStatus, "/phase5/gate_test/state", self.state_cb, 100)
        self.create_subscription(Bool, "/system/collision_monitor_valid", self.valid_cb, 100)
        for action in ("navigate_to_pose", "compute_path_to_pose", "follow_path", "spin", "backup", "drive_on_heading"):
            self.create_subscription(GoalStatusArray, f"/{action}/_action/status", self.status_cb(action), 20)

    def record(self, topic, value):
        event = {"monotonic_ns": time.monotonic_ns(), "topic": topic, "value": value}
        self.events.append(event)
        self.latest[topic] = event

    def map_cb(self, msg):
        self.map = msg
        self.record("map", [msg.header.frame_id, msg.info.width, msg.info.height])

    def path_cb(self, msg):
        self.record("path", path_metrics(msg))

    def twist_cb(self, name):
        return lambda msg: self.record(name, [msg.linear.x, msg.angular.z])

    def gate_cb(self, msg):
        self.record("gate", [msg.twist.linear.x, msg.twist.angular.z, msg.header.frame_id])

    def array_cb(self, name):
        return lambda msg: self.record(name, list(msg.data))

    def bridge_diag_cb(self, msg):
        if msg.status:
            self.record("bridge_reason", msg.status[0].message)

    def state_cb(self, msg):
        self.record("gate_state", msg.message)

    def valid_cb(self, msg):
        self.record("required_valid", bool(msg.data))

    def status_cb(self, action):
        def callback(msg):
            statuses = [int(item.status) for item in msg.status_list]
            self.record(f"action:{action}", statuses)
        return callback

    def spin_until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def lifecycle_state(self, name):
        client = self.lifecycle[name]
        self.spin_until(client.service_is_ready, 30.0, f"{name} lifecycle service")
        future = client.call_async(GetState.Request())
        self.spin_until(future.done, 10.0, f"{name} lifecycle response")
        return int(future.result().current_state.id)

    def current_pose(self):
        self.spin_until(
            lambda: self.tf_buffer.can_transform("map", "base_footprint", Time(), timeout=Duration()),
            45.0, "map<-base_footprint TF")
        transform = self.tf_buffer.lookup_transform("map", "base_footprint", Time()).transform
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.translation.x
        pose.pose.position.y = transform.translation.y
        pose.pose.position.z = transform.translation.z
        pose.pose.orientation = transform.rotation
        return pose

    def map_value(self, x, y):
        grid = self.map
        origin = grid.info.origin
        yaw = yaw_of(origin)
        dx, dy = x - origin.position.x, y - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ix = int(math.floor(local_x / grid.info.resolution))
        iy = int(math.floor(local_y / grid.info.resolution))
        if ix < 0 or iy < 0 or ix >= grid.info.width or iy >= grid.info.height:
            return -1
        return int(grid.data[iy * grid.info.width + ix])

    def choose_goal(self, start):
        self.spin_until(lambda: self.map is not None and self.map.header.frame_id == "map", 45.0, "RTAB /map")
        yaw = yaw_of(start.pose)
        # Select from current TF only. Require a known-free straight corridor
        # wider than the authoritative footprint before invoking BT Navigator.
        for distance in (2.5, 2.0, 1.5, 1.0):
            clear = True
            samples = max(2, int(distance / 0.05))
            for step in range(samples + 1):
                forward = distance * step / samples
                for lateral in (-0.45, -0.30, 0.0, 0.30, 0.45):
                    x = start.pose.position.x + forward * math.cos(yaw) - lateral * math.sin(yaw)
                    y = start.pose.position.y + forward * math.sin(yaw) + lateral * math.cos(yaw)
                    value = self.map_value(x, y)
                    if value < 0 or value >= 50:
                        clear = False
                        break
                if not clear:
                    break
            if clear:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = start.pose.position.x + distance * math.cos(yaw)
                goal.pose.position.y = start.pose.position.y + distance * math.sin(yaw)
                goal.pose.orientation = start.pose.orientation
                return goal
        raise RuntimeError("no known-free current-pose forward goal corridor")

    def call_arm(self, enabled):
        self.spin_until(self.arm.service_is_ready, 10.0, "Gate arm service")
        request = SetBool.Request()
        request.data = enabled
        future = self.arm.call_async(request)
        self.spin_until(future.done, 10.0, f"Gate arm={enabled}")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("Gate arm request failed: " + (response.message if response else "no response"))
        return response.message

    def action_seen(self, action, statuses):
        return any(status in statuses for event in self.events if event["topic"] == f"action:{action}" for status in event["value"])

    def summarize(self, start_index):
        grouped = {}
        for event in self.events[start_index:]:
            grouped.setdefault(event["topic"], []).append(event["value"])
        mppi = grouped.get("mppi", [])
        backend = grouped.get("backend", [])
        return {
            "samples": {key: len(value) for key, value in grouped.items()},
            "mppi_linear_mps": value_range([value[0] for value in mppi]),
            "mppi_angular_rps": value_range([value[1] for value in mppi]),
            "cm_linear_mps": value_range([value[0] for value in grouped.get("cm", [])]),
            "gate_linear_mps": value_range([value[0] for value in grouped.get("gate", [])]),
            "backend_gears": sorted({int(value[0]) for value in backend}),
            "backend_speed_mmps": value_range([value[1] for value in backend]),
            "backend_steering_deg": value_range([value[2] for value in backend]),
            "checksum_all_valid": bool(backend) and all(value[5] == 1.0 for value in backend),
            "reserved_all_zero": bool(backend) and all(value[6] == 1.0 for value in backend),
            "bridge_reject_count": sum(value != "VALID" for value in grouped.get("bridge_reason", [])),
            "gate_states": sorted(set(grouped.get("gate_state", []))),
        }

    def run(self):
        self.spin_until(self.navigate.server_is_ready, 90.0, "NavigateToPose action")
        lifecycle = {name: self.lifecycle_state(name) for name in self.lifecycle}
        if any(value != State.PRIMARY_STATE_ACTIVE for value in lifecycle.values()):
            raise RuntimeError("required lifecycle node not active: " + json.dumps(lifecycle))
        self.spin_until(lambda: self.latest.get("required_valid", {}).get("value") is True, 45.0, "required perception validity")
        start = self.current_pose()
        goal = self.choose_goal(start)
        request = NavigateToPose.Goal()
        request.pose = goal
        request.behavior_tree = ""  # Exercise bt_navigator's configured canonical default.
        sent_ns = time.monotonic_ns()
        feedback = []
        future = self.navigate.send_goal_async(
            request,
            feedback_callback=lambda item: feedback.append({
                "monotonic_ns": time.monotonic_ns(),
                "recoveries": int(item.feedback.number_of_recoveries),
                "distance_remaining": float(item.feedback.distance_remaining),
            }))
        self.spin_until(future.done, 15.0, "NavigateToPose goal acceptance")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("NavigateToPose goal rejected")
        self.record("navigate_goal_accepted", True)
        result_future = handle.get_result_async()
        self.spin_until(lambda: self.action_seen("compute_path_to_pose", {GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_SUCCEEDED}), 20.0, "BT planner invocation")
        self.spin_until(lambda: self.latest.get("path", {}).get("monotonic_ns", 0) > sent_ns, 20.0, "planner path")
        self.spin_until(lambda: self.action_seen("follow_path", {GoalStatus.STATUS_EXECUTING}), 20.0, "BT FollowPath invocation")
        self.spin_until(lambda: self.latest.get("mppi", {}).get("monotonic_ns", 0) > sent_ns and abs(self.latest["mppi"]["value"][0]) >= 0.002, 15.0, "nonzero MPPI")
        self.spin_until(lambda: self.latest.get("cm", {}).get("monotonic_ns", 0) > sent_ns, 3.0, "Collision Monitor output")
        self.call_arm(False)
        self.spin_for(1.1)
        self.call_arm(True)
        self.spin_until(lambda: self.latest.get("gate_state", {}).get("value") == "ARMED_COMMAND" and abs(self.latest.get("gate", {}).get("value", [0.0])[0]) >= 0.002, 12.0, "armed Gate command")
        active_start = len(self.events)
        self.spin_for(2.0)
        active = self.summarize(active_start)
        disarm_message = self.call_arm(False)
        disarm_start = len(self.events)
        self.spin_for(1.0)
        disarmed = self.summarize(disarm_start)
        cancel_future = handle.cancel_goal_async()
        self.spin_until(cancel_future.done, 10.0, "NavigateToPose cancel response")
        self.spin_until(result_future.done, 15.0, "NavigateToPose canceled result")
        cancel_status = int(result_future.result().status)
        self.spin_for(0.5)
        cancel_start = len(self.events)
        self.spin_for(1.0)
        cancelled = self.summarize(cancel_start)
        incompatible = {
            name: sum(len(event["value"]) for event in self.events if event["topic"] == f"action:{name}")
            for name in ("spin", "backup", "drive_on_heading")
        }
        return {
            "lifecycle_state_ids": lifecycle,
            "start_pose": [start.pose.position.x, start.pose.position.y, yaw_of(start.pose)],
            "goal_pose": [goal.pose.position.x, goal.pose.position.y, yaw_of(goal.pose)],
            "navigate_to_pose_accepted": True,
            "planner_invoked": self.action_seen("compute_path_to_pose", {GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_SUCCEEDED}),
            "follow_path_invoked": self.action_seen("follow_path", {GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_CANCELED}),
            "path": self.latest["path"]["value"],
            "active": active,
            "disarm": disarmed,
            "disarm_response": disarm_message,
            "cancel_status": cancel_status,
            "after_cancel": cancelled,
            "feedback_count": len(feedback),
            "max_reported_recoveries": max((item["recoveries"] for item in feedback), default=0),
            "incompatible_recovery_status_samples": incompatible,
            "events": self.events,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--timeline-csv", required=True)
    parser.add_argument("--bt-node-csv", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = NavigateToPoseStationaryRunner()
    try:
        result = node.run()
        Path(args.output_json).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        with Path(args.timeline_csv).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["monotonic_ns", "topic", "value"])
            writer.writeheader()
            for event in result["events"]:
                writer.writerow({**event, "value": json.dumps(event["value"], sort_keys=True)})
        with Path(args.bt_node_csv).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["node", "status_sample_count"])
            writer.writeheader()
            for name in ("compute_path_to_pose", "follow_path", "spin", "backup", "drive_on_heading"):
                count = sum(len(event["value"]) for event in result["events"] if event["topic"] == f"action:{name}")
                writer.writerow({"node": name, "status_sample_count": count})
        print("MK2F3_RESULT " + json.dumps({key: value for key, value in result.items() if key != "events"}, sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
