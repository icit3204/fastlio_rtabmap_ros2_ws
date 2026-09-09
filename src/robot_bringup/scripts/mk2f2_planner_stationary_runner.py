#!/usr/bin/env python3
"""Direct ComputePathToPose -> FollowPath qualification for stationary MK2F2."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def finite_range(values):
    values = [value for value in values if math.isfinite(value)]
    return {
        "count": len(values),
        "min": min(values) if values else None,
        "median": statistics.median(values) if values else None,
        "max": max(values) if values else None,
    }


def analyze_path(path: PathMessage):
    poses = path.poses
    lengths, headings, radii = [], [], []
    reverse = cusp = inplace = duplicate = nonfinite = 0
    previous_heading = None
    previous_segment = None
    for pose in poses:
        values = (pose.pose.position.x, pose.pose.position.y, yaw_of(pose.pose))
        if not all(math.isfinite(value) for value in values):
            nonfinite += 1
    for first, second in zip(poses, poses[1:]):
        dx = second.pose.position.x - first.pose.position.x
        dy = second.pose.position.y - first.pose.position.y
        distance = math.hypot(dx, dy)
        yaw_delta = abs(wrap(yaw_of(second.pose) - yaw_of(first.pose)))
        if distance < 0.005:
            duplicate += 1
            if yaw_delta > 0.10:
                inplace += 1
            continue
        heading = math.atan2(dy, dx)
        lengths.append(distance)
        headings.append(heading)
        forward_projection = dx * math.cos(yaw_of(first.pose)) + dy * math.sin(yaw_of(first.pose))
        if forward_projection < -0.01:
            reverse += 1
        if previous_heading is not None:
            average = 0.5 * (lengths[-2] + lengths[-1])
            delta = wrap(heading - previous_heading)
            if abs(delta) > 1e-5 and average > 1e-5:
                radii.append(abs(average / delta))
            if previous_segment is not None and (dx * previous_segment[0] + dy * previous_segment[1]) < -0.0005:
                cusp += 1
        previous_heading = heading
        previous_segment = (dx, dy)
    return {
        "frame": path.header.frame_id,
        "pose_count": len(poses),
        "length_m": sum(lengths),
        "segment_length_m": finite_range(lengths),
        "estimated_radius_m": finite_range(radii),
        "radius_below_1_75_count": sum(radius < 1.75 for radius in radii),
        "reverse_segment_count": reverse,
        "cusp_count": cusp,
        "in_place_segment_count": inplace,
        "duplicate_segment_count": duplicate,
        "nonfinite_pose_count": nonfinite,
    }


class PlannerStationaryRunner(Node):
    def __init__(self):
        super().__init__("mk2f2_planner_stationary_runner")
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.controller = ActionClient(self, FollowPath, "/follow_path")
        self.arm = self.create_client(SetBool, "/phase5/gate_test/arm")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.events, self.latest = [], {}
        self.create_subscription(Twist, "/cmd_vel_nav", self.twist_cb("mppi"), 100)
        self.create_subscription(Twist, "/cmd_vel", self.twist_cb("cm"), 100)
        self.create_subscription(TwistStamped, "/vehicle_cmd_safe", self.gate_cb, 100)
        self.create_subscription(Float32MultiArray, "/wheelchair_control_command", self.array_cb("bridge"), 100)
        self.create_subscription(Float32MultiArray, "/phase5/mk2e4/mock_can_decoded", self.array_cb("backend"), 100)
        self.create_subscription(DiagnosticArray, "/gate_to_labmate_bridge/diagnostics", self.bridge_diag_cb, 100)
        self.create_subscription(DiagnosticStatus, "/phase5/gate_test/state", self.state_cb, 100)
        self.create_subscription(Bool, "/system/collision_monitor_valid", self.valid_cb, 100)

    def record(self, topic, value):
        event = {"monotonic_ns": time.monotonic_ns(), "topic": topic, "value": value}
        self.events.append(event)
        self.latest[topic] = event

    def twist_cb(self, topic):
        return lambda msg: self.record(topic, [msg.linear.x, msg.angular.z])

    def array_cb(self, topic):
        return lambda msg: self.record(topic, list(msg.data))

    def gate_cb(self, msg):
        self.record("gate", [msg.twist.linear.x, msg.twist.angular.z, msg.header.frame_id])

    def bridge_diag_cb(self, msg):
        if msg.status:
            self.record("bridge_reason", msg.status[0].message)

    def state_cb(self, msg):
        self.record("gate_state", msg.message)

    def valid_cb(self, msg):
        self.record("required_valid", bool(msg.data))

    def spin_until(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def map_pose(self):
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

    def compute(self, start, goal):
        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal
        request.use_start = True
        request.planner_id = "GridBased"
        future = self.planner.send_goal_async(request)
        self.spin_until(future.done, 15.0, "ComputePathToPose goal acceptance")
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        self.spin_until(result_future.done, 30.0, "ComputePathToPose result")
        result = result_future.result()
        return result.result.path if result and result.result else None

    def find_path(self):
        start = self.map_pose()
        yaw = yaw_of(start.pose)
        # These are map-frame candidates projected from the *current* TF pose.
        # The first successful planner result, not a fabricated path, becomes evidence.
        for distance in (2.5, 2.0, 1.5, 1.0):
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = start.pose.position.x + distance * math.cos(yaw)
            goal.pose.position.y = start.pose.position.y + distance * math.sin(yaw)
            goal.pose.orientation = start.pose.orientation
            path = self.compute(start, goal)
            if path and len(path.poses) >= 2:
                return start, goal, path
        raise RuntimeError("no current-TF forward candidate produced a planner path")

    def call_arm(self, enabled):
        self.spin_until(self.arm.service_is_ready, 10.0, "Gate arm service")
        request = SetBool.Request()
        request.data = enabled
        future = self.arm.call_async(request)
        self.spin_until(future.done, 10.0, "Gate arm response")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("Gate arm request failed: " + (response.message if response else "no response"))
        return response.message

    def follow(self, path):
        request = FollowPath.Goal()
        request.path = path
        request.controller_id = "FollowPath"
        request.goal_checker_id = "general_goal_checker"
        sent_at = time.monotonic_ns()
        future = self.controller.send_goal_async(request)
        self.spin_until(future.done, 10.0, "FollowPath goal acceptance")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("planner-derived FollowPath rejected")
        self.spin_until(lambda: self.latest.get("mppi", {}).get("monotonic_ns", 0) > sent_at and abs(self.latest["mppi"]["value"][0]) >= 0.002, 12.0, "new nonzero MPPI")
        self.spin_until(lambda: self.latest.get("cm", {}).get("monotonic_ns", 0) > sent_at, 2.0, "new Collision Monitor output")
        self.call_arm(False)
        self.spin_for(1.1)
        self.call_arm(True)
        self.spin_until(lambda: self.latest.get("gate_state", {}).get("value") == "ARMED_COMMAND" and abs(self.latest.get("gate", {}).get("value", [0.0])[0]) >= 0.002, 12.0, "armed Gate output")
        return handle

    def cancel(self, handle):
        future = handle.cancel_goal_async()
        self.spin_until(future.done, 10.0, "FollowPath cancellation")

    def summarize(self, start):
        topics = {}
        for event in self.events[start:]:
            topics.setdefault(event["topic"], []).append(event["value"])
        mppi = topics.get("mppi", [])
        backend = topics.get("backend", [])
        radii, inplace, nonfinite = [], 0, 0
        for velocity, yaw_rate in mppi:
            if not math.isfinite(velocity) or not math.isfinite(yaw_rate):
                nonfinite += 1
            elif abs(velocity) < 0.001 and abs(yaw_rate) >= 0.001:
                inplace += 1
            elif abs(velocity) >= 0.001:
                radii.append(10.0 if abs(yaw_rate) < 0.001 else abs(velocity / yaw_rate))
        reasons = topics.get("bridge_reason", [])
        return {
            "mppi_linear_mps": finite_range([item[0] for item in mppi]),
            "mppi_angular_rps": finite_range([item[1] for item in mppi]),
            "derived_radius_abs_m": finite_range(radii),
            "bridge_reject_count": sum(reason != "VALID" for reason in reasons),
            "bridge_reasons": sorted(set(reasons)),
            "in_place_request_count": inplace,
            "nonfinite_request_count": nonfinite,
            "backend_gears": sorted({int(item[0]) for item in backend}),
            "backend_speed_mmps": finite_range([item[1] for item in backend]),
            "backend_steering_deg": finite_range([item[2] for item in backend]),
            "checksum_all_valid": all(item[5] == 1.0 for item in backend),
            "reserved_bits_all_zero": all(item[6] == 1.0 for item in backend),
            "gate_states": sorted(set(topics.get("gate_state", []))),
        }

    def run(self):
        self.spin_until(self.planner.server_is_ready, 60.0, "ComputePathToPose server")
        self.spin_until(self.controller.server_is_ready, 60.0, "FollowPath server")
        self.spin_until(lambda: self.latest.get("required_valid", {}).get("value") is True, 45.0, "required perception validity")
        start, goal, path = self.find_path()
        feasibility = analyze_path(path)
        if feasibility["frame"] != "map" or feasibility["reverse_segment_count"] or feasibility["cusp_count"] or feasibility["in_place_segment_count"] or feasibility["nonfinite_pose_count"] or feasibility["radius_below_1_75_count"]:
            raise RuntimeError("planner path failed the Ackermann feasibility contract: " + json.dumps(feasibility))
        handle = self.follow(path)
        active_start = len(self.events)
        self.spin_for(3.0)
        active = self.summarize(active_start)
        if active["bridge_reject_count"] or active["in_place_request_count"] or active["nonfinite_request_count"]:
            raise RuntimeError("normal planned-path commands violated bridge contract: " + json.dumps(active))
        disarm_response = self.call_arm(False)
        disarm_start = len(self.events)
        self.spin_for(1.0)
        disarmed = self.summarize(disarm_start)
        self.cancel(handle)
        cancel_start = len(self.events)
        self.spin_for(1.0)
        cancelled = self.summarize(cancel_start)
        return {
            "start_pose": [start.pose.position.x, start.pose.position.y, yaw_of(start.pose)],
            "goal_pose": [goal.pose.position.x, goal.pose.position.y, yaw_of(goal.pose)],
            "feasibility": feasibility,
            "planned_path_active": active,
            "gate_disarmed": disarmed,
            "after_cancel": cancelled,
            "disarm_response": disarm_response,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--path-csv", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = PlannerStationaryRunner()
    try:
        result = node.run()
        Path(args.output_json).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        with Path(args.path_csv).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["metric", "value"])
            writer.writeheader()
            for key, value in result["feasibility"].items():
                writer.writerow({"metric": key, "value": json.dumps(value) if isinstance(value, dict) else value})
        print("MK2F2_RESULT " + json.dumps(result, sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
