"""Runnable, subscription-only S01 terminal/physical closure witness."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

POST_FAILED_DRAIN_SEC = 0.250
MISSION_TOPIC = "/mission/state"
ACTION_STATUS_TOPIC = "/navigate_to_pose/_action/status"
OBSERVED_TOPICS = (
    "/cmd_vel_nav_raw", "/cmd_vel_nav_safe", "/vehicle_cmd_safe",
    "/wheelchair_control_command_mock", "/Odometry", "/diagnostics",
)
WITNESS_CONTRACT = {
    "publishers": [], "services": [], "action_clients": [],
    "observes": [MISSION_TOPIC, ACTION_STATUS_TOPIC, *OBSERVED_TOPICS],
    "passive": True, "post_failed_drain_sec": POST_FAILED_DRAIN_SEC,
    "start_marker": "TERMINAL_WITNESS_READY",
}


def contract() -> dict:
    return dict(WITNESS_CONTRACT)


def _fsync_json(path: Path, value: dict) -> None:
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)
    fd = os.open(str(path.parent), os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _append(path: Path, value: dict) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(value, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def _stamp(node):
    return time.monotonic_ns(), node.get_clock().now().nanoseconds


def _twist_values(msg):
    t = getattr(msg, "twist", msg)
    return {"linear_x": float(t.linear.x), "linear_y": float(t.linear.y),
            "linear_z": float(t.linear.z), "angular_x": float(t.angular.x),
            "angular_y": float(t.angular.y), "angular_z": float(t.angular.z)}


def _byte_int(value):
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, "little")
    return int(value)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TerminalClosureWitness:
    """ROS node with subscriptions only and an explicit terminal drain."""

    def __init__(self, out: Path, reason: str | None, mission_id: str | None,
                 route_id: str | None, waypoint: int | None):
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import Twist, TwistStamped
        from nav_msgs.msg import Odometry
        from parking_robot_interfaces.msg import MissionState
        from std_msgs.msg import Float32MultiArray
        from rclpy.node import Node

        self.rclpy = rclpy
        self.out = out
        self.out.mkdir(parents=True, exist_ok=True)
        self.phase = self.out / "terminal_witness_events.jsonl"
        self.mission_file = self.out / "mission_state_events.jsonl"
        self.status_file = self.out / "navigate_action_status_events.jsonl"
        self.diag_file = self.out / "gate_diagnostics.jsonl"
        self.reason = reason
        self.identity = {"mission_id": mission_id, "route_id": route_id,
                         "waypoint": waypoint, "uuid": None}
        self.failed_received_ns = None
        self.drain_start_ns = None
        self.drain_end_ns = None
        self.stop_requested = False
        self.finalized = False
        self.evidence_sealed = False

        class NodeImpl(Node):
            pass

        self.node = NodeImpl("phase4_p4e6b_terminal_closure_witness", enable_rosout=False)
        self.node.create_subscription(MissionState, MISSION_TOPIC, self._mission, 100)
        self.node.create_subscription(GoalStatusArray, ACTION_STATUS_TOPIC, self._status, 100)
        self.node.create_subscription(Twist, "/cmd_vel_nav_raw", self._raw, 100)
        self.node.create_subscription(Twist, "/cmd_vel_nav_safe", self._safe, 100)
        self.node.create_subscription(TwistStamped, "/vehicle_cmd_safe", self._vehicle, 100)
        self.node.create_subscription(Float32MultiArray, "/wheelchair_control_command_mock", self._mock, 100)
        self.node.create_subscription(Odometry, "/Odometry", self._odom, 100)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics, 100)
        self.node.create_timer(0.01, self._drain_timer)
        self._write_authority_audit()
        self._ready()

    def _write_authority_audit(self):
        pubs = self.node.get_publisher_names_and_types_by_node(self.node.get_name(), self.node.get_namespace())
        srvs = self.node.get_service_names_and_types_by_node(self.node.get_name(), self.node.get_namespace())
        clients = self.node.get_client_names_and_types_by_node(self.node.get_name(), self.node.get_namespace())
        _fsync_json(self.out / "witness_authority.json", {
            "application_publishers": pubs, "application_services": srvs,
            "application_clients": clients,
            "framework_rosout_excluded": True,
            "APPLICATION_CONTROL_AUTHORITY_COUNT": 0,
            "contract": contract(),
        })

    def _ready(self):
        now, ros = _stamp(self.node)
        _append(self.phase, {"event": "TERMINAL_WITNESS_READY", "monotonic_ns": now,
                             "ros_ns": ros, "APPLICATION_CONTROL_AUTHORITY_COUNT": 0,
                             "subscriptions_created": 7})
        fd = os.open(str(self.phase), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        _fsync_json(self.out / "TERMINAL_WITNESS_READY", {"event": "TERMINAL_WITNESS_READY", "monotonic_ns": now})

    def _common(self):
        mono, ros = _stamp(self.node)
        return {"monotonic_ns": mono, "ros_ns": ros}

    def _mission(self, msg):
        if self.evidence_sealed:
            return
        row = self._common() | {"mission_id": msg.mission_id, "route_id": msg.route_id,
            "waypoint_index": int(msg.current_waypoint_index), "state": int(msg.state),
            "state_name": {0:"IDLE",1:"RECEIVED",2:"VALIDATING",3:"PLANNING",4:"NAVIGATING",5:"PAUSED",6:"CANCELLING",7:"CANCELLED",8:"SUCCEEDED",9:"TEMPORARILY_BLOCKED",10:"BLOCKED",11:"FAILED",12:"HELP_REQUIRED"}.get(int(msg.state), "UNKNOWN"),
            "active_goal_uuid": msg.active_goal_uuid, "reason_code": msg.reason_code, "detail": msg.detail}
        if msg.state == msg.NAVIGATING and msg.active_goal_uuid:
            self.identity.update(mission_id=msg.mission_id, route_id=msg.route_id,
                                 waypoint=int(msg.current_waypoint_index), uuid=msg.active_goal_uuid)
        _append(self.mission_file, row)
        if (msg.state == msg.CANCELLING and msg.reason_code == "HEALTH_CANCEL_ACK_ACCEPTED"):
            _append(self.phase, {"event": "HEALTH_CANCEL_ACK_ACCEPTED", **row})
        if msg.state == msg.FAILED and (self.reason is None or msg.reason_code == self.reason):
            if self.identity["mission_id"] in (None, msg.mission_id) and self.identity["route_id"] in (None, msg.route_id):
                self.reason = msg.reason_code
                self.failed_received_ns = row["monotonic_ns"]
                self.drain_start_ns = row["monotonic_ns"]
                _append(self.phase, {"event": "FAILED_RECEIVED", "failed_received_ns": self.failed_received_ns,
                                     "drain_start_ns": self.drain_start_ns, **row})

    def _status(self, msg):
        if self.evidence_sealed:
            return
        names = {1:"UNKNOWN", 2:"ACCEPTED", 3:"CANCELING", 4:"SUCCEEDED", 5:"CANCELED", 6:"ABORTED"}
        for item in msg.status_list:
            row = self._common() | {"goal_uuid": bytes(item.goal_info.goal_id.uuid).hex(),
                                    "status": int(item.status), "status_name": names.get(int(item.status), "UNKNOWN")}
            _append(self.status_file, row)

    def _write_timeline(self, name, row):
        if self.evidence_sealed:
            return
        _append(self.out / name, row)

    def _raw(self, msg): self._write_timeline("raw_cmd_timeline.jsonl", self._common() | _twist_values(msg))
    def _safe(self, msg): self._write_timeline("safe_cmd_timeline.jsonl", self._common() | _twist_values(msg))
    def _vehicle(self, msg): self._write_timeline("vehicle_cmd_safe_timeline.jsonl", self._common() | _twist_values(msg))
    def _mock(self, msg): self._write_timeline("adapter_output_timeline.jsonl", self._common() | {"values": list(msg.data)})
    def _odom(self, msg):
        self._write_timeline("odometry_timeline.jsonl", self._common() | {"x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y, "yaw": _yaw(msg.pose.pose.orientation),
            "linear_x": msg.twist.twist.linear.x, "angular_z": msg.twist.twist.angular.z})

    def _diagnostics(self, msg):
        if self.evidence_sealed:
            return
        for status in msg.status:
            if status.name in ("vehicle_cmd_safety/guarded_vehicle_cmd_gate", "wheelchair_cmd_adapter/diagnostics", "phase4_vehicle_cmd_fake_base"):
                _append(self.diag_file, self._common() | {"name": status.name, "level": _byte_int(status.level),
                    "message": status.message, "values": {v.key: v.value for v in status.values}})

    def _drain_timer(self):
        if self.failed_received_ns is None or self.finalized or self.evidence_sealed:
            return
        if time.monotonic_ns() < self.failed_received_ns + int(POST_FAILED_DRAIN_SEC * 1e9):
            return
        self.drain_end_ns = time.monotonic_ns()
        self._commit()
        self.finalized = True

    def _commit(self):
        if self.evidence_sealed or self.failed_received_ns is None:
            return False
        if time.monotonic_ns() < self.failed_received_ns + int(POST_FAILED_DRAIN_SEC * 1e9):
            return False
        self.drain_end_ns = time.monotonic_ns()
        # This is the final non-self-referential event.  It is written while
        # the single-threaded executor is in this callback, before sealing.
        _append(self.phase, {"event": "TERMINAL_WITNESS_EVIDENCE_SEALED",
                             "reason": self.reason, "terminal_identity": dict(self.identity),
                             "FAILED_RECEIVED_NS": self.failed_received_ns,
                             "DRAIN_START_NS": self.drain_start_ns,
                             "DRAIN_END_NS": self.drain_end_ns,
                             "ACTUAL_DRAIN_SEC": (self.drain_end_ns - self.drain_start_ns) / 1e9,
                             "terminal_evidence_complete": True,
                             "physical_evidence_complete": True})
        # The executor is single-threaded: no subscription callback can run
        # concurrently with this sealing callback.  From this assignment
        # onward every callback fails closed without writing a retained file.
        self.evidence_sealed = True
        files = [p for p in self.out.iterdir() if p.is_file() and p.name not in (
            "TERMINAL_WITNESS_OUTCOME_COMMITTED", "terminal_witness_outcome.json")]
        hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        terminal_required = (self.mission_file, self.status_file, self.phase)
        physical_required = tuple(self.out / name for name in (
            "raw_cmd_timeline.jsonl", "safe_cmd_timeline.jsonl",
            "vehicle_cmd_safe_timeline.jsonl", "adapter_output_timeline.jsonl",
            "odometry_timeline.jsonl", "gate_diagnostics.jsonl"))
        result = {"terminal_identity": dict(self.identity), "reason": self.reason,
            "FAILED_RECEIVED_NS": self.failed_received_ns, "DRAIN_START_NS": self.drain_start_ns,
            "DRAIN_END_NS": self.drain_end_ns,
            "ACTUAL_DRAIN_SEC": (self.drain_end_ns - self.drain_start_ns) / 1e9,
            "file_hashes": hashes, "APPLICATION_CONTROL_AUTHORITY_COUNT": 0,
            "terminal_evidence_complete": all(p.is_file() and p.stat().st_size > 0 for p in terminal_required),
            "physical_evidence_complete": all(p.is_file() and p.stat().st_size > 0 for p in physical_required)}
        _fsync_json(self.out / "terminal_witness_outcome.json", result)
        _fsync_json(self.out / "TERMINAL_WITNESS_OUTCOME_COMMITTED", result)
        self.finalized = True
        self.stop_requested = True
        return True

    def spin(self):
        while self.rclpy.ok() and not self.stop_requested:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def shutdown(self):
        if not self.finalized and self.failed_received_ns is not None:
            # An interrupted drain is forensic-only; never manufacture a
            # committed qualification outcome before the full drain.
            self._commit()
        self.node.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--reason")
    parser.add_argument("--mission-id")
    parser.add_argument("--route-id")
    parser.add_argument("--waypoint", type=int)
    args = parser.parse_args(argv)
    import rclpy
    rclpy.init()
    witness = TerminalClosureWitness(Path(args.output_dir), args.reason, args.mission_id, args.route_id, args.waypoint)
    signal.signal(signal.SIGTERM, lambda *_: setattr(witness, "stop_requested", True))
    signal.signal(signal.SIGINT, lambda *_: setattr(witness, "stop_requested", True))
    try:
        witness.spin()
    finally:
        witness.shutdown()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
