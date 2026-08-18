"""Qualification-only ROS observer for P4-E.6C live evidence.

This node observes the production graph and writes evidence; it does not
publish policy, action, cancel, Gate, or command decisions.  It is deliberately
small: production Mission Manager remains the sole progress authority.
"""
from __future__ import annotations

import os
import time
from numbers import Integral
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from parking_robot_interfaces.msg import MissionState
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from std_msgs.msg import String

from .phase4_p4e6c_progress_recorder import ProgressEvidenceRecorder


def _status_values(msg: DiagnosticStatus) -> dict[str, str]:
    return {str(item.key): str(item.value) for item in msg.values}


def parse_bool(value: Any, *, field: str) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text == "true":
        return True
    if text == "false":
        return False
    raise RuntimeError("P4E6C_BOOL_MALFORMED:" + field)


def parse_int(value: Any, *, field: str) -> int:
    if isinstance(value, bool):
        raise RuntimeError("P4E6C_INT_MALFORMED:" + field)
    try:
        return int(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise RuntimeError("P4E6C_INT_MALFORMED:" + field) from exc


def optional_number(value: Any, *, field: str) -> float | None:
    if value is None or str(value).strip().lower() in {"", "none", "n/a", "not_available"}:
        return None
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError("P4E6C_NUMBER_MALFORMED:" + field) from exc


def normalize_uint8(value: Any, *, field: str) -> int:
    """Normalize ROS-generated uint8 values without coercing arbitrary text."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise RuntimeError("P4E6C_UINT8_MALFORMED:" + field)
        return int(value[0])
    if isinstance(value, Integral) and not isinstance(value, bool):
        normalized = int(value)
        if 0 <= normalized <= 255:
            return normalized
    raise RuntimeError("P4E6C_UINT8_MALFORMED:" + field)


def normalize_progress_diagnostic(values: dict[str, Any], *, state: dict[str, Any],
                                  now_ns: int) -> dict[str, Any]:
    """Convert the production Mission Manager DiagnosticStatus schema once."""
    event = str(values.get("progress_supervisor_event", ""))
    policy = values.get("progress_supervisor_mode") == "ACTIVE_POLICY"
    ages = {name: optional_number(values.get(name + "_age_sec"), field=name + "_age_sec")
            for name in ("odometry", "tf", "feedback", "gate", "collision_monitor",
                         "localization", "controller", "adapter", "raw_command", "safe_command")}
    command_state = str(values.get("command_pair_state", ""))
    collision = str(values.get("collision_classification", ""))
    progress = str(values.get("progress_classification", ""))
    health_current = (ages["gate"] is not None and ages["gate"] <= .50
                      and ages["localization"] is not None and ages["localization"] <= .50
                      and ages["controller"] is not None and ages["controller"] <= .50
                      and ages["collision_monitor"] is not None and ages["collision_monitor"] <= .50
                      and (ages["adapter"] is None or ages["adapter"] <= .50))
    row = {
        "sequence": int(state.get("sequence", 0)), "monotonic_ns": int(now_ns),
        "mission_id": state.get("mission_id", values.get("mission_id", "")),
        "route_id": state.get("route_id", values.get("route_id", "")),
        "active_goal_uuid": state.get("active_goal_uuid", values.get("active_goal_uuid", "")),
        "odom_current": ages["odometry"] is not None and ages["odometry"] <= .50,
        "tf_current": ages["tf"] is not None and ages["tf"] <= .50,
        "feedback_current": ages["feedback"] is not None and ages["feedback"] <= 2.0,
        "health_current": health_current,
        "command_pair_current": command_state in {"VALID", "PENDING"} and ages["raw_command"] is not None,
        "policy_active": policy and str(values.get("progress_policy_activation_state", "")) == "ACTIVE"
                         and parse_bool(values.get("progress_policy_activation_latched", False), field="progress_policy_activation_latched"),
        "collision_clear": collision == "CLEAR",
        "movement_intent": parse_bool(values.get("movement_intent", state.get("movement_intent", False)), field="movement_intent"),
        "action_terminal": str(values.get("state", state.get("state", ""))) in {"CANCELLED", "SUCCEEDED", "BLOCKED", "FAILED"},
        "recovery_delta": parse_int(values.get("recovery_delta"), field="recovery_delta"),
        "measurable_progress": progress == "MEASURABLE",
        "progress_event": event,
        "no_progress_age_sec": optional_number(values.get("no_progress_age_sec"), field="no_progress_age_sec"),
        "competing_reason": (None if event in {"", "NO_PROGRESS_PENDING", "PROGRESSING", "CONTROLLER_NO_PROGRESS",
                                                  "RECOVERY_EXHAUSTED_NO_PROGRESS"} else event),
    }
    if not policy:
        row["policy_active"] = False
    return row


class ProgressEvidenceObserver(Node):
    def __init__(self) -> None:
        super().__init__("phase4_p4e6c_progress_observer")
        self.case = str(self.declare_parameter("case_id", "C-P01").value).upper()
        output = os.environ.get("P4E6C_EVIDENCE_OUTPUT")
        if not output:
            raise RuntimeError("P4E6C_EVIDENCE_OUTPUT_MISSING")
        self.output = __import__("pathlib").Path(output)
        self.recorder = ProgressEvidenceRecorder(self.output, self.case,
                                                 self._durable_write)
        self.latest: dict[str, Any] = {}
        self._sequence = 0
        self.create_subscription(MissionState, "/mission/state", self._state, 20)
        self.create_subscription(DiagnosticStatus, "/mission/status", self._status, 20)
        self.create_subscription(DiagnosticStatus, "/mission/block_reason", self._block, 20)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._action, 20)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage,
                                 "/navigate_to_pose/_action/feedback", self._feedback, 20)
        self.create_subscription(Odometry, "/Odometry", self._odom, 20)
        self.create_subscription(TFMessage, "/tf", self._tf, 20)
        self.create_subscription(Twist, "/cmd_vel_nav_raw", self._raw, 20)
        self.create_subscription(Twist, "/cmd_vel_nav_safe", self._safe, 20)
        self.create_subscription(TwistStamped, "/vehicle_cmd_safe", self._applied, 20)
        self.create_subscription(DiagnosticStatus, "/vehicle_cmd_safety/state", self._gate, 20)
        self.create_subscription(Bool, "/system/collision_monitor_valid", self._collision, 20)
        self.create_subscription(DiagnosticArray, "/wheelchair_cmd_adapter/diagnostics", self._health, 20)
        self.create_subscription(String, "/phase4_qualification/p4e6c/recovery_feedback_relay_state",
                                 self._relay_state, 20)

    def _durable_write(self, path, value) -> None:
        from .phase4_p4e6c_controller import ProgressQualificationController
        ProgressQualificationController(self.case, self.output)._write_json(path, value)

    def _row(self, filename: str, **values: Any) -> None:
        self.recorder.record(filename, {
            "transaction_id": os.environ.get("P4E6C_ATTEMPT_IDENTITY", ""),
            "observation_monotonic_ns": time.monotonic_ns(), **values})

    def _state(self, msg: MissionState) -> None:
        self.latest.update(mission_id=msg.mission_id, route_id=msg.route_id,
                           active_goal_uuid=msg.active_goal_uuid,
                           mission_active=msg.state == MissionState.NAVIGATING,
                           action_terminal=msg.state in (MissionState.CANCELLED,
                                                         MissionState.SUCCEEDED,
                                                         MissionState.BLOCKED,
                                                         MissionState.FAILED))
        self._row("mission_state_events.jsonl", mission_id=msg.mission_id,
                  route_id=msg.route_id, active_goal_uuid=msg.active_goal_uuid,
                  state=int(msg.state), reason_code=msg.reason_code)

    def _status(self, msg: DiagnosticStatus) -> None:
        values = _status_values(msg)
        self.latest.update(values)
        self._row("mission_policy_diagnostics.jsonl",
                  level=normalize_uint8(msg.level, field="diagnostic_level"),
                  name=msg.name, values=values)
        try:
            normalized = normalize_progress_diagnostic(values, state=self.latest,
                                                       now_ns=time.monotonic_ns())
            self._sequence += 1; normalized["sequence"] = self._sequence
            self.latest["normalized_progress"] = normalized
            self.latest["policy_active"] = normalized["policy_active"]
            self.latest["recovery_delta"] = normalized["recovery_delta"]
            self.latest["measurable_progress"] = normalized["measurable_progress"]
            self.latest["progress_event"] = normalized["progress_event"]
            self._row("progress_events.jsonl", **normalized)
        except RuntimeError:
            pass

    def _block(self, msg: DiagnosticStatus) -> None:
        self._row("block_cancel_ack_events.jsonl", name=msg.name,
                  values=_status_values(msg))

    def _action(self, msg: GoalStatusArray) -> None:
        self._row("navigate_action_status_events.jsonl", statuses=[
            {"goal_id": [int(value) for value in item.goal_info.goal_id.uuid],
             "status": normalize_uint8(item.status, field="goal_status")}
            for item in msg.status_list])

    def _feedback(self, msg) -> None:
        self.latest["feedback_current"] = True
        self.latest["feedback_receipt_monotonic_ns"] = time.monotonic_ns()
        pose = msg.feedback.current_pose.pose.position
        self._row("feedback_receipts.jsonl", source="navigate_feedback",
                  goal_uuid=[int(value) for value in msg.goal_id.uuid],
                  navigation_time_sec=float(msg.feedback.navigation_time.sec),
                  estimated_time_remaining_sec=float(msg.feedback.estimated_time_remaining.sec),
                  number_of_recoveries=int(msg.feedback.number_of_recoveries),
                  distance_remaining=float(msg.feedback.distance_remaining),
                  current_pose={"x": float(pose.x), "y": float(pose.y), "z": float(pose.z)})

    def _odom(self, msg: Odometry) -> None:
        self.latest["odom_current"] = True
        self.latest["odom_receipt_monotonic_ns"] = time.monotonic_ns()
        q = msg.pose.pose.orientation
        import math
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._row("physical_evidence.jsonl", source="odometry",
                  x=float(msg.pose.pose.position.x), y=float(msg.pose.pose.position.y), yaw=yaw)

    def _tf(self, msg: TFMessage) -> None:
        self.latest["tf_current"] = True
        self.latest["tf_receipt_monotonic_ns"] = time.monotonic_ns()

    def _raw(self, msg: Twist) -> None:
        self.latest["command_pair_current"] = True
        self.latest["movement_intent"] = abs(float(msg.linear.x)) > 1e-6 or abs(float(msg.angular.z)) > 1e-6
        receipt_ns = time.monotonic_ns()
        self.latest["raw_receipt_monotonic_ns"] = receipt_ns
        self._row("physical_evidence.jsonl", source="raw_command",
                  linear_x=float(msg.linear.x), linear_y=float(msg.linear.y),
                  linear_z=float(msg.linear.z), angular_x=float(msg.angular.x),
                  angular_y=float(msg.angular.y), angular_z=float(msg.angular.z),
                  movement_intent=self.latest["movement_intent"],
                  monotonic_ns=receipt_ns)

    def _safe(self, msg: Twist) -> None:
        self.latest["command_pair_current"] = True
        self._row("physical_evidence.jsonl", source="canonical_command",
                  linear_x=float(msg.linear.x), linear_y=float(msg.linear.y),
                  linear_z=float(msg.linear.z), angular_x=float(msg.angular.x),
                  angular_y=float(msg.angular.y), angular_z=float(msg.angular.z))

    def _applied(self, msg: TwistStamped) -> None:
        self._row("physical_evidence.jsonl", source="applied_command",
                  linear_x=float(msg.twist.linear.x), linear_y=float(msg.twist.linear.y),
                  linear_z=float(msg.twist.linear.z), angular_x=float(msg.twist.angular.x),
                  angular_y=float(msg.twist.angular.y), angular_z=float(msg.twist.angular.z))

    def _gate(self, msg: DiagnosticStatus) -> None:
        values = _status_values(msg); self.latest.update(values)
        self.latest["gate_armed"] = values.get("state") == "ARMED"
        self._row("gate_state_events.jsonl", source="gate_diagnostic",
                  state=values.get("state", ""),
                  mission_id=self.latest.get("mission_id", ""),
                  route_id=self.latest.get("route_id", ""),
                  active_goal_uuid=self.latest.get("active_goal_uuid", ""),
                  gate_armed=self.latest["gate_armed"],
                  values=values)

    def _collision(self, msg: Bool) -> None:
        self.latest["collision_clear"] = bool(msg.data)

    def _health(self, msg: DiagnosticArray) -> None:
        try:
            levels = [normalize_uint8(item.level, field="diagnostic_level") for item in msg.status]
        except RuntimeError:
            self.latest["health_current"] = False
            return
        error_level = normalize_uint8(DiagnosticStatus.ERROR, field="diagnostic_error_level")
        self.latest["health_current"] = bool(levels) and all(level < error_level for level in levels)

    def _relay_state(self, msg: String) -> None:
        import json
        payload = json.loads(msg.data)
        row = {"transaction_id": os.environ.get("P4E6C_ATTEMPT_IDENTITY", ""),
               "sequence": self._sequence + 1, "monotonic_ns": time.monotonic_ns(),
               "goal_uuid": self.latest.get("active_goal_uuid") or payload.get("target_goal_uuid", []),
               "canonical_recovery_count": payload.get("latest_canonical_recovery_count"),
               "shadow_recovery_count": payload.get("latest_shadow_recovery_count"),
               "relay_state": payload.get("state"),
               "injection_count": payload.get("injection_count", 0),
               "sequence_complete": payload.get("sequence_complete", False),
               "mission_id": self.latest.get("mission_id", ""),
               "route_id": self.latest.get("route_id", ""),
               "active_goal_uuid": self.latest.get("active_goal_uuid", ""),
               "measurable_progress": self.latest.get("measurable_progress", False),
               "progress_event": self.latest.get("progress_event", ""),
               "actual_reason": self.latest.get("progress_event", "")}
        self._sequence += 1; row["sequence"] = self._sequence
        self.recorder.record_recovery(row)
        self.latest["recovery_relay_ready"] = (
            payload.get("state") == "READY_FORWARD" and bool(payload.get("healthy"))
            and int(payload.get("injection_count", 0)) == 0
            and int(payload.get("latest_canonical_recovery_count") or 0) == 0)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProgressEvidenceObserver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
