"""ROS transport adapter for the typed Phase 3 Mission Manager."""

from __future__ import annotations

from dataclasses import replace
from enum import Enum, auto
from functools import partial
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from parking_robot_interfaces.msg import MissionState, RouteMission
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .mission_progress_observation_adapter import (
    CausalCommandPairer,
    CausalPairState,
    CommandStreamPhase,
    adapter_health_sample,
    bool_health_sample,
    command_sample,
    feedback_sample,
    gate_health_sample,
    odometry_sample,
    transform_sample,
    transform_stamp_key,
)
from .mission_progress_supervisor_core import (
    CommandSample,
    GateState,
    MissionProgressSupervisorCore,
    SupervisorEvent,
    SupervisorInputs,
)

from .mission_state_machine import (
    GoalOutcome,
    GoalResultCode,
    MissionGoalExecutor,
    MissionSnapshot,
    MissionStateCode,
    MissionStateMachine,
)


def _status_name(status: int) -> str:
    names = {
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
    }
    return names.get(status, f"STATUS_{status}")


def _goal_result_code(status: int) -> GoalResultCode:
    if status == GoalStatus.STATUS_SUCCEEDED:
        return GoalResultCode.SUCCEEDED
    if status == GoalStatus.STATUS_CANCELED:
        return GoalResultCode.CANCELED
    return GoalResultCode.ABORTED


class ProgressPolicyActivationState(Enum):
    NOT_STARTED = auto()
    INITIAL_PRIMING = auto()
    ACTIVE = auto()


INITIAL_MISSION_ACTIVATION_DEADLINE_SEC = 2.0
INITIAL_COMMAND_ACQUISITION_TIMEOUT_SEC = 1.0


class _InitialCommandAcquisitionEvent(Enum):
    INITIAL_COMMAND_ACQUISITION_TIMEOUT = auto()


class _NavigateToPoseTransport(MissionGoalExecutor):
    """Action transport used by MissionManagerNode.

    This class owns ROS action handles and Futures. It never owns mission
    state; every state-visible event is forwarded to MissionStateMachine.
    """

    def __init__(self, node: "MissionManagerNode", action_name: str) -> None:
        self.node = node
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.goal_handles: Dict[str, object] = {}
        self.futures: List[object] = []
        self.cancel_request_count = 0

    def server_available(self) -> bool:
        return self.client.server_is_ready()

    def send_goal(self, pose, result_callback) -> GoalOutcome:
        del result_callback
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.header.frame_id = pose.frame_id
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        goal.pose.pose.position.z = pose.z
        goal.pose.pose.orientation.x = pose.qx
        goal.pose.pose.orientation.y = pose.qy
        goal.pose.pose.orientation.z = pose.qz
        goal.pose.pose.orientation.w = pose.qw
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        self.futures.append(future)
        future.add_done_callback(partial(self._goal_response_cb, waypoint_id=str(getattr(pose, "waypoint_id", ""))))
        return GoalOutcome(True, reason_code="GOAL_PENDING", detail="goal request sent")

    def cancel_goal(self, goal_uuid: str, timeout_sec: float) -> bool:
        del timeout_sec
        goal_handle = self.goal_handles.get(goal_uuid)
        if goal_handle is None:
            return False
        self.cancel_request_count += 1
        future = goal_handle.cancel_goal_async()
        self.futures.append(future)
        future.add_done_callback(self._cancel_response_cb)
        return True

    def prune_futures(self) -> None:
        self.futures = [future for future in self.futures if not future.done()]

    def _goal_response_cb(self, future, waypoint_id: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.apply_core_event(
                lambda core: core.on_goal_rejected("GOAL_RESPONSE_EXCEPTION", f"{type(exc).__name__}: {exc}")
            )
            return
        if not goal_handle.accepted:
            self.node.apply_core_event(lambda core: core.on_goal_rejected("GOAL_REJECTED", f"waypoint {waypoint_id} rejected"))
            return
        goal_uuid = bytes(goal_handle.goal_id.uuid).hex()
        self.goal_handles[goal_uuid] = goal_handle
        self.node.apply_core_event(lambda core: core.on_goal_accepted(goal_uuid))
        self.node.accept_progress_goal(goal_uuid)
        result_future = goal_handle.get_result_async()
        self.futures.append(result_future)
        result_future.add_done_callback(partial(self._result_cb, goal_uuid=goal_uuid))

    def _feedback_cb(self, feedback_msg) -> None:
        steady_receipt_sec = self.node.steady_now()
        try:
            goal_uuid = bytes(feedback_msg.goal_id.uuid).hex()
        except (AttributeError, TypeError, ValueError):
            return
        self.node.receive_nav_feedback(goal_uuid, feedback_msg, steady_receipt_sec)

    def _result_cb(self, future, goal_uuid: str) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self.node.apply_core_event(
                lambda core: core.on_goal_result(GoalResultCode.ABORTED, f"RESULT_EXCEPTION: {type(exc).__name__}: {exc}")
            )
            return
        status = int(wrapped.status)
        self.goal_handles.pop(goal_uuid, None)
        self.node.finish_progress_goal(goal_uuid)
        self.node.apply_core_event(lambda core: core.on_goal_result(_goal_result_code(status), _status_name(status)))

    def _cancel_response_cb(self, future) -> None:
        try:
            response = future.result()
        except Exception:
            self.node.apply_core_event(lambda core: core.on_cancel_response_rejected())
            return
        if getattr(response, "goals_canceling", []):
            self.node.apply_core_event(lambda core: core.on_cancel_response_accepted())
        else:
            self.node.apply_core_event(lambda core: core.on_cancel_response_rejected())


class MissionManagerNode(Node):
    """Sequential typed RouteMission executor.

    This node publishes no Twist or velocity-equivalent command. Navigation is
    delegated exclusively to the standard NavigateToPose action interface.
    """

    def __init__(
        self,
        *,
        goal_executor: Optional[MissionGoalExecutor] = None,
        steady_clock=time.monotonic,
        parameter_overrides=None,
    ) -> None:
        super().__init__("mission_manager", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("mission_topic", "/mission/route")
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("expected_topology_version", "v1")
        self.declare_parameter("goal_xy_tolerance_m", 0.25)
        self.declare_parameter("waypoint_separation_margin_m", 0.05)
        self.declare_parameter("min_waypoint_separation_m", 0.55)
        self.declare_parameter("cancel_response_timeout_sec", 2.0)
        self.declare_parameter("cancel_result_timeout_sec", 5.0)

        self._lock = threading.RLock()
        self._steady_clock = steady_clock
        self._initial_command_acquisition_timeout_sec = (
            INITIAL_COMMAND_ACQUISITION_TIMEOUT_SEC
        )

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(MissionState, str(self.get_parameter("state_topic").value), state_qos)
        self._status_pub = self.create_publisher(DiagnosticStatus, "/mission/status", state_qos)
        self._block_reason_pub = self.create_publisher(DiagnosticStatus, "/mission/block_reason", state_qos)
        self._mission_sub = self.create_subscription(
            RouteMission,
            str(self.get_parameter("mission_topic").value),
            self._mission_cb,
            QoSProfile(depth=10),
        )
        self._start_srv = self.create_service(Trigger, "/mission/start", self._start_cb)
        self._cancel_srv = self.create_service(Trigger, "/mission/cancel", self._cancel_cb)
        self._pause_srv = self.create_service(SetBool, "/mission/pause", self._pause_cb)

        self._transport = goal_executor
        if self._transport is None:
            self._transport = _NavigateToPoseTransport(self, str(self.get_parameter("navigate_to_pose_action").value))
        self._action_client = getattr(self._transport, "client", None)

        self._latest_snapshot: Optional[MissionSnapshot] = None
        self._progress_supervisor = MissionProgressSupervisorCore()
        self._active_progress_goal_uuid = ""
        self._progress_policy_activation_state = ProgressPolicyActivationState.NOT_STARTED
        self._progress_policy_activation_start_sec = None
        self._progress_policy_activated_sec = None
        self._progress_goal_reset_count = 0
        self._latest_feedback = None
        self._latest_odometry = None
        self._latest_transform = None
        self._latest_raw_command = None
        self._latest_safe_command = None
        self._causal_raw_command = None
        self._causal_safe_command = None
        self._command_pairer = CausalCommandPairer(
            freshness_sec=self._progress_supervisor.thresholds.command_pair_freshness_sec)
        self._latest_gate = None
        self._latest_collision_valid = None
        self._latest_localization_valid = None
        self._latest_controller_valid = None
        self._latest_adapter_health = None
        self._latest_passive_result = None
        self._passive_result_pending_apply = False
        self._published_block_reason = ""
        self._latest_tf_stamp_key = None
        self._feedback_receipt_times = []

        self.create_subscription(Odometry, "/Odometry", self._odometry_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_nav_raw", self._raw_command_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_nav_safe", self._safe_command_cb, 50)
        self.create_subscription(DiagnosticStatus, "/vehicle_cmd_safety/state", self._gate_state_cb, 50)
        self.create_subscription(Bool, "/system/collision_monitor_valid", self._collision_valid_cb, 50)
        self.create_subscription(Bool, "/system/localization_valid", self._localization_valid_cb, 50)
        self.create_subscription(Bool, "/system/controller_valid", self._controller_valid_cb, 50)
        self.create_subscription(DiagnosticArray, "/wheelchair_cmd_adapter/diagnostics", self._adapter_health_cb, 50)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._core = MissionStateMachine(
            self._transport,
            expected_topology_version=str(self.get_parameter("expected_topology_version").value),
            state_callback=self._publish_snapshot,
            goal_xy_tolerance_m=float(self.get_parameter("goal_xy_tolerance_m").value),
            waypoint_separation_margin_m=float(self.get_parameter("waypoint_separation_margin_m").value),
            min_waypoint_separation_m=float(self.get_parameter("min_waypoint_separation_m").value),
            cancel_response_timeout_sec=float(self.get_parameter("cancel_response_timeout_sec").value),
            cancel_result_timeout_sec=float(self.get_parameter("cancel_result_timeout_sec").value),
            steady_clock=self._steady_clock,
        )
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_cb)

    @property
    def mission_state_machine(self) -> MissionStateMachine:
        return self._core

    def apply_core_event(self, callback) -> None:
        with self._lock:
            callback(self._core)

    def steady_now(self) -> float:
        return float(self._steady_clock())

    def accept_progress_goal(self, goal_uuid: str) -> None:
        with self._lock:
            if not goal_uuid or goal_uuid == self._active_progress_goal_uuid:
                return
            now = self.steady_now()
            self._active_progress_goal_uuid = goal_uuid
            self._command_pairer.start_epoch(now)
            self._causal_raw_command = None
            self._causal_safe_command = None
            self._latest_passive_result = None
            self._passive_result_pending_apply = False
            if self._progress_policy_activation_state is ProgressPolicyActivationState.NOT_STARTED:
                self._progress_policy_activation_state = ProgressPolicyActivationState.INITIAL_PRIMING
                self._progress_policy_activation_start_sec = now
                self._progress_policy_activated_sec = None
            self._latest_feedback = None
            pose_sample = self._latest_transform or self._latest_odometry
            pose_xy = None if pose_sample is None else (pose_sample.x, pose_sample.y)
            self._progress_supervisor.accept_new_goal(now, recovery_baseline=0, pose_xy=pose_xy)
            self._progress_goal_reset_count += 1

    def receive_nav_feedback(self, goal_uuid: str, msg, steady_receipt_sec: float) -> None:
        with self._lock:
            if not goal_uuid or goal_uuid != self._active_progress_goal_uuid:
                return
            sample = feedback_sample(msg, steady_receipt_sec)
            self._latest_feedback = sample
            if sample is not None:
                self._feedback_receipt_times.append(sample.steady_receipt_sec)

    def finish_progress_goal(self, goal_uuid: str) -> None:
        with self._lock:
            if goal_uuid == self._active_progress_goal_uuid:
                self._active_progress_goal_uuid = ""
                self._latest_passive_result = None
                self._passive_result_pending_apply = False
                self._command_pairer.end_epoch(self.steady_now())
                self._causal_raw_command = None
                self._causal_safe_command = None

    def _odometry_cb(self, msg) -> None:
        now = self.steady_now()
        self._latest_odometry = odometry_sample(msg, now)

    def _raw_command_cb(self, msg) -> None:
        with self._lock:
            now = self.steady_now()
            self._latest_raw_command = command_sample(msg, now)
            if self._latest_raw_command is not None:
                self._command_pairer.observe_raw(self._latest_raw_command)
                # A newer raw starts a new causal acquisition.  Never reuse the
                # prior pair while its Collision Monitor output is outstanding.
                self._causal_raw_command = None
                self._causal_safe_command = None

    def _safe_command_cb(self, msg) -> None:
        with self._lock:
            now = self.steady_now()
            self._latest_safe_command = command_sample(msg, now)
            if self._latest_safe_command is None:
                return
            prior_state = self._command_pairer.current.state
            result = self._command_pairer.observe_safe(self._latest_safe_command)
            if result.state is CausalPairState.VALID and not result.drain_only:
                self._causal_raw_command = CommandSample(
                    result.raw.steady_receipt_sec, result.raw.linear_x, result.raw.angular_z)
                self._causal_safe_command = CommandSample(
                    result.safe.steady_receipt_sec, result.safe.linear_x, result.safe.angular_z)
                self._evaluate_progress_passively(now)
                self._apply_pending_progress_result(now)
            else:
                self._causal_raw_command = None
                self._causal_safe_command = None
                if (result.state is CausalPairState.AMBIGUOUS
                        and prior_state is not CausalPairState.AMBIGUOUS):
                    self._evaluate_progress_passively(now)

    def _gate_state_cb(self, msg) -> None:
        now = self.steady_now()
        self._latest_gate = gate_health_sample(msg, now)

    def _collision_valid_cb(self, msg) -> None:
        now = self.steady_now()
        self._latest_collision_valid = bool_health_sample(msg, now)

    def _localization_valid_cb(self, msg) -> None:
        now = self.steady_now()
        self._latest_localization_valid = bool_health_sample(msg, now)

    def _controller_valid_cb(self, msg) -> None:
        now = self.steady_now()
        self._latest_controller_valid = bool_health_sample(msg, now)

    def _adapter_health_cb(self, msg) -> None:
        now = self.steady_now()
        self._latest_adapter_health = adapter_health_sample(msg, now)

    def _observe_transform(self, steady_receipt_sec: float) -> None:
        try:
            msg = self._tf_buffer.lookup_transform(
                "odom", "base_footprint", Time(), timeout=Duration(seconds=0.0)
            )
        except TransformException:
            return
        stamp_key = transform_stamp_key(msg)
        if stamp_key is None or stamp_key == self._latest_tf_stamp_key:
            return
        sample = transform_sample(msg, steady_receipt_sec)
        if sample is not None:
            self._latest_tf_stamp_key = stamp_key
            self._latest_transform = sample

    def _evaluate_progress_passively(self, now: float) -> None:
        if not self._active_progress_goal_uuid:
            return
        inputs = SupervisorInputs(
            feedback=self._latest_feedback,
            odometry=self._latest_odometry,
            transform=self._latest_transform,
            raw_command=self._causal_raw_command,
            safe_command=self._causal_safe_command,
            gate=self._latest_gate,
            collision_monitor_valid=self._latest_collision_valid,
            localization_valid=self._latest_localization_valid,
            controller_valid=self._latest_controller_valid,
            adapter_health=self._latest_adapter_health,
        )
        try:
            result = self._progress_supervisor.evaluate(now, inputs)
            if (result.primary_event is SupervisorEvent.COMMAND_PAIR_STALE
                    and self._command_pairer.pending_within_budget(now)):
                # The evaluation deliberately breaks any STOP/CLEAR continuity,
                # but bounded causal acquisition is not yet a health failure.
                self._latest_passive_result = None
                self._passive_result_pending_apply = False
            else:
                self._latest_passive_result = result
                self._passive_result_pending_apply = True
        except ValueError:
            self._latest_passive_result = None
            self._passive_result_pending_apply = False

    def _apply_pending_progress_result(self, now: float) -> None:
        if not self._passive_result_pending_apply or self._latest_passive_result is None:
            return
        policy_result = self._progress_policy_result_to_apply(self._latest_passive_result, now)
        self._passive_result_pending_apply = False
        if policy_result is not None:
            self._core.apply_progress_supervisor_result(policy_result)

    def _progress_policy_activation_age(self, now: float) -> Optional[float]:
        if self._progress_policy_activation_start_sec is None:
            return None
        return max(0.0, now - self._progress_policy_activation_start_sec)

    @staticmethod
    def _sample_age(now: float, sample) -> Optional[float]:
        if sample is None:
            return None
        return now - float(sample.steady_receipt_sec)

    def _sample_fresh(self, now: float, sample, freshness_sec: float) -> bool:
        age = self._sample_age(now, sample)
        return age is not None and 0.0 <= age <= freshness_sec

    def _progress_policy_ready_for_activation(self, now: float) -> bool:
        thresholds = self._progress_supervisor.thresholds
        if not self._sample_fresh(now, self._latest_feedback, thresholds.feedback_freshness_sec):
            return False
        if not self._sample_fresh(now, self._latest_odometry, thresholds.odometry_freshness_sec):
            return False
        if not self._sample_fresh(now, self._latest_transform, thresholds.tf_freshness_sec):
            return False
        if not self._sample_fresh(now, self._causal_raw_command, thresholds.command_pair_freshness_sec):
            return False
        if not self._sample_fresh(now, self._causal_safe_command, thresholds.command_pair_freshness_sec):
            return False
        if (abs(self._causal_raw_command.steady_receipt_sec - self._causal_safe_command.steady_receipt_sec)
                > thresholds.command_pair_freshness_sec):
            return False
        health_freshness = thresholds.odometry_freshness_sec
        for sample in (self._latest_collision_valid, self._latest_localization_valid,
                       self._latest_controller_valid):
            if not self._sample_fresh(now, sample, health_freshness) or not sample.value:
                return False
        gate = self._latest_gate
        if not self._sample_fresh(now, gate, health_freshness):
            return False
        return gate.state is GateState.ARMED and not gate.fault_latched

    def _explicit_failure_event(self, now: float):
        thresholds = self._progress_supervisor.thresholds
        health_freshness = thresholds.odometry_freshness_sec
        gate = self._latest_gate
        if gate is not None:
            if (not self._sample_fresh(now, gate, health_freshness)
                    or gate.fault_latched or gate.state is GateState.FAULT):
                return SupervisorEvent.GATE_FAULT
            if (gate.state is GateState.DISARMED
                    and (self._progress_policy_activation_age(now) or 0.0)
                    >= INITIAL_MISSION_ACTIVATION_DEADLINE_SEC):
                return SupervisorEvent.GATE_DISARMED
        for sample, event in (
            (self._latest_localization_valid, SupervisorEvent.LOCALIZATION_INVALID),
            (self._latest_controller_valid, SupervisorEvent.CONTROLLER_INVALID),
            (self._latest_collision_valid, SupervisorEvent.COLLISION_MONITOR_INVALID),
        ):
            if sample is not None and (
                    not self._sample_fresh(now, sample, health_freshness) or not sample.value):
                return event
        adapter = self._latest_adapter_health
        if adapter is not None and (
                not self._sample_fresh(now, adapter, health_freshness) or not adapter.valid):
            return SupervisorEvent.ADAPTER_INVALID
        for sample, freshness, event in (
            (self._latest_odometry, thresholds.odometry_freshness_sec, SupervisorEvent.ODOMETRY_STALE),
            (self._latest_transform, thresholds.tf_freshness_sec, SupervisorEvent.TF_STALE),
            (self._latest_feedback, thresholds.feedback_freshness_sec, SupervisorEvent.FEEDBACK_STALE),
        ):
            if sample is not None and not self._sample_fresh(now, sample, freshness):
                return event
        raw, safe = self._causal_raw_command, self._causal_safe_command
        if (self._command_pairer.command_stream_phase
                is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW):
            acquisition_age = self._command_pairer.no_raw_acquisition_age(now)
            if (acquisition_age is not None
                    and acquisition_age >= self._initial_command_acquisition_timeout_sec):
                return _InitialCommandAcquisitionEvent.INITIAL_COMMAND_ACQUISITION_TIMEOUT
            return None
        if raw is not None and safe is not None and (
                not self._sample_fresh(now, raw, thresholds.command_pair_freshness_sec)
                or not self._sample_fresh(now, safe, thresholds.command_pair_freshness_sec)
                or abs(raw.steady_receipt_sec - safe.steady_receipt_sec)
                > thresholds.command_pair_freshness_sec):
            return SupervisorEvent.COMMAND_PAIR_STALE
        if raw is None or safe is None:
            if not self._command_pairer.pending_within_budget(now):
                return SupervisorEvent.COMMAND_PAIR_STALE
        return None

    def _progress_policy_result_to_apply(self, result, now: float):
        explicit_event = self._explicit_failure_event(now)
        if self._progress_policy_activation_state is ProgressPolicyActivationState.ACTIVE:
            if explicit_event is _InitialCommandAcquisitionEvent.INITIAL_COMMAND_ACQUISITION_TIMEOUT:
                return replace(result, primary_event=explicit_event, reason=explicit_event.name)
            return result
        if self._progress_policy_activation_state is ProgressPolicyActivationState.NOT_STARTED:
            return None
        if explicit_event is not None:
            if result.primary_event is explicit_event:
                return result
            return replace(result, primary_event=explicit_event, reason=explicit_event.name)
        if self._progress_policy_ready_for_activation(now):
            self._progress_policy_activation_state = ProgressPolicyActivationState.ACTIVE
            self._progress_policy_activated_sec = now
            return result
        age = self._progress_policy_activation_age(now)
        if age is not None and age >= INITIAL_MISSION_ACTIVATION_DEADLINE_SEC:
            return result
        return None

    def _progress_policy_can_apply(self, result, now: float) -> bool:
        return self._progress_policy_result_to_apply(result, now) is not None

    def _reset_progress_policy_activation(self) -> None:
        self._progress_policy_activation_state = ProgressPolicyActivationState.NOT_STARTED
        self._progress_policy_activation_start_sec = None
        self._progress_policy_activated_sec = None
        self._command_pairer.reset()
        self._causal_raw_command = None
        self._causal_safe_command = None
        self._passive_result_pending_apply = False

    def _progress_policy_diagnostics(self, now: float) -> dict[str, str]:
        age = self._progress_policy_activation_age(now)
        acquisition_age = self._command_pairer.no_raw_acquisition_age(now)
        pair_liveness_age = self._command_pairer.pair_liveness_age(now)
        command_stream_phase = self._command_pairer.command_stream_phase
        return {
            "progress_policy_activation_state": self._progress_policy_activation_state.name,
            "progress_policy_activation_age_sec": "none" if age is None else f"{age:.6f}",
            "progress_policy_activation_latched": str(
                self._progress_policy_activation_state is ProgressPolicyActivationState.ACTIVE
            ).lower(),
            "progress_policy_activation_start_sec": (
                "none" if self._progress_policy_activation_start_sec is None
                else f"{self._progress_policy_activation_start_sec:.9f}"
            ),
            "command_stream_phase": (
                "NONE" if command_stream_phase is None else command_stream_phase.name
            ),
            "initial_command_acquisition_timeout_sec": (
                f"{self._initial_command_acquisition_timeout_sec:.6f}"
            ),
            "no_raw_acquisition_age_sec": (
                "none" if acquisition_age is None else f"{acquisition_age:.6f}"
            ),
            "pair_liveness_age_sec": (
                "none" if pair_liveness_age is None else f"{pair_liveness_age:.6f}"
            ),
        }

    def _publish_passive_status(self) -> None:
        snapshot = self._latest_snapshot
        result = self._latest_passive_result
        if snapshot is None or result is None:
            return
        diagnostic = DiagnosticStatus()
        diagnostic.name = "mission_manager"
        diagnostic.level = DiagnosticStatus.OK
        diagnostic.message = snapshot.reason_code or snapshot.detail or snapshot.state.name
        values = {
            "state": snapshot.state.name,
            "mission_id": snapshot.mission_id,
            "route_id": snapshot.route_id,
            "reason_code": snapshot.reason_code,
            "active_goal_uuid": snapshot.active_goal_uuid,
            "progress_supervisor_mode": "ACTIVE_POLICY",
            "progress_supervisor_event": result.primary_event.name,
            "progress_classification": result.progress.name,
            "collision_classification": result.collision.name,
            "command_pair_state": self._command_pairer.current.state.name,
            "recovery_delta": str(result.recovery_delta),
            "distance_remaining": "none" if self._latest_feedback is None else f"{self._latest_feedback.distance_remaining_m:.6f}",
        }
        values.update(self._progress_policy_diagnostics(self.steady_now()))
        for name, age in result.ages_sec.items():
            values[f"{name}_age_sec"] = "none" if age is None else f"{age:.6f}"
        samples = {
            "feedback": self._latest_feedback,
            "odometry": self._latest_odometry,
            "tf": self._latest_transform,
            "raw_command": self._causal_raw_command,
            "safe_command": self._causal_safe_command,
            "gate": self._latest_gate,
            "collision_monitor": self._latest_collision_valid,
            "localization": self._latest_localization_valid,
            "controller": self._latest_controller_valid,
            "adapter": self._latest_adapter_health,
        }
        for name, sample in samples.items():
            values[f"{name}_receipt_steady_sec"] = (
                "none" if sample is None else f"{sample.steady_receipt_sec:.9f}"
            )
        values["feedback_sample_count"] = str(len(self._feedback_receipt_times))
        values["number_of_recoveries"] = (
            "none" if self._latest_feedback is None else str(self._latest_feedback.number_of_recoveries)
        )
        if self._causal_raw_command is None or self._causal_safe_command is None:
            values["raw_safe_skew_sec"] = "none"
        else:
            values["raw_safe_skew_sec"] = f"{abs(self._causal_raw_command.steady_receipt_sec - self._causal_safe_command.steady_receipt_sec):.6f}"
        diagnostic.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        self._status_pub.publish(diagnostic)

    def _publish_snapshot(self, snapshot: MissionSnapshot) -> None:
        self._latest_snapshot = snapshot
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.mission_id = snapshot.mission_id
        msg.route_id = snapshot.route_id
        msg.state = int(snapshot.state)
        msg.current_waypoint_index = snapshot.current_waypoint_index
        msg.completed_waypoint_count = snapshot.completed_waypoint_count
        msg.total_waypoint_count = snapshot.total_waypoint_count
        msg.progress = float(snapshot.progress)
        msg.active_goal_uuid = snapshot.active_goal_uuid
        msg.reason_code = snapshot.reason_code
        msg.detail = snapshot.detail
        self._state_pub.publish(msg)

        diagnostic = DiagnosticStatus()
        diagnostic.name = "mission_manager"
        diagnostic.level = DiagnosticStatus.ERROR if snapshot.state in (
            MissionStateCode.FAILED,
            MissionStateCode.BLOCKED,
            MissionStateCode.HELP_REQUIRED,
        ) else DiagnosticStatus.OK
        diagnostic.message = snapshot.reason_code or snapshot.detail or snapshot.state.name
        diagnostic.values = [
            KeyValue(key="state", value=snapshot.state.name),
            KeyValue(key="mission_id", value=snapshot.mission_id),
            KeyValue(key="route_id", value=snapshot.route_id),
            KeyValue(key="reason_code", value=snapshot.reason_code),
            KeyValue(key="active_goal_uuid", value=snapshot.active_goal_uuid),
        ]
        result = self._latest_passive_result
        diagnostic.values.append(KeyValue(key="progress_supervisor_mode", value="ACTIVE_POLICY"))
        diagnostic.values.extend(
            KeyValue(key=key, value=value)
            for key, value in self._progress_policy_diagnostics(self.steady_now()).items()
        )
        if result is not None:
            diagnostic.values.extend([
                KeyValue(key="progress_supervisor_event", value=result.primary_event.name),
                KeyValue(key="progress_classification", value=result.progress.name),
                KeyValue(key="collision_classification", value=result.collision.name),
                KeyValue(key="recovery_delta", value=str(result.recovery_delta)),
            ])
            diagnostic.values.extend(KeyValue(key=f"{name}_age_sec", value="none" if age is None else f"{age:.6f}")
                                     for name, age in result.ages_sec.items())
        self._status_pub.publish(diagnostic)
        if snapshot.state in (MissionStateCode.TEMPORARILY_BLOCKED, MissionStateCode.BLOCKED):
            diagnostic.message = snapshot.block_reason
            diagnostic.values.append(KeyValue(key="block_reason", value=snapshot.block_reason))
            self._block_reason_pub.publish(diagnostic)
            self._published_block_reason = snapshot.block_reason
        elif self._published_block_reason:
            cleared = DiagnosticStatus()
            cleared.name = "mission_manager"
            cleared.level = DiagnosticStatus.OK
            cleared.message = ""
            cleared.values = [KeyValue(key="block_reason", value="")]
            self._block_reason_pub.publish(cleared)
            self._published_block_reason = ""
        if snapshot.state in (
            MissionStateCode.SUCCEEDED, MissionStateCode.CANCELLED,
            MissionStateCode.BLOCKED, MissionStateCode.FAILED,
        ):
            self._reset_progress_policy_activation()

    def _mission_cb(self, msg: RouteMission) -> None:
        with self._lock:
            self._core.receive_mission(msg)

    def _start_cb(self, request, response):
        del request
        with self._lock:
            result = self._core.start()
            response.success = bool(result.valid and self._core.state != MissionStateCode.FAILED)
            response.message = result.reason_code if result.reason_code else ("mission started" if response.success else "start refused")
        return response

    def _cancel_cb(self, request, response):
        del request
        with self._lock:
            accepted = self._core.request_cancel()
            response.success = bool(accepted)
            response.message = "cancel request accepted" if accepted else "cancel not accepted in current state"
        return response

    def _pause_cb(self, request, response):
        with self._lock:
            if request.data:
                accepted = self._core.request_pause()
                response.message = "pause request accepted" if accepted else "pause not accepted in current state"
            else:
                accepted = self._core.resume()
                response.message = "resume accepted" if accepted else "resume requires PAUSED"
            response.success = bool(accepted)
        return response

    def _watchdog_cb(self) -> None:
        with self._lock:
            now = self.steady_now()
            self._observe_transform(now)
            pair_state = self._command_pairer.adjudicate(now).state
            if (pair_state is CausalPairState.STALE
                    or self._explicit_failure_event(now) is not None):
                self._evaluate_progress_passively(now)
            self._publish_passive_status()
            if self._active_progress_goal_uuid:
                self._apply_pending_progress_result(now)
            self._core.tick(now)
            if hasattr(self._transport, "prune_futures"):
                self._transport.prune_futures()

    def destroy_node(self) -> bool:
        if hasattr(self, "_watchdog_timer"):
            self.destroy_timer(self._watchdog_timer)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
