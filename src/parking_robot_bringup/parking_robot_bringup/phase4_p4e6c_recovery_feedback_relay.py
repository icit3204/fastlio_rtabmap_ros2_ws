"""Qualification-only NavigateToPose recovery-count shadow relay.

The relay has one canonical feedback subscription and one Mission-Manager-only
shadow publisher.  It never creates an action client/server and never writes
to the canonical feedback topic.
"""
from __future__ import annotations

import copy
import json
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


CANONICAL_FEEDBACK = "/navigate_to_pose/_action/feedback"
SHADOW_FEEDBACK = "/phase4_qualification/p4e6c/navigate_to_pose_feedback"
STATE_TOPIC = "/phase4_qualification/p4e6c/recovery_feedback_relay_state"
MAX_INJECTED_RECOVERIES = 6


class StepwiseRecoverySequence:
    """Qualification-only shadow-count controller requiring source ACKs."""

    def __init__(self, maximum: int = MAX_INJECTED_RECOVERIES):
        self.maximum = int(maximum)
        self.requested = 0
        self.injected = 0

    def request(self, count: int) -> None:
        count = int(count)
        if count != self.injected + 1 or count > self.maximum:
            raise RuntimeError("RECOVERY_STEP_OUT_OF_ORDER")
        if self.requested:
            raise RuntimeError("RECOVERY_STEP_AWAITING_SOURCE_ACK")
        self.requested = count

    def observe_source_ack(self, count: int) -> None:
        if int(count) != self.requested:
            raise RuntimeError("RECOVERY_SOURCE_ACK_MISMATCH")
        self.injected = int(count)
        self.requested = 0

    @property
    def complete(self) -> bool:
        return self.injected == self.maximum and self.requested == 0


def _uuid(message) -> tuple[int, ...]:
    return tuple(int(value) for value in message.goal_id.uuid)


class RecoveryFeedbackRelay(Node):
    """Forward feedback transparently, or apply one bounded count episode."""

    def __init__(self, feedback_timeout_sec: float = 0.75):
        super().__init__("phase4_p4e6c_recovery_feedback_relay")
        self.feedback_timeout_sec = float(feedback_timeout_sec)
        self.state = "READY_FORWARD"
        self.healthy = True
        self.target_goal_uuid = None
        self._observed_goal_uuid = None
        self.injection_count = 0
        self.injection_monotonic_ns = 0
        self.canonical_feedback_count = 0
        self.shadow_feedback_count = 0
        self.latest_canonical_recovery_count = None
        self.latest_shadow_recovery_count = None
        self.injected_recovery_step = 0
        self._stepper = StepwiseRecoverySequence()
        self.sequence_complete = False
        self._armed = False
        self._last_feedback_monotonic_ns = 0
        self.publisher = self.create_publisher(
            NavigateToPose.Impl.FeedbackMessage, SHADOW_FEEDBACK, 10)
        self.subscription = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage, CANONICAL_FEEDBACK,
            self._feedback, 10)
        self.state_publisher = self.create_publisher(String, STATE_TOPIC, 1)
        self.arm_service = self.create_service(Trigger, "~/arm_recovery_injection", self._arm_service)
        self.advance_service = self.create_service(Trigger, "~/advance_recovery_step", self._advance_service)
        self.ack_service = self.create_service(Trigger, "~/acknowledge_recovery_step", self._ack_service)
        self.timer = self.create_timer(0.2, self._watchdog)

    def _invalid(self, reason: str):
        self.state = "INVALID"
        self.healthy = False
        self._armed = False
        self.get_logger().error("P4E6C recovery relay invalid: %s", reason)
        self._publish_state()

    def _arm_service(self, request, response):
        del request
        try:
            self.arm_recovery_injection()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        else:
            response.success = True
            response.message = "RECOVERY_INJECTION_ARMED"
        return response

    def _advance_service(self, request, response):
        del request
        try:
            step = self.advance_recovery_step()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        else:
            response.success = True
            response.message = f"RECOVERY_STEP_REQUESTED_{step}"
        return response

    def _ack_service(self, request, response):
        del request
        try:
            step = self.acknowledge_source_step()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        else:
            response.success = True
            response.message = f"RECOVERY_STEP_ACKNOWLEDGED_{step}"
        return response

    def arm_recovery_injection(self):
        if not self.healthy or self.state == "INVALID":
            raise RuntimeError("RECOVERY_RELAY_INVALID")
        if self._armed or self.injection_count:
            self._invalid("duplicate injection request")
            raise RuntimeError("RECOVERY_INJECTION_DUPLICATE")
        if self._observed_goal_uuid is None:
            self._invalid("no UUID established before arm")
            raise RuntimeError("RECOVERY_UUID_NOT_ESTABLISHED")
        if self.latest_canonical_recovery_count != 0:
            self._invalid("canonical recovery count nonzero before arm")
            raise RuntimeError("RECOVERY_BASELINE_NOT_ZERO")
        self.target_goal_uuid = self._observed_goal_uuid
        self._armed = True
        self.state = "ARMED"
        self._publish_state()

    def advance_recovery_step(self):
        if not self._armed or not self.healthy:
            raise RuntimeError("RECOVERY_RELAY_NOT_ARMED")
        next_step = self.injected_recovery_step + 1
        self._stepper.request(next_step)
        self.state = "INJECTION_ACTIVE"
        self._publish_state()
        return next_step

    def acknowledge_source_step(self):
        if not self._armed or not self._stepper.requested:
            raise RuntimeError("RECOVERY_STEP_NOT_PENDING")
        step = self._stepper.requested
        self._stepper.observe_source_ack(step)
        self.injected_recovery_step = step
        self.state = "INJECTED_HOLDING" if step == MAX_INJECTED_RECOVERIES else "INJECTION_ACTIVE"
        self.sequence_complete = self._stepper.complete
        self._publish_state()
        return step

    def _feedback(self, message):
        if not self.healthy:
            return
        current_uuid = _uuid(message)
        canonical_count = int(message.feedback.number_of_recoveries)
        self.canonical_feedback_count += 1
        self._last_feedback_monotonic_ns = time.monotonic_ns()
        self.latest_canonical_recovery_count = canonical_count
        if self._observed_goal_uuid is None:
            self._observed_goal_uuid = current_uuid
        if not self._armed:
            if current_uuid != self._observed_goal_uuid:
                self._observed_goal_uuid = current_uuid
            self.publisher.publish(message)
            self.shadow_feedback_count += 1
            self.latest_shadow_recovery_count = canonical_count
            self._publish_state()
            return
        if current_uuid != self.target_goal_uuid:
            self._invalid("goal UUID changed after arm")
            return
        if canonical_count != 0:
            self._invalid("canonical recovery count changed after arm")
            return
        if self._stepper.requested:
            self.injected_recovery_step = self._stepper.requested
            self.injection_count = 1
            if not self.injection_monotonic_ns:
                self.injection_monotonic_ns = time.monotonic_ns()
            self.state = ("INJECTING" if self.injected_recovery_step < MAX_INJECTED_RECOVERIES
                          else "INJECTED_HOLDING")
            self.sequence_complete = self._stepper.complete
        else:
            self.state = ("INJECTED_HOLDING" if self.sequence_complete
                          else ("INJECTION_ACTIVE" if self.injected_recovery_step else "ARMED"))
        shadow = copy.deepcopy(message)
        shadow.feedback.number_of_recoveries = self.injected_recovery_step
        self.publisher.publish(shadow)
        self.shadow_feedback_count += 1
        self.latest_shadow_recovery_count = self.injected_recovery_step
        self._publish_state()

    def _watchdog(self):
        if self._armed and self._last_feedback_monotonic_ns:
            age = (time.monotonic_ns() - self._last_feedback_monotonic_ns) / 1e9
            if age > self.feedback_timeout_sec:
                self._invalid("canonical feedback disappeared")
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps({
            "state": self.state,
            "healthy": self.healthy,
            "target_goal_uuid": list(self.target_goal_uuid or ()),
            "injection_count": self.injection_count,
            "injection_monotonic_ns": self.injection_monotonic_ns,
            "canonical_feedback_count": self.canonical_feedback_count,
            "shadow_feedback_count": self.shadow_feedback_count,
            "latest_canonical_recovery_count": self.latest_canonical_recovery_count,
            "latest_shadow_recovery_count": self.latest_shadow_recovery_count,
            "injected_recovery_step": self.injected_recovery_step,
            "requested_recovery_step": self._stepper.requested,
            "sequence_complete": self.sequence_complete,
        }, sort_keys=True)
        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryFeedbackRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
