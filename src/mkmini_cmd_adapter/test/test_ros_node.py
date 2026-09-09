import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from mkmini_cmd_adapter import (
    AdapterState,
    CtrlFeedback,
    HealthAssessment,
    HealthStatus,
    RunningMode,
    WheelFeedback,
)
from mkmini_cmd_adapter.codec import CTRL_CMD_ID
from mkmini_cmd_adapter.codec import verify_checksum
from mkmini_cmd_adapter.ros_node import MkminiCmdAdapterNode


GATE_NAME = "vehicle_cmd_safety/guarded_vehicle_cmd_gate"
GATE_HARDWARE = "vehicle_cmd_safety"


def qos(depth=1):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class MutableFeedback:
    def __init__(self):
        self.health = HealthAssessment(HealthStatus.VALID, True)
        self.speed = 0.0

    def snapshot(self, now):
        del now
        feedback = CtrlFeedback(
            gear=4,
            speed_magnitude_mps=self.speed,
            inner_wheel_steering_deg=0.0,
            running_mode=RunningMode.AUTO,
            alive_counter=0,
            checksum_valid=True,
            reserved_bits_36_43=0,
            reserved_bits_46_51=0,
        )
        wheel = WheelFeedback(self.speed, 0, 0, True)
        return type("Snapshot", (), {
            "health": self.health,
            "ctrl_feedback": feedback,
            "left_wheel_feedback": wheel,
            "right_wheel_feedback": wheel,
        })()


class PublisherHarness(Node):
    def __init__(self):
        super().__init__("mkmini_d1_test_harness")
        self.command_pub = self.create_publisher(TwistStamped, "/vehicle_cmd_safe", qos())
        self.gate_pub = self.create_publisher(DiagnosticStatus, "/vehicle_cmd_safety/state", qos())
        self.diagnostics = []
        self.create_subscription(DiagnosticStatus, "/mkmini_cmd_adapter/state", self.diagnostics.append, qos(10))

    def command(self, v=0.0, w=0.0, unsupported=None):
        msg = TwistStamped()
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        if unsupported:
            for field, value in unsupported.items():
                setattr(getattr(msg.twist, field.split(".")[0]), field.split(".")[1], value)
        self.command_pub.publish(msg)

    def gate(self, state="ARMED", arm_pending=False, fault_latched=False, safe_zero_quiescent=False):
        msg = DiagnosticStatus()
        msg.name = GATE_NAME
        msg.hardware_id = GATE_HARDWARE
        msg.message = "TEST_GATE"
        msg.values = [
            KeyValue(key="state", value=state),
            KeyValue(key="arm_pending", value=str(arm_pending).lower()),
            KeyValue(key="fault_latched", value=str(fault_latched).lower()),
            KeyValue(key="safe_zero_quiescent", value=str(safe_zero_quiescent).lower()),
        ]
        self.gate_pub.publish(msg)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init(args=None)
    yield
    if rclpy.ok():
        rclpy.shutdown()


def make_system(*, feedback=None, timeout=0.5, gate_timeout=0.5, stability=0.1):
    node = MkminiCmdAdapterNode(feedback_provider=feedback)
    node.set_parameters([
        rclpy.parameter.Parameter("command_timeout_sec", rclpy.Parameter.Type.DOUBLE, timeout),
        rclpy.parameter.Parameter("gate_state_timeout_sec", rclpy.Parameter.Type.DOUBLE, gate_timeout),
        rclpy.parameter.Parameter("standstill_speed_threshold_mps", rclpy.Parameter.Type.DOUBLE, 0.01),
        rclpy.parameter.Parameter("standstill_stability_sec", rclpy.Parameter.Type.DOUBLE, stability),
    ])
    harness = PublisherHarness()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    return node, harness, executor


def spin(executor, duration, publish=None):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        if publish is not None:
            publish()
        executor.spin_once(timeout_sec=0.005)


def close_system(node, harness, executor):
    executor.remove_node(node)
    executor.remove_node(harness)
    node.destroy_node()
    harness.destroy_node()


def speed_raw(frame):
    word = int.from_bytes(frame.data[:7], "little")
    return (word >> 4) & 0xFFFF


def steering_raw(frame):
    word = int.from_bytes(frame.data[:7], "little")
    value = (word >> 20) & 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def test_startup_without_gate_command_or_feedback_emits_no_frame():
    node, harness, executor = make_system()
    try:
        spin(executor, 0.15)
        assert node.transport.count == 0
        assert node.core is not None
    finally:
        close_system(node, harness, executor)


@pytest.mark.parametrize(
    "gate_kwargs",
    [
        {},
        {"state": "DISARMED"},
        {"state": "FAULT"},
        {"state": "ARMED", "arm_pending": True},
        {"state": "ARMED", "fault_latched": True},
        {"state": "ARMED", "safe_zero_quiescent": True},
    ],
)
def test_nonzero_requires_fresh_armed_nonquiescent_gate(gate_kwargs):
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback)
    try:
        def publish():
            harness.command(0.2, 0.0)
            if gate_kwargs:
                harness.gate(**gate_kwargs)
        spin(executor, 0.25, publish)
        assert all(speed_raw(record.frame) == 0 for record in node.transport.records)
        assert node.core is None or node.core.state is not AdapterState.MOVING_FORWARD
    finally:
        close_system(node, harness, executor)


def test_gate_without_command_emits_no_frame_and_happy_path_is_d():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback)
    try:
        spin(executor, 0.15, lambda: harness.gate())
        assert node.transport.count == 0
        spin(executor, 0.25, lambda: (harness.gate(), harness.command(0.2, 0.1)))
        assert node.transport.count > 3
        latest = node.transport.latest().frame
        assert latest.can_id == CTRL_CMD_ID
        assert (latest.data[0] & 0x0F) == 4
        assert speed_raw(latest) == 200
        assert steering_raw(latest) > 0
        assert verify_checksum(latest)
        assert any(status.name == "mkmini_cmd_adapter/state" for status in harness.diagnostics)
    finally:
        close_system(node, harness, executor)


@pytest.mark.parametrize("v,w,gear,sign", [(0.2, -0.1, 4, -1), (-0.2, 0.1, 2, -1), (-0.2, -0.1, 2, 1)])
def test_ros_direction_and_steering_signs(v, w, gear, sign):
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback)
    try:
        spin(executor, 0.25, lambda: (harness.gate(), harness.command(v, w)))
        assert node.transport.count > 0
        frame = node.transport.latest().frame
        assert (frame.data[0] & 0x0F) == gear
        raw = steering_raw(frame)
        assert (1 if raw > 0 else -1 if raw < 0 else 0) == sign
    finally:
        close_system(node, harness, executor)


def test_input_20hz_is_separate_from_approximately_100hz_mock_heartbeat_and_alive_rolls():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback, timeout=0.3, gate_timeout=0.3)
    try:
        start = time.monotonic()
        spin(executor, 0.65, lambda: (harness.gate(), harness.command(0.2, 0.0)) if int((time.monotonic() - start) * 20) != int((time.monotonic() - start - 0.006) * 20) else None)
        frames = node.transport.records
        assert 35 <= len(frames) <= 90
        alive = [((record.frame.data[6] >> 4) & 0x0F) for record in frames]
        assert all(alive[i] == (alive[i - 1] + 1) % 16 for i in range(1, len(alive)))
        assert len(set(alive)) == 16
    finally:
        close_system(node, harness, executor)


def test_zero_after_motion_is_stopping_zero_speed_with_retained_d_and_steering_then_no_stationary_frame():
    feedback = MutableFeedback()
    feedback.speed = 0.2
    node, harness, executor = make_system(feedback=feedback, stability=0.1)
    try:
        spin(executor, 0.2, lambda: (harness.gate(), harness.command(0.2, 0.1)))
        before = node.transport.latest().frame
        before_steering = steering_raw(before)
        feedback.speed = 0.2
        spin(executor, 0.08, lambda: (harness.gate(), harness.command(0.0, 0.0)))
        stopping = node.transport.latest().frame
        assert node.core.state is AdapterState.STOPPING_FORWARD
        assert (stopping.data[0] & 0x0F) == 4
        assert speed_raw(stopping) == 0
        assert steering_raw(stopping) == before_steering
        feedback.speed = 0.0
        spin(executor, 0.15, lambda: (harness.gate(), harness.command(0.0, 0.0)))
        assert node.core.state is AdapterState.STATIONARY_HOLD
        held_count = node.transport.count
        spin(executor, 0.1, lambda: (harness.gate(), harness.command(0.0, 0.0)))
        assert node.transport.count == held_count
    finally:
        close_system(node, harness, executor)


def test_gate_deadman_stops_new_nonzero_frames():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback, timeout=0.5, gate_timeout=0.12)
    try:
        spin(executor, 0.2, lambda: (harness.gate(), harness.command(0.2, 0.0)))
        assert node.transport.count > 0
        baseline = node.transport.count
        spin(executor, 0.16)
        after_deadman = node.transport.count
        spin(executor, 0.1)
        assert node.transport.count == after_deadman
        assert node.core.state is AdapterState.FAULTED
        spin(executor, 0.1, lambda: harness.command(0.2, 0.0))
        assert node.transport.count == after_deadman
    finally:
        close_system(node, harness, executor)


def test_malformed_nonfinite_unsupported_and_spin_commands_are_fail_closed():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback)
    try:
        spin(executor, 0.12, lambda: (harness.gate(), harness.command(0.2, 0.0, {"linear.y": 0.1})))
        assert node.transport.count == 0
        spin(executor, 0.12, lambda: (harness.gate(), harness.command(0.0, 0.2)))
        assert node.transport.count == 0
        spin(executor, 0.12, lambda: (harness.gate(), harness.command(float("nan"), 0.0)))
        assert node.transport.count == 0
        spin(executor, 0.12, lambda: (harness.gate(), harness.command(0.2, 10.0)))
        assert node.transport.count == 0
    finally:
        close_system(node, harness, executor)


def test_missing_required_gate_field_fails_closed():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback)
    try:
        def malformed_gate():
            msg = DiagnosticStatus()
            msg.name = GATE_NAME
            msg.hardware_id = GATE_HARDWARE
            msg.values = [KeyValue(key="state", value="ARMED")]
            harness.gate_pub.publish(msg)
            harness.command(0.2, 0.0)
        spin(executor, 0.2, malformed_gate)
        assert node.transport.count == 0
    finally:
        close_system(node, harness, executor)


def test_recovery_requires_fresh_new_command_and_quiescent_is_restrictive():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback, timeout=0.12, gate_timeout=0.12)
    try:
        spin(executor, 0.18, lambda: (harness.gate(), harness.command(0.2, 0.0)))
        baseline = node.transport.count
        spin(executor, 0.18, lambda: harness.gate(safe_zero_quiescent=True))
        assert all(speed_raw(record.frame) == 0 for record in node.transport.records[baseline:])
        assert node.core.state is AdapterState.FAULTED
        spin(executor, 0.1, lambda: harness.gate())
        assert all(speed_raw(record.frame) == 0 for record in node.transport.records[baseline:])
        spin(executor, 0.18, lambda: (harness.gate(), harness.command(0.2, 0.0)))
        assert node.transport.count > baseline
    finally:
        close_system(node, harness, executor)


def test_feedback_loss_forces_fault_and_recovery_does_not_auto_resume():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback, timeout=0.5, gate_timeout=0.5)
    try:
        spin(executor, 0.3, lambda: (harness.gate(), harness.command(0.2, 0.0)))
        baseline = node.transport.count
        assert baseline > 0
        feedback.health = HealthAssessment(HealthStatus.INVALID, False)
        spin(executor, 0.15, lambda: (harness.gate(), harness.command(0.2, 0.0)))
        assert node.core.state is AdapterState.FAULTED
        assert node.transport.count == baseline
        feedback.health = HealthAssessment(HealthStatus.VALID, True)
        spin(executor, 0.1, lambda: harness.gate())
        assert node.transport.count == baseline
        spin(executor, 0.15, lambda: (harness.gate(), harness.command(0.2, 0.0)))
        assert node.transport.count > baseline
    finally:
        close_system(node, harness, executor)


def test_steering_over_manufacturer_limit_is_rejected_without_frame():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback)
    try:
        spin(executor, 0.2, lambda: (harness.gate(), harness.command(1.0, 1.0)))
        assert node.transport.count == 0
        assert node.core.state is AdapterState.FAULTED
    finally:
        close_system(node, harness, executor)


def test_command_then_gate_and_gate_then_command_both_require_fresh_inputs():
    for first in ("command", "gate"):
        feedback = MutableFeedback()
        node, harness, executor = make_system(feedback=feedback)
        try:
            if first == "command":
                harness.command(0.2, 0.0)
                spin(executor, 0.08)
                assert node.transport.count == 0
                spin(executor, 0.15, lambda: harness.gate())
            else:
                harness.gate()
                spin(executor, 0.08)
                assert node.transport.count == 0
                spin(executor, 0.15, lambda: harness.command(0.2, 0.0))
            assert node.transport.count > 0
        finally:
            close_system(node, harness, executor)


def test_command_deadman_stops_new_nonzero_frames():
    feedback = MutableFeedback()
    node, harness, executor = make_system(feedback=feedback, timeout=0.12, gate_timeout=0.5)
    try:
        spin(executor, 0.18, lambda: (harness.gate(), harness.command(0.2, 0.0)))
        baseline = node.transport.count
        spin(executor, 0.2, lambda: harness.gate())
        after_deadman = node.transport.count
        spin(executor, 0.1, lambda: harness.gate())
        assert node.transport.count == after_deadman
        assert node.core.state is AdapterState.FAULTED
    finally:
        close_system(node, harness, executor)
