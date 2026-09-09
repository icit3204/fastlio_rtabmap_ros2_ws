import pathlib
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from mkmini_cmd_adapter import HealthAssessment, HealthReason, HealthStatus
from mkmini_cmd_adapter.validity_ros_node import (
    MkminiControllerValidityNode,
    SHADOW_DIAGNOSTIC_TOPIC,
    SHADOW_VALID_TOPIC,
    ValidityFeedbackSnapshot,
)


def qos(depth=1):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class MutableProvider:
    def __init__(self):
        self.snapshot_value = ValidityFeedbackSnapshot(None, None, None, None)

    def snapshot(self, now):
        del now
        return self.snapshot_value


class Observer(Node):
    def __init__(self):
        super().__init__("mkmini_validity_ros_test_observer")
        self.bools = []
        self.diagnostics = []
        self.create_subscription(Bool, SHADOW_VALID_TOPIC, self.bools.append, qos())
        self.create_subscription(
            DiagnosticStatus, SHADOW_DIAGNOSTIC_TOPIC, self.diagnostics.append, qos(10)
        )


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init(args=None)
    yield
    if rclpy.ok():
        rclpy.shutdown()


def run(executor, duration=0.15):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


def make(provider):
    node = MkminiControllerValidityNode(feedback_provider=provider)
    node.set_parameters([
        rclpy.parameter.Parameter(
            "feedback_source_timeout_sec", rclpy.Parameter.Type.DOUBLE, 0.5
        ),
        rclpy.parameter.Parameter(
            "recovery_stability_sec", rclpy.Parameter.Type.DOUBLE, 0.1
        ),
    ])
    observer = Observer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)
    return node, observer, executor


def close(node, observer, executor):
    executor.remove_node(node)
    executor.remove_node(observer)
    node.destroy_node()
    observer.destroy_node()


def healthy_snapshot():
    healthy = HealthAssessment(HealthStatus.VALID, True)
    return ValidityFeedbackSnapshot(healthy, healthy, 0.01, 0.02)


def test_ros_node_defaults_false_without_feedback_and_publishes_shadow_only():
    provider = MutableProvider()
    node, observer, executor = make(provider)
    try:
        run(executor)
        assert observer.bools
        assert all(not message.data for message in observer.bools)
        assert node.last_result is not None
        assert node.last_result.reason == "NO_CTRL_FEEDBACK"
        source = pathlib.Path(__file__).parents[1] / "mkmini_cmd_adapter" / "validity_ros_node.py"
        assert "/system/controller_valid" not in source.read_text()
    finally:
        close(node, observer, executor)


def test_healthy_provider_waits_then_publishes_true():
    provider = MutableProvider()
    provider.snapshot_value = healthy_snapshot()
    node, observer, executor = make(provider)
    try:
        run(executor, 0.07)
        assert node.last_result is not None
        assert not node.last_result.valid
        assert node.last_result.state.value == "STABILITY_WAIT"
        run(executor, 0.10)
        assert node.last_result.valid
        assert any(message.data for message in observer.bools)
        assert observer.diagnostics
        values = {item.key: item.value for item in observer.diagnostics[-1].values}
        assert values["publisher_authority_mode"] == "SHADOW_PHASE5A"
    finally:
        close(node, observer, executor)


@pytest.mark.parametrize(
    "reason",
    [HealthReason.MODE_REMOTE, HealthReason.MODE_STOP, HealthReason.ESTOP_ASSERTED,
     HealthReason.AUTO_CAN_FAULT, HealthReason.EPS_FAULT, HealthReason.LEFT_DRIVE_FAULT,
     HealthReason.RIGHT_DRIVE_FAULT, HealthReason.BMS_CAN_FAULT,
     HealthReason.REMOTE_RECEIVER_FAULT, HealthReason.FEEDBACK_CHECKSUM_INVALID],
)
def test_invalid_provider_evidence_publishes_false(reason):
    provider = MutableProvider()
    invalid = HealthAssessment(HealthStatus.INVALID, False, reasons=(reason,))
    healthy = HealthAssessment(HealthStatus.VALID, True)
    provider.snapshot_value = ValidityFeedbackSnapshot(invalid, healthy, 0.01, 0.02)
    node, observer, executor = make(provider)
    try:
        run(executor, 0.15)
        assert node.last_result is not None
        assert not node.last_result.valid
        assert reason.value in node.last_result.contributing_reasons
        assert observer.bools and all(not message.data for message in observer.bools)
    finally:
        close(node, observer, executor)
