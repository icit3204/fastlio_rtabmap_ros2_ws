"""Bounded software-only Gate remap trial for the D2A shadow authority."""

import time

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from mkmini_cmd_adapter import HealthAssessment, HealthReason, HealthStatus
from mkmini_cmd_adapter.ros_node import MkminiCmdAdapterNode
from mkmini_cmd_adapter.validity_ros_node import (
    MkminiControllerValidityNode,
    ValidityFeedbackSnapshot,
)
from vehicle_cmd_safety.guarded_vehicle_cmd_gate import GuardedVehicleCmdGate


def qos(depth=1):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class MutableValidityProvider:
    def __init__(self):
        self.snapshot_value = ValidityFeedbackSnapshot(None, None, None, None)

    def snapshot(self, now):
        del now
        return self.snapshot_value


class MutableAdapterFeedback:
    def snapshot(self, now):
        del now
        from mkmini_cmd_adapter import CtrlFeedback, RunningMode, WheelFeedback

        return type("Snapshot", (), {
            "health": HealthAssessment(HealthStatus.VALID, True),
            "ctrl_feedback": CtrlFeedback(4, 0.0, 0.0, RunningMode.AUTO, 0, True, 0, 0),
            "left_wheel_feedback": WheelFeedback(0.0, 0, 0, True),
            "right_wheel_feedback": WheelFeedback(0.0, 0, 0, True),
        })()


class Harness(Node):
    def __init__(self):
        super().__init__("mkmini_d2a_gate_harness")
        self.safe_pub = self.create_publisher(Twist, "/cmd_vel_nav_safe", qos())
        self.localization_pub = self.create_publisher(Bool, "/system/localization_valid", qos())
        self.collision_pub = self.create_publisher(Bool, "/system/collision_monitor_valid", qos())
        self.gate_states = []
        self.safe_outputs = []
        self.create_subscription(
            DiagnosticStatus, "/vehicle_cmd_safety/state", self.gate_states.append, qos(10)
        )
        self.create_subscription(
            TwistStamped, "/vehicle_cmd_safe", self.safe_outputs.append, qos(10)
        )

    def publish_authorities(self, velocity=0.10):
        command = Twist()
        command.linear.x = velocity
        self.safe_pub.publish(command)
        permission = Bool()
        permission.data = True
        self.localization_pub.publish(permission)
        self.collision_pub.publish(permission)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    # This is the task-local runtime remap required by D2A. It affects only
    # this test process; the Gate source/configuration remains untouched.
    rclpy.init(args=["--ros-args", "-r", "/system/controller_valid:=/phase5a/mkmini/controller_valid",
                     "-p", "max_forward_velocity:=0.20", "-p", "max_angular_velocity:=0.80",
                     "-p", "max_linear_increase_rate:=1.0", "-p", "max_angular_increase_rate:=1.0",
                     "-p", "authority_stability_sec:=0.10", "-p", "controller_timeout_sec:=0.20",
                     "-p", "safe_twist_timeout_sec:=0.30"])
    yield
    if rclpy.ok():
        rclpy.shutdown()


def spin(executor, duration, publish=None):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        if publish:
            publish()
        executor.spin_once(timeout_sec=0.005)


def test_shadow_validity_remap_drives_gate_and_fails_closed():
    provider = MutableValidityProvider()
    validity = MkminiControllerValidityNode(feedback_provider=provider)
    validity.set_parameters([
        rclpy.parameter.Parameter(
            "feedback_source_timeout_sec", rclpy.Parameter.Type.DOUBLE, 0.20
        ),
        rclpy.parameter.Parameter(
            "recovery_stability_sec", rclpy.Parameter.Type.DOUBLE, 0.10
        ),
    ])
    adapter = MkminiCmdAdapterNode(feedback_provider=MutableAdapterFeedback())
    adapter.set_parameters([
        rclpy.parameter.Parameter("command_timeout_sec", rclpy.Parameter.Type.DOUBLE, 0.30),
        rclpy.parameter.Parameter("gate_state_timeout_sec", rclpy.Parameter.Type.DOUBLE, 0.30),
        rclpy.parameter.Parameter("standstill_speed_threshold_mps", rclpy.Parameter.Type.DOUBLE, 0.01),
        rclpy.parameter.Parameter("standstill_stability_sec", rclpy.Parameter.Type.DOUBLE, 0.10),
    ])
    gate = GuardedVehicleCmdGate()
    harness = Harness()
    executor = SingleThreadedExecutor()
    for node in (validity, adapter, gate, harness):
        executor.add_node(node)

    def close():
        for node in (validity, adapter, gate, harness):
            executor.remove_node(node)
            node.destroy_node()

    try:
        # No feedback means the shadow is false and the Gate cannot arm.
        # Allow the Gate's lightweight 0.5 s graph audit to discover the
        # task-local authority publishers.
        spin(executor, 0.65, harness.publish_authorities)
        assert validity.last_result is not None and not validity.last_result.valid
        assert gate._core.request_arm(True, time.monotonic())[0] is False

        # Healthy evidence enters stability wait, then becomes the sole
        # controller-valid publisher on the remapped Gate input.
        healthy = HealthAssessment(HealthStatus.VALID, True)
        provider.snapshot_value = ValidityFeedbackSnapshot(healthy, healthy, 0.01, 0.01)
        spin(executor, 0.30, harness.publish_authorities)
        assert validity.last_result is not None and validity.last_result.valid

        client = harness.create_client(SetBool, "/vehicle_cmd_safety/arm")
        assert client.wait_for_service(timeout_sec=1.0)
        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert future.result() is not None and future.result().success
        spin(executor, 0.20, lambda: harness.publish_authorities(0.20))
        assert gate._core.state == "ARMED"
        assert adapter.transport.count > 0

        # Gate receives the remapped shadow false, emits its existing
        # canonical fault reason, and the adapter cannot emit nonzero motion.
        invalid = HealthAssessment(
            HealthStatus.INVALID, False, reasons=(HealthReason.MODE_REMOTE,)
        )
        provider.snapshot_value = ValidityFeedbackSnapshot(invalid, healthy, 0.01, 0.01)
        deadline = time.monotonic() + 0.35
        while gate._core.state != "FAULT" and time.monotonic() < deadline:
            harness.publish_authorities(0.20)
            executor.spin_once(timeout_sec=0.005)
        assert validity.last_result is not None and not validity.last_result.valid
        assert gate._core.state == "FAULT"
        assert gate._core.fault_reason == "CONTROLLER_PERMISSION_INVALID"
        spin(executor, 0.10, lambda: harness.publish_authorities(0.20))
        assert any(status.message == "CONTROLLER_PERMISSION_INVALID" for status in harness.gate_states)
        deadline = time.monotonic() + 0.20
        while (
            (adapter.core is None or adapter.core.state.value != "FAULTED")
            and time.monotonic() < deadline
        ):
            harness.publish_authorities(0.20)
            executor.spin_once(timeout_sec=0.005)
        fault_record_start = adapter.transport.count
        spin(executor, 0.10, lambda: harness.publish_authorities(0.20))
        after_fault = adapter.transport.records[fault_record_start:]
        assert all(((int.from_bytes(record.frame.data[:7], "little") >> 4) & 0xFFFF) == 0
                   for record in after_fault)

        # The topic used by the Gate subscription is remapped in this process;
        # no publisher was created on the canonical topic by this trial.
        shadow_publishers = gate.get_publishers_info_by_topic("/phase5a/mkmini/controller_valid")
        assert any(info.node_name == "mkmini_controller_validity_node" for info in shadow_publishers)
        assert not any(
            info.node_name == "mkmini_d2a_gate_harness"
            for info in gate.get_publishers_info_by_topic("/system/controller_valid")
        )
    finally:
        close()
