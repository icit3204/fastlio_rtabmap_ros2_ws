"""Qualification-only NavigateToPose feedback observation relay.

This node owns no action server/client entities.  It can only copy canonical
feedback to the Mission Manager shadow topic or stop making those copies.
"""
import json
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter as ParameterMsg, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.node import Node
from std_msgs.msg import String


CANONICAL_FEEDBACK = "/navigate_to_pose/_action/feedback"
SHADOW_FEEDBACK = "/phase4_qualification/p4e6b/navigate_to_pose_feedback"
STATE_TOPIC = "/phase4_qualification/p4e6b/feedback_relay_state"


class FeedbackRelay(Node):
    def __init__(self):
        # Humble's generic parameter service discards a successful callback's
        # reason before serializing SetParameters.Response.  The frozen health
        # runner uses that reason as the qualification acknowledgement, so this
        # qualification-only relay owns an explicit service response.
        super().__init__("phase4_p4e6b_feedback_relay", start_parameter_services=False)
        self.declare_parameter("suppress_feedback", False)
        self.suppress = bool(self.get_parameter("suppress_feedback").value)
        self.forwarded = 0
        self.injection_count = 0
        self.injection_monotonic_ns = 0
        self.publisher = self.create_publisher(
            NavigateToPose.Impl.FeedbackMessage, SHADOW_FEEDBACK, 10)
        self.subscription = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage, CANONICAL_FEEDBACK,
            self._feedback, 10)
        self.state_publisher = self.create_publisher(String, STATE_TOPIC, 1)
        self.parameter_service = self.create_service(SetParameters, "~/set_parameters", self._set_parameters_service)
        self._timer = self.create_timer(0.2, self._publish_state)

    def _set_parameters_service(self, request, response):
        try:
            parameters = [Parameter.from_parameter_msg(p) for p in request.parameters]
            result = self._parameter_callback(parameters)
        except Exception as exc:
            result = SetParametersResult(successful=False, reason=f"parameter request error: {exc}")
        response.results = [result]
        return response

    def _feedback(self, message):
        if self.suppress:
            return
        self.publisher.publish(message)
        self.forwarded += 1

    def _parameter_callback(self, parameters):
        requested = self.suppress
        for parameter in parameters:
            if parameter.name == "suppress_feedback":
                requested = bool(parameter.value)
        if requested and not self.suppress:
            self.injection_count += 1
            self.injection_monotonic_ns = time.monotonic_ns()
        self.suppress = requested
        self._publish_state()
        return SetParametersResult(successful=True, reason=self.state)

    @property
    def state(self):
        if self.suppress and self.injection_count == 1:
            return "INJECTED"
        if self.suppress:
            return "INVALID_MULTIPLE_INJECTIONS"
        return "READY_FORWARD"

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps({
            "state": self.state,
            "healthy": True,
            "suppress_feedback": self.suppress,
            "injection_count": self.injection_count,
            "injection_monotonic_ns": self.injection_monotonic_ns,
            "forwarded_count": self.forwarded,
        }, sort_keys=True)
        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
