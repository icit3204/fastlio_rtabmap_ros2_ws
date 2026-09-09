"""Stationary-only ROS entry point for the MK-mini command-chain dry run.

It selects the existing in-memory ``MockTransport`` adapter node and injects
only deterministic stationary feedback.  It imports neither the optional TX
module nor SocketCAN.  This is a qualification seam, not a production
feedback or vehicle-control authority.
"""

from __future__ import annotations

from .codec import CtrlFeedback, RunningMode, WheelFeedback
from .health import HealthAssessment, HealthStatus
from .model import Gear
from .ros_node import FeedbackSnapshot, MkminiCmdAdapterNode
import rclpy


class DryRunStationaryFeedbackProvider:
    """Explicit dry-run feedback only; never reads or writes a vehicle bus."""

    def snapshot(self, now_monotonic_sec: float) -> FeedbackSnapshot:
        del now_monotonic_sec
        ctrl = CtrlFeedback(
            gear=Gear.D,
            speed_magnitude_mps=0.0,
            inner_wheel_steering_deg=0.0,
            running_mode=RunningMode.AUTO,
            alive_counter=0,
            checksum_valid=True,
            reserved_bits_36_43=0,
            reserved_bits_46_51=0,
        )
        wheel = WheelFeedback(speed_mps=0.0, pulse_count=0, alive_counter=0, checksum_valid=True)
        return FeedbackSnapshot(
            health=HealthAssessment(HealthStatus.VALID, True),
            ctrl_feedback=ctrl,
            left_wheel_feedback=wheel,
            right_wheel_feedback=wheel,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MkminiCmdAdapterNode(feedback_provider=DryRunStationaryFeedbackProvider())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
