"""Dependency-free tests for the contained commissioning operator gate."""

from __future__ import annotations

import unittest
from dataclasses import replace

from mkmini_cmd_adapter import (
    ContainedBenchEligibilityResult,
    OperatorGateDecision,
    OperatorGateOutcome,
    SafeNeutralOperatorGate,
    BenchFeedbackSnapshot,
    ContainedBenchEligibilityPolicy,
    PhysicalContainment,
    RunningMode,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeTransport:
    def __init__(self) -> None:
        self.frames = []

    def send_frame(self, frame, timestamp=None):
        self.frames.append((frame, timestamp))


def decode_command(frame):
    word = int.from_bytes(frame.data[:7], "little")
    speed = (word >> 4) & 0xFFFF
    steer_raw = (word >> 20) & 0xFFFF
    steer = steer_raw - 0x10000 if steer_raw & 0x8000 else steer_raw
    return (word & 0xF, speed, steer, (word >> 36) & 0xFFFF, frame.data[7])


class SafeNeutralOperatorGateTests(unittest.TestCase):
    def make_gate(self, clock, transport, health=None, eligible=None, max_wait=30.0):
        return SafeNeutralOperatorGate(
            transport,
            health or (lambda: None),
            eligible or (lambda: True),
            clock=clock,
            sleep=clock.sleep,
            max_wait_sec=max_wait,
        )

    def test_delayed_yes_keeps_100hz_neutral_heartbeat(self):
        for delay in (0.0, 1.0, 2.0, 5.0):
            clock = FakeClock()
            transport = FakeTransport()
            result = self.make_gate(clock, transport).wait(
                lambda: OperatorGateDecision.YES if clock.now >= delay else None,
                alive_initial=14,
            )
            self.assertEqual(result.outcome, OperatorGateOutcome.APPROVED)
            self.assertGreaterEqual(result.emitted_frames, 1)
            timestamps = [stamp for _, stamp in transport.frames]
            self.assertEqual([round(b - a, 6) for a, b in zip(timestamps, timestamps[1:])], [0.01] * (len(timestamps) - 1))
            for frame, _ in transport.frames:
                gear, speed, steer, reserved, checksum = decode_command(frame)
                self.assertEqual((gear, speed, steer, reserved), (3, 0, 0, 0))
                self.assertEqual(checksum, __import__("functools").reduce(lambda x, y: x ^ y, frame.data[:7], 0))

    def test_yes_requires_fresh_final_eligibility(self):
        clock = FakeClock()
        transport = FakeTransport()
        checks = iter([True, False])
        result = self.make_gate(clock, transport, eligible=lambda: next(checks)).wait(lambda: "YES")
        self.assertEqual(result.outcome, OperatorGateOutcome.FAULT)
        self.assertEqual(result.reason, "FRESH_ELIGIBILITY_FAILED")

    def test_detailed_eligibility_reason_is_preserved(self):
        clock = FakeClock()
        transport = FakeTransport()
        result = self.make_gate(
            clock,
            transport,
            eligible=lambda: ContainedBenchEligibilityResult(
                eligible=False,
                hard_abort_reasons=("MODE_NOT_AUTO", "AUTO_CAN_ERROR"),
                unexpected_warnings=("UNEXPECTED_DIAGNOSTIC",),
            ),
        ).wait(lambda: None)
        self.assertEqual(result.outcome, OperatorGateOutcome.FAULT)
        self.assertEqual(
            result.reason,
            "FRESH_ELIGIBILITY_FAILED:MODE_NOT_AUTO,AUTO_CAN_ERROR,UNEXPECTED_DIAGNOSTIC",
        )

    def test_fake_r6_post_handshake_state_is_not_bypassed(self):
        healthy = BenchFeedbackSnapshot(
            ctrl_current=True,
            diagnostic_current=True,
            ctrl_checksum_valid=True,
            diagnostic_checksum_valid=True,
            ctrl_alive_contiguous=True,
            diagnostic_alive_contiguous=True,
            can_state="ERROR-ACTIVE",
            mode=RunningMode.AUTO,
            auto_can_error=False,
            auto_io_error=True,
            remote_off_warning=True,
            vehicle_fault_level=1,
        )
        good = ContainedBenchEligibilityPolicy.evaluate(PhysicalContainment(True, True, True, True), healthy)
        self.assertTrue(good.eligible)

        r6_post_watchdog = replace(healthy, mode=RunningMode.STOP, auto_can_error=True)
        blocked = ContainedBenchEligibilityPolicy.evaluate(
            PhysicalContainment(True, True, True, True), r6_post_watchdog
        )
        self.assertFalse(blocked.eligible)
        self.assertIn("MODE_NOT_AUTO", blocked.hard_abort_reasons)
        self.assertIn("AUTO_CAN_ERROR", blocked.hard_abort_reasons)

    def test_no_and_abort_never_advance(self):
        for decision, outcome, reason in (
            ("NO", OperatorGateOutcome.REJECTED, "OPERATOR_NO"),
            ("ABORT", OperatorGateOutcome.ABORTED, "OPERATOR_ABORT"),
        ):
            clock = FakeClock()
            result = self.make_gate(clock, FakeTransport()).wait(lambda: decision)
            self.assertEqual(result.outcome, outcome)
            self.assertEqual(result.reason, reason)

    def test_timeout_is_bounded_and_safe(self):
        clock = FakeClock()
        transport = FakeTransport()
        result = self.make_gate(clock, transport, max_wait=0.05).wait(lambda: None)
        self.assertEqual(result.outcome, OperatorGateOutcome.TIMEOUT)
        self.assertEqual(result.reason, "OPERATOR_GATE_TIMEOUT")
        self.assertEqual(result.emitted_frames, 5)
        self.assertLessEqual(clock.now, 0.05)

    def test_fault_during_gate_aborts(self):
        clock = FakeClock()
        calls = [0]

        def health():
            calls[0] += 1
            return "AUTO_CAN_ERROR" if calls[0] == 3 else None

        result = self.make_gate(clock, FakeTransport(), health=health).wait(lambda: None)
        self.assertEqual(result.outcome, OperatorGateOutcome.FAULT)
        self.assertEqual(result.reason, "AUTO_CAN_ERROR")
        self.assertEqual(calls[0], 3)

    def test_neutral_gate_keeps_wheel_motion_as_hard_fault(self):
        clock = FakeClock()
        result = self.make_gate(
            clock,
            FakeTransport(),
            health=lambda: "UNEXPECTED_WHEEL_MOTION_DURING_N_PLUS_0",
        ).wait(lambda: None)
        self.assertEqual(result.outcome, OperatorGateOutcome.FAULT)
        self.assertEqual(result.reason, "UNEXPECTED_WHEEL_MOTION_DURING_N_PLUS_0")

    def test_gate_has_no_motion_surface(self):
        self.assertFalse(hasattr(SafeNeutralOperatorGate, "set_speed"))
        self.assertFalse(hasattr(SafeNeutralOperatorGate, "set_steering"))
        self.assertFalse(hasattr(SafeNeutralOperatorGate, "set_gear"))


if __name__ == "__main__":
    unittest.main()
