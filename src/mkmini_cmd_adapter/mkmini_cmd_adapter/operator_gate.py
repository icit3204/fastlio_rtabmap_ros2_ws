"""Fail-closed safe-neutral operator gate for contained commissioning.

The gate is deliberately transport-neutral.  The caller supplies a
non-blocking decision poll and feedback/eligibility checks; the gate owns the
only command permitted while a human is deciding: codec-generated N+0+0.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Any, Callable

from .micro_motion import MicroMotionPhase, MicroMotionTransport, MicroMotionRunner


class OperatorGateDecision(str, Enum):
    YES = "YES"
    NO = "NO"
    ABORT = "ABORT"


class OperatorGateOutcome(str, Enum):
    APPROVED = "APPROVED"
    REJECTED = "REJECTED"
    ABORTED = "ABORTED"
    TIMEOUT = "TIMEOUT"
    FAULT = "FAULT"


@dataclass(frozen=True)
class OperatorGateResult:
    outcome: OperatorGateOutcome
    next_alive_counter: int
    emitted_frames: int
    elapsed_sec: float
    reason: str | None = None


class SafeNeutralOperatorGate:
    """Maintain N+0+0 at 100 Hz until a safe, explicit decision arrives.

    ``decision_poll`` must be non-blocking and return ``None`` or one of the
    three allowed decisions.  ``health_check`` and ``eligibility_check`` are
    called after every heartbeat, so an operator pause never disables safety
    monitoring.  A YES is accepted only after a second fresh eligibility
    check.
    """

    STATE = "SAFE_NEUTRAL_OPERATOR_GATE"
    OUTPUT_HZ = 100.0
    PERIOD_SEC = 0.010
    DEFAULT_MAX_WAIT_SEC = 30.0

    def __init__(
        self,
        transport: MicroMotionTransport,
        health_check: Callable[[], str | None],
        eligibility_check: Callable[[], bool | Any],
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        max_wait_sec: float = DEFAULT_MAX_WAIT_SEC,
    ) -> None:
        if not math.isfinite(max_wait_sec) or max_wait_sec <= 0.0:
            raise ValueError("max_wait_sec must be finite and positive")
        self.transport = transport
        self.health_check = health_check
        self.eligibility_check = eligibility_check
        self.clock = clock
        self.sleep = sleep
        self.max_wait_sec = max_wait_sec

    @staticmethod
    def _decision(value: OperatorGateDecision | str | None) -> OperatorGateDecision | None:
        if value is None:
            return None
        if isinstance(value, OperatorGateDecision):
            return value
        try:
            return OperatorGateDecision(str(value).strip().upper())
        except ValueError as exc:
            raise ValueError("operator decision must be YES, NO, or ABORT") from exc

    def _eligibility_reason(self) -> str | None:
        result = self.eligibility_check()
        if isinstance(result, bool):
            return None if result else "FRESH_ELIGIBILITY_FAILED"
        if getattr(result, "eligible", None) is True:
            return None
        details = tuple(getattr(result, "hard_abort_reasons", ())) + tuple(
            getattr(result, "unexpected_warnings", ())
        )
        if details:
            return "FRESH_ELIGIBILITY_FAILED:" + ",".join(details)
        return "FRESH_ELIGIBILITY_FAILED"

    def wait(self, decision_poll: Callable[[], OperatorGateDecision | str | None], *, alive_initial: int = 0) -> OperatorGateResult:
        if isinstance(alive_initial, bool) or not 0 <= alive_initial <= 15:
            raise ValueError("alive_initial must be in [0, 15]")
        start = self.clock()
        deadline = start + self.max_wait_sec
        next_emit = start
        alive = alive_initial
        emitted = 0

        while True:
            now = self.clock()
            if now >= deadline:
                return OperatorGateResult(
                    OperatorGateOutcome.TIMEOUT, alive, emitted, now - start, "OPERATOR_GATE_TIMEOUT"
                )

            if now + 1e-12 >= next_emit:
                # The phase definition routes through MkminiCanCodec and
                # cannot produce D/R/steering frames.
                emission = MicroMotionRunner.synthesize_phase(
                    MicroMotionPhase.FORWARD_NEUTRAL,
                    self.PERIOD_SEC,
                    alive_initial=alive,
                    timestamp_initial=now,
                )[0]
                self.transport.send_frame(emission.frame, timestamp=emission.timestamp_sec)
                alive = (alive + 1) & 0x0F
                emitted += 1
                next_emit += self.PERIOD_SEC

                reason = self.health_check()
                if reason:
                    return OperatorGateResult(OperatorGateOutcome.FAULT, alive, emitted, now - start, reason)
                eligibility_reason = self._eligibility_reason()
                if eligibility_reason:
                    return OperatorGateResult(
                        OperatorGateOutcome.FAULT, alive, emitted, now - start, eligibility_reason
                    )

                decision = self._decision(decision_poll())
                if decision is OperatorGateDecision.YES:
                    reason = self.health_check()
                    if reason:
                        return OperatorGateResult(OperatorGateOutcome.FAULT, alive, emitted, now - start, reason)
                    eligibility_reason = self._eligibility_reason()
                    if eligibility_reason:
                        return OperatorGateResult(
                            OperatorGateOutcome.FAULT, alive, emitted, now - start, eligibility_reason
                        )
                    return OperatorGateResult(OperatorGateOutcome.APPROVED, alive, emitted, now - start)
                if decision is OperatorGateDecision.NO:
                    return OperatorGateResult(OperatorGateOutcome.REJECTED, alive, emitted, now - start, "OPERATOR_NO")
                if decision is OperatorGateDecision.ABORT:
                    return OperatorGateResult(OperatorGateOutcome.ABORTED, alive, emitted, now - start, "OPERATOR_ABORT")

            remaining = min(max(0.0, next_emit - self.clock()), max(0.0, deadline - self.clock()))
            self.sleep(min(self.PERIOD_SEC, remaining) if remaining else self.PERIOD_SEC)


__all__ = [
    "OperatorGateDecision",
    "OperatorGateOutcome",
    "OperatorGateResult",
    "SafeNeutralOperatorGate",
]
