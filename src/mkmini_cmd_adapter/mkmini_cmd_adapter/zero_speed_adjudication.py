"""Bounded MK2E2A zero-speed gear adjudication primitives.

This is deliberately not a vehicle controller.  It accepts only the four
stationary candidate gears, with exactly zero speed and centered steering,
and requires an explicit task-scoped authority before any injected transport
is opened.  The CLI that uses it performs the receive-only baseline check.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from .codec import AliveCounter, ManufacturerCommandValidator, MkminiCanCodec
from .model import CanFrame, CtrlCommand, Gear


class ZeroSpeedAdjudicationViolation(ValueError):
    """The requested candidate is outside the MK2E2A stationary scope."""


class Mk2E2aAuthorityError(RuntimeError):
    """The explicit, task-scoped physical-TX acknowledgement is absent."""


class FrameTxTransport(Protocol):
    def open(self) -> None:
        ...

    def send_frame(self, frame: CanFrame, timestamp: float | None = None) -> object:
        ...

    def close(self) -> None:
        ...


_CANDIDATE_GEARS = frozenset((Gear.DISABLE, Gear.N, Gear.D, Gear.P))


@dataclass(frozen=True)
class Mk2E2aZeroSpeedAuthority:
    """One-run authority; false by default and never inferred from config."""

    acknowledged: bool = False

    def require(self) -> None:
        if not self.acknowledged:
            raise Mk2E2aAuthorityError("MK2E2A_ZERO_SPEED_AUTHORITY_NOT_ACKNOWLEDGED")


@dataclass(frozen=True)
class ZeroSpeedEmission:
    frame: CanFrame
    timestamp_sec: float
    gear: Gear
    alive_counter: int


class ZeroSpeedGearSession:
    """Finite-rate, zero-speed-only session for one candidate gear.

    Opening and each emission are independently constrained.  In particular,
    this has no API capable of encoding reverse, a nonzero speed, or steering.
    """

    def __init__(
        self,
        transport: FrameTxTransport,
        *,
        gear: Gear,
        authority: Mk2E2aZeroSpeedAuthority | None = None,
        output_hz: float = 100.0,
        alive_initial: int = 0,
    ) -> None:
        if gear not in _CANDIDATE_GEARS:
            raise ZeroSpeedAdjudicationViolation("MK2E2A_GEAR_NOT_APPROVED")
        if not math.isfinite(output_hz) or not 1.0 <= output_hz <= 100.0:
            raise ValueError("output_hz must be finite and in [1, 100]")
        self.transport = transport
        self.gear = gear
        self.authority = authority or Mk2E2aZeroSpeedAuthority()
        self.period_sec = 1.0 / output_hz
        self._alive = AliveCounter(alive_initial)
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        self.authority.require()
        self.transport.open()
        self._open = True

    def emit(self, now_sec: float) -> ZeroSpeedEmission:
        if not self._open:
            raise RuntimeError("MK2E2A session is not open")
        if not math.isfinite(now_sec):
            raise ValueError("timestamp must be finite")
        alive = self._alive.next()
        command = CtrlCommand(self.gear, 0.0, 0.0, alive)
        if ManufacturerCommandValidator.validate(command):
            raise ZeroSpeedAdjudicationViolation("MK2E2A_MANUFACTURER_ENVELOPE_REJECTED")
        frame = MkminiCanCodec.encode(command)
        self.transport.send_frame(frame, timestamp=now_sec)
        return ZeroSpeedEmission(frame, now_sec, self.gear, alive)

    def close(self) -> None:
        self.transport.close()
        self._open = False


def candidate_gears() -> frozenset[Gear]:
    """The exhaustive, intentionally non-reverse MK2E2A candidate set."""

    return _CANDIDATE_GEARS
