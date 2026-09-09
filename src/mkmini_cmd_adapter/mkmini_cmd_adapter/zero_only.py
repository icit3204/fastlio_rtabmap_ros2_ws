"""Restricted first-TX commissioning session.

The session is deliberately narrower than keyboard control: it accepts only a
centered neutral command and delegates encoding to ``MkminiCanCodec``.  A
qualified authority object is still required before the transport is opened.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from .codec import ManufacturerCommandValidator, MkminiCanCodec
from .commissioning import RealTransportAuthority
from .model import CanFrame, CtrlCommand, Gear
from .adapter import PhysicalCommandModel


class TxFrameTransport(Protocol):
    def open(self) -> None:
        ...

    def send_frame(self, frame: CanFrame, timestamp: float | None = None) -> object:
        ...

    def close(self) -> None:
        ...


class ZeroOnlyFirstTxViolation(ValueError):
    """A semantic request is outside the restricted first-TX contract."""


@dataclass(frozen=True)
class ZeroOnlyEmission:
    frame: CanFrame
    timestamp_sec: float
    alive_counter: int


class ZeroOnlyFirstTxSession:
    """100-Hz N+0 session; all authority checks precede transport open."""

    def __init__(
        self,
        transport: TxFrameTransport,
        *,
        authority: RealTransportAuthority | None = None,
        output_hz: float = 100.0,
        alive_initial: int = 0,
    ) -> None:
        if not math.isfinite(output_hz) or output_hz <= 0.0:
            raise ValueError("output_hz must be finite and positive")
        if isinstance(alive_initial, bool) or not 0 <= alive_initial <= 15:
            raise ValueError("alive_initial must be in [0, 15]")
        self.transport = transport
        self.authority = authority or RealTransportAuthority.current_unqualified()
        self.period_sec = 1.0 / output_hz
        self._alive = alive_initial
        self._next_due_sec: float | None = None
        self._open = False
        self._requested = False

    @property
    def alive_next(self) -> int:
        return self._alive

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        # This check intentionally precedes the injected transport constructor
        # in the normal factory and precedes physical transport open here.
        self.authority.require_tx()
        self.transport.open()
        self._open = True

    @staticmethod
    def _validate_request(command: PhysicalCommandModel) -> None:
        if not isinstance(command, PhysicalCommandModel):
            raise ZeroOnlyFirstTxViolation("ZERO_ONLY_REQUIRES_PHYSICAL_COMMAND_MODEL")
        if command.gear is not Gear.N:
            raise ZeroOnlyFirstTxViolation("ZERO_ONLY_REQUIRES_N_GEAR")
        if command.speed_magnitude_mps != 0.0:
            raise ZeroOnlyFirstTxViolation("ZERO_ONLY_REQUIRES_ZERO_SPEED")
        if command.inner_wheel_steering_deg != 0.0:
            raise ZeroOnlyFirstTxViolation("ZERO_ONLY_REQUIRES_ZERO_STEERING")

    def request(self, command: PhysicalCommandModel) -> None:
        self._validate_request(command)
        self._requested = True

    def request_neutral(self) -> None:
        self.request(PhysicalCommandModel(Gear.N, 0.0, 0.0))

    def emit(self, now_sec: float) -> ZeroOnlyEmission | None:
        if not math.isfinite(now_sec):
            raise ValueError("frame timestamp must be finite")
        if not self._open:
            raise RuntimeError("zero-only session is not open")
        if not self._requested:
            return None
        command = CtrlCommand(Gear.N, 0.0, 0.0, self._alive)
        if ManufacturerCommandValidator.validate(command):
            raise ZeroOnlyFirstTxViolation("ZERO_ONLY_MANUFACTURER_VALIDATION_FAILED")
        frame = MkminiCanCodec.encode(command)
        alive = self._alive
        self.transport.send_frame(frame, timestamp=now_sec)
        self._alive = (self._alive + 1) & 0x0F
        return ZeroOnlyEmission(frame, now_sec, alive)

    def emit_due(self, now_sec: float) -> ZeroOnlyEmission | None:
        if self._next_due_sec is None:
            self._next_due_sec = now_sec
        if now_sec + 1.0e-12 < self._next_due_sec:
            return None
        emission = self.emit(now_sec)
        self._next_due_sec = now_sec + self.period_sec
        return emission

    def close(self) -> None:
        self.transport.close()
        self._open = False


__all__ = ["TxFrameTransport", "ZeroOnlyEmission", "ZeroOnlyFirstTxSession", "ZeroOnlyFirstTxViolation"]
