"""Transport-neutral MK-mini command and frame models."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class Gear(IntEnum):
    """Manufacturer ctrl_cmd gear encoding."""

    DISABLE = 0
    P = 1
    R = 2
    N = 3
    D = 4


@dataclass(frozen=True)
class CtrlCommand:
    """Physical wire-level command, before any v,w kinematics."""

    gear: Gear
    speed_magnitude_mps: float
    inner_wheel_steering_deg: float
    alive_counter: int


@dataclass(frozen=True)
class CanFrame:
    """Neutral frame representation for codecs and test transports."""

    can_id: int
    data: bytes
    is_extended: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.data, bytes):
            object.__setattr__(self, "data", bytes(self.data))
        if not 0 <= self.can_id <= 0x1FFFFFFF:
            raise ValueError("CAN ID is outside the extended-frame range")

    @property
    def dlc(self) -> int:
        return len(self.data)
