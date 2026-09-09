import pytest

from mkmini_cmd_adapter import Gear
from mkmini_cmd_adapter.zero_speed_adjudication import (
    Mk2E2aAuthorityError,
    Mk2E2aZeroSpeedAuthority,
    ZeroSpeedAdjudicationViolation,
    ZeroSpeedGearSession,
    candidate_gears,
)


class FakeTransport:
    def __init__(self):
        self.opened = False
        self.closed = False
        self.frames = []

    def open(self):
        self.opened = True

    def send_frame(self, frame, timestamp=None):
        assert self.opened
        self.frames.append((frame, timestamp))

    def close(self):
        self.closed = True


def test_only_four_non_reverse_candidate_gears_are_available():
    assert candidate_gears() == frozenset((Gear.DISABLE, Gear.N, Gear.D, Gear.P))
    with pytest.raises(ZeroSpeedAdjudicationViolation, match="NOT_APPROVED"):
        ZeroSpeedGearSession(FakeTransport(), gear=Gear.R)


def test_authority_is_required_before_opening_transport():
    transport = FakeTransport()
    session = ZeroSpeedGearSession(transport, gear=Gear.N)
    with pytest.raises(Mk2E2aAuthorityError):
        session.open()
    assert transport.opened is False


@pytest.mark.parametrize("gear", (Gear.DISABLE, Gear.N, Gear.D, Gear.P))
def test_every_candidate_emits_only_zero_speed_centered_codec_frames(gear):
    transport = FakeTransport()
    session = ZeroSpeedGearSession(
        transport,
        gear=gear,
        authority=Mk2E2aZeroSpeedAuthority(acknowledged=True),
        alive_initial=15,
    )
    session.open()
    first = session.emit(1.0)
    second = session.emit(1.01)
    session.close()
    assert (first.alive_counter, second.alive_counter) == (15, 0)
    assert transport.closed is True
    for frame, _ in transport.frames:
        word = int.from_bytes(frame.data[:7], "little")
        assert (word & 0x0F) == gear
        assert ((word >> 4) & 0xFFFF) == 0
        assert ((word >> 20) & 0xFFFF) == 0
        assert ((word >> 36) & 0xFFFF) == 0
        assert frame.dlc == 8 and frame.is_extended
