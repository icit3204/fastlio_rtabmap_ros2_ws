import math

import pytest

from wheelchair_cmd_adapter.labmate_bridge_core import (
    BridgeReason,
    SafeTwist,
    convert_safe_twist,
)


def command(v: float, w: float, **kwargs) -> SafeTwist:
    values = dict(frame_id="base_footprint", stamp_sec=9.9, linear_x=v, angular_z=w)
    values.update(kwargs)
    return SafeTwist(**values)


@pytest.mark.parametrize(
    "v,w,expected",
    [
        (0.25, 0.0, (10000.0, 250.0, 0.0)),
        (0.20, 0.10, (-2000.0, 200.0, 0.0)),
        (0.20, -0.10, (2000.0, 200.0, 0.0)),
        (-0.25, 0.0, (10000.0, -250.0, 0.0)),
        (-0.20, 0.10, (2000.0, -200.0, 0.0)),
        (-0.20, -0.10, (-2000.0, -200.0, 0.0)),
    ],
)
def test_direction_matrix(v, w, expected):
    result = convert_safe_twist(command(v, w), now_ros_sec=10.0, receipt_age_sec=0.01)
    assert result.valid
    assert result.output == expected


@pytest.mark.parametrize(
    "candidate,now,age,reason",
    [
        (None, 10.0, None, BridgeReason.STARTUP),
        (command(0.2, 0.0), 10.0, 0.251, BridgeReason.STALE),
        (command(0.2, 0.0, stamp_sec=0.0), 10.0, 0.01, BridgeReason.INVALID_TIMESTAMP),
        (command(0.2, 0.0, frame_id="base_link"), 10.0, 0.01, BridgeReason.WRONG_FRAME),
        (command(math.nan, 0.0), 10.0, 0.01, BridgeReason.NONFINITE),
        (command(0.0, 0.1), 10.0, 0.01, BridgeReason.IN_PLACE_ROTATION),
        (command(0.2, 0.3), 10.0, 0.01, BridgeReason.RADIUS_LIMIT),
        (command(0.26, 0.0), 10.0, 0.01, BridgeReason.VELOCITY_LIMIT),
    ],
)
def test_fail_closed(candidate, now, age, reason):
    result = convert_safe_twist(candidate, now_ros_sec=now, receipt_age_sec=age)
    assert not result.valid
    assert result.reason == reason
    assert result.output == (0.0, 0.0, 0.0)


def test_unsupported_axis_and_zero_are_fail_closed():
    unsupported = command(0.2, 0.0, linear_y=0.1)
    assert convert_safe_twist(
        unsupported, now_ros_sec=10.0, receipt_age_sec=0.01
    ).reason == BridgeReason.UNSUPPORTED_AXIS
    zero = convert_safe_twist(command(0.0, 0.0), now_ros_sec=10.0, receipt_age_sec=0.01)
    assert not zero.valid
    assert zero.output == (0.0, 0.0, 0.0)
