import math

import pytest

from mkmini_cmd_adapter import FeedbackAliveObserver


def observe(observer, value, stamp, checksum=True):
    return observer.observe(value, stamp, checksum_valid=checksum)


def test_normal_increments_are_valid():
    observer = FeedbackAliveObserver()
    assert observe(observer, 4, 1.0).valid
    result = observe(observer, 5, 1.01)
    assert result.valid and not result.warning and result.reason == "NORMAL_INCREMENT"


def test_counter_wrap_15_to_zero_is_normal():
    observer = FeedbackAliveObserver()
    observe(observer, 15, 1.0)
    result = observe(observer, 0, 1.01)
    assert result.valid and result.delta_mod16 == 1


@pytest.mark.parametrize("previous,current,delta", [(3, 5, 2), (14, 1, 3), (2, 7, 5)])
def test_one_or_multiple_missed_observations_are_valid_warnings(previous, current, delta):
    observer = FeedbackAliveObserver()
    observe(observer, previous, 1.0)
    result = observe(observer, current, 1.02)
    assert result.valid and result.warning
    assert result.reason == "FORWARD_OBSERVATION_GAP"
    assert result.delta_mod16 == delta


def test_duplicate_or_replay_is_invalid_and_does_not_advance_baseline():
    observer = FeedbackAliveObserver()
    observe(observer, 7, 1.0)
    repeated = observe(observer, 7, 1.01)
    assert not repeated.valid and repeated.reason == "REPEATED_REPLAY"
    assert observe(observer, 8, 1.02).valid


@pytest.mark.parametrize("previous,current", [(5, 4), (0, 15), (2, 10)])
def test_backward_or_implausibly_large_delta_is_invalid(previous, current):
    observer = FeedbackAliveObserver()
    observe(observer, previous, 1.0)
    result = observe(observer, current, 1.01)
    assert not result.valid
    assert result.reason == "BACKWARD_OR_IMPLAUSIBLE_DELTA"


def test_stale_and_reversed_timestamps_fail_closed():
    observer = FeedbackAliveObserver(maximum_interval_sec=0.05)
    observe(observer, 1, 1.0)
    assert observe(observer, 2, 1.051).reason == "STALE_SEQUENCE"
    assert observe(observer, 2, 0.99).reason == "TIMESTAMP_REVERSED_OR_DUPLICATE"
    assert observe(FeedbackAliveObserver(), 1, math.nan).reason == "TIMESTAMP_INVALID"


def test_checksum_failure_is_invalid_and_does_not_advance_baseline():
    observer = FeedbackAliveObserver()
    observe(observer, 8, 1.0)
    failed = observe(observer, 9, 1.01, checksum=False)
    assert not failed.valid and failed.reason == "CHECKSUM_INVALID"
    assert observe(observer, 9, 1.02).valid
