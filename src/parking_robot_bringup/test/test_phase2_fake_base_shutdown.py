import threading

import pytest
from builtin_interfaces.msg import Time
from rclpy._rclpy_pybind11 import RCLError

from parking_robot_bringup.phase2_fake_base import (
    Phase2FakeBase,
    _shutdown_context_once,
    _spin_until_shutdown,
)
from parking_robot_bringup.phase2_fake_base_math import Pose2D, Twist2D


class _ClockNow:
    def to_msg(self):
        return Time(sec=1, nanosec=0)


class _Clock:
    def now(self):
        return _ClockNow()


class _Publisher:
    def __init__(self, exc=None):
        self.exc = exc
        self.count = 0

    def publish(self, _msg):
        self.count += 1
        if self.exc is not None:
            raise self.exc


class _TfBroadcaster:
    def __init__(self):
        self.count = 0

    def sendTransform(self, _msg):
        self.count += 1


class _Timer:
    def __init__(self):
        self.cancel_count = 0

    def cancel(self):
        self.cancel_count += 1


def _fake_base(valid_context=True, publish_exc=None):
    fake = object.__new__(Phase2FakeBase)
    fake._shutting_down = False
    fake._timer = _Timer()
    fake._lock = threading.Lock()
    fake._pose = Pose2D(1.0, 2.0, 0.25)
    fake._active_twist = Twist2D(0.0, 0.0)
    fake._last_command_steady = None
    fake._last_integration_steady = 1.0
    fake._cmd_timeout_sec = 0.5
    fake._max_dt_sec = 0.1
    fake._odom_frame = "odom"
    fake._base_frame = "base_footprint"
    fake._odom_pub = _Publisher(publish_exc)
    fake._tf_broadcaster = _TfBroadcaster()
    fake._ros_context_is_valid = lambda: valid_context
    fake.get_clock = lambda: _Clock()
    return fake


def test_stop_publication_cancels_timer_and_sets_shutdown_flag():
    fake = _fake_base()

    Phase2FakeBase.stop_publication(fake)

    assert fake._shutting_down is True
    assert fake._timer.cancel_count == 1


def test_timer_callback_does_not_publish_when_context_invalid():
    fake = _fake_base(valid_context=False)

    Phase2FakeBase._timer_cb(fake)

    assert fake._odom_pub.count == 0
    assert fake._tf_broadcaster.count == 0


def test_timer_callback_does_not_publish_after_shutdown_flag():
    fake = _fake_base(valid_context=True)
    fake._shutting_down = True

    Phase2FakeBase._timer_cb(fake)

    assert fake._odom_pub.count == 0
    assert fake._tf_broadcaster.count == 0


def test_valid_context_publication_errors_are_not_hidden():
    fake = _fake_base(valid_context=True, publish_exc=RuntimeError("publish failed"))

    with pytest.raises(RuntimeError):
        Phase2FakeBase._timer_cb(fake)


def test_shutdown_context_once_skips_when_already_shutdown(monkeypatch):
    calls = []
    monkeypatch.setattr("parking_robot_bringup.phase2_fake_base.rclpy.ok", lambda: False)
    monkeypatch.setattr(
        "parking_robot_bringup.phase2_fake_base.rclpy.shutdown",
        lambda: calls.append("shutdown"),
    )

    _shutdown_context_once()

    assert calls == []


def test_shutdown_context_once_calls_shutdown_only_when_ok(monkeypatch):
    calls = []
    monkeypatch.setattr("parking_robot_bringup.phase2_fake_base.rclpy.ok", lambda: True)
    monkeypatch.setattr(
        "parking_robot_bringup.phase2_fake_base.rclpy.shutdown",
        lambda: calls.append("shutdown"),
    )

    _shutdown_context_once()

    assert calls == ["shutdown"]


def test_expected_spin_shutdown_exception_exits_cleanly():
    from rclpy.executors import ExternalShutdownException

    def raise_expected(_node):
        raise ExternalShutdownException()

    _spin_until_shutdown(object(), spin_fn=raise_expected)


def test_context_invalid_spin_rclerror_exits_cleanly_when_context_not_ok():
    class _Context:
        def ok(self):
            return False

    class _Node:
        context = _Context()

    def raise_context_invalid(_node):
        raise RCLError("failed to initialize wait set: the given context is not valid")

    _spin_until_shutdown(_Node(), spin_fn=raise_context_invalid)


def test_unrelated_spin_rclerror_is_not_hidden_when_context_ok():
    class _Context:
        def ok(self):
            return True

    class _Node:
        context = _Context()

    def raise_unrelated(_node):
        raise RCLError("unexpected rcl error while context is valid")

    with pytest.raises(RCLError):
        _spin_until_shutdown(_Node(), spin_fn=raise_unrelated)
