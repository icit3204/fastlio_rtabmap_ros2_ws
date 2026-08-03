"""Shared process-wide ROS 2 runtime ownership for plan_nav."""

from __future__ import annotations

import threading

_LOCK = threading.RLock()
_INITIALIZED_BY_PLAN_NAV = False
_SHUTDOWN_CALLED = False


def ensure_rclpy_initialized(args=None):
    """Initialize rclpy at most once and return the imported module."""
    global _INITIALIZED_BY_PLAN_NAV, _SHUTDOWN_CALLED
    import rclpy

    with _LOCK:
        if not rclpy.ok():
            rclpy.init(args=args if args is not None else [])
            _INITIALIZED_BY_PLAN_NAV = True
            _SHUTDOWN_CALLED = False
        return rclpy


def shutdown_rclpy_once() -> None:
    """Shutdown rclpy once, after all plan_nav ROS threads have stopped."""
    global _SHUTDOWN_CALLED
    import rclpy

    with _LOCK:
        if rclpy.ok() and not _SHUTDOWN_CALLED:
            rclpy.shutdown()
            _SHUTDOWN_CALLED = True

