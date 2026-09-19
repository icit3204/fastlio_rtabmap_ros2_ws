"""Independent, receive-only planar-speed observation from FAST-LIO odometry.

This is deliberately an observer, not a command source.  Its speed estimate
is the displacement between consecutive finite FAST-LIO poses divided by their
advancing odometry timestamps.  A stale, non-finite, or non-advancing source
is invalid rather than silently treated as zero speed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time


@dataclass(frozen=True)
class FastLioSpeedSample:
    received_monotonic_sec: float
    stamp_sec: float
    x_m: float
    y_m: float
    z_m: float
    speed_mps: float | None
    valid: bool
    reason: str | None


class FastLioSpeedEstimator:
    """Pure consecutive-pose estimator, kept independent of ROS for tests."""

    def __init__(self) -> None:
        self._previous: FastLioSpeedSample | None = None
        self._latest: FastLioSpeedSample | None = None
        self._origin: tuple[float, float] | None = None
        self._max_speed_mps = 0.0
        self._sample_count = 0

    def observe(self, *, received_monotonic_sec: float, stamp_sec: float,
                x_m: float, y_m: float, z_m: float) -> FastLioSpeedSample:
        values = (received_monotonic_sec, stamp_sec, x_m, y_m, z_m)
        if not all(math.isfinite(value) for value in values):
            sample = FastLioSpeedSample(received_monotonic_sec, stamp_sec, x_m, y_m, z_m,
                                        None, False, "FASTLIO_ODOMETRY_NONFINITE")
            self._latest = sample
            return sample
        if self._previous is not None and stamp_sec <= self._previous.stamp_sec:
            sample = FastLioSpeedSample(received_monotonic_sec, stamp_sec, x_m, y_m, z_m,
                                        None, False, "FASTLIO_ODOMETRY_TIMESTAMP_NOT_ADVANCING")
            self._latest = sample
            return sample
        speed = None
        if self._previous is not None:
            dt = stamp_sec - self._previous.stamp_sec
            speed = math.hypot(x_m - self._previous.x_m, y_m - self._previous.y_m) / dt
            self._max_speed_mps = max(self._max_speed_mps, speed)
        sample = FastLioSpeedSample(received_monotonic_sec, stamp_sec, x_m, y_m, z_m,
                                    speed, True, None)
        self._previous = sample
        self._latest = sample
        self._sample_count += 1
        if self._origin is None:
            self._origin = (x_m, y_m)
        return sample

    def snapshot(self, now_monotonic_sec: float, *, freshness_sec: float = .30) -> dict:
        latest = self._latest
        if latest is None:
            return {"valid": False, "reason": "FASTLIO_ODOMETRY_MISSING", "sample_count": 0}
        age = now_monotonic_sec - latest.received_monotonic_sec
        valid = latest.valid and age <= freshness_sec
        reason = latest.reason if not latest.valid else (None if age <= freshness_sec else "FASTLIO_ODOMETRY_STALE")
        displacement = None
        if self._origin is not None:
            displacement = math.hypot(latest.x_m - self._origin[0], latest.y_m - self._origin[1])
        return {
            "valid": valid, "reason": reason, "sample_count": self._sample_count,
            "age_sec": age, "speed_mps": latest.speed_mps,
            "max_speed_mps": self._max_speed_mps,
            "x_m": latest.x_m, "y_m": latest.y_m, "z_m": latest.z_m,
            "stamp_sec": latest.stamp_sec, "displacement_from_origin_m": displacement,
        }

    def reset_distance_origin(self) -> None:
        if self._latest is None:
            self._origin = None
        else:
            self._origin = (self._latest.x_m, self._latest.y_m)


class FastLioSpeedMonitor:
    """Threaded ROS subscription with no publisher and no command authority."""

    def __init__(self, topic: str = "/Odometry") -> None:
        self._topic = topic
        self._estimator = FastLioSpeedEstimator()
        self._lock = threading.Lock()
        self._executor = None
        self._node = None
        self._thread = None
        self._owns_rclpy_context = False

    def open(self) -> None:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy_context = True
        self._node = rclpy.create_node("mkmini_fastlio_speed_observer")
        self._node.create_subscription(Odometry, self._topic, self._callback, 20)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin,
                                        name="mkmini-fastlio-speed-rx", daemon=True)
        self._thread.start()

    def _callback(self, message) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        position = message.pose.pose.position
        with self._lock:
            self._estimator.observe(received_monotonic_sec=time.monotonic(), stamp_sec=stamp,
                                    x_m=position.x, y_m=position.y, z_m=position.z)

    def snapshot(self, now_monotonic_sec: float | None = None) -> dict:
        with self._lock:
            return self._estimator.snapshot(time.monotonic() if now_monotonic_sec is None else now_monotonic_sec)

    def reset_distance_origin(self) -> None:
        with self._lock:
            self._estimator.reset_distance_origin()

    def close(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._owns_rclpy_context:
            import rclpy
            rclpy.shutdown()


__all__ = ["FastLioSpeedEstimator", "FastLioSpeedMonitor", "FastLioSpeedSample"]
