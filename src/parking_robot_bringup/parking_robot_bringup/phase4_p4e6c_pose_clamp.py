"""Qualification-owned persistent odom-frame pose clamp.

This module is deliberately independent of evidence polling and shell CLI
processes.  It publishes one fixed odometry anchor on ``/initialpose`` at a
monotonic 10 Hz schedule and retains acceptance observations from /Odometry.
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


PERIOD_SEC = 0.100


def clamp_warmup_status(publication_count: int, *, required: int = 5) -> str:
    """Qualification warmup authority: actual publication rows only."""
    return "CLAMP_WARMUP_READY" if int(publication_count) >= int(required) else "WAIT"


def validate_clamp_snapshot(snapshot: dict, *, minimum_publications: int | None = 27,
                            duration_aware: bool = False) -> dict:
    """Fail-closed actual cadence and fixed-frame acceptance contract."""
    events = list(snapshot.get("events", []))
    intervals = [float(e["actual_interval_sec"]) for e in events
                 if e.get("actual_interval_sec") is not None]
    if snapshot.get("frame_id") != "odom" or any(e.get("frame_id") != "odom" for e in events):
        raise RuntimeError("STIMULATOR_FRAME_CONTRACT_FAILURE")
    required = (max(5, int(float(snapshot.get("elapsed_sec", 0.0)) * 9.0))
                if duration_aware else int(minimum_publications))
    if len(events) < required:
        raise RuntimeError("STIMULATOR_INSUFFICIENT_PUBLICATIONS")
    if not intervals or snapshot.get("effective_frequency_hz", 0.0) < 9.0 \
            or snapshot.get("effective_frequency_hz", 0.0) > 11.0:
        raise RuntimeError("STIMULATOR_CADENCE_FAILURE")
    if max(intervals) > 0.200:
        raise RuntimeError("STIMULATOR_PUBLICATION_GAP_FAILURE")
    if snapshot.get("anchor", {}).get("frame_id") != "odom":
        raise RuntimeError("STIMULATOR_ANCHOR_FRAME_FAILURE")
    return {"pass": True, "publication_count": len(events),
            "effective_frequency_hz": snapshot["effective_frequency_hz"],
            "minimum_publications_required": required,
            "median_interval_sec": snapshot.get("median_interval_sec"),
            "max_interval_sec": max(intervals)}


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PersistentPoseClamp:
    """One persistent publisher plus an independent monotonic scheduler."""

    def __init__(self, *, transaction_id: str, output: Path, event_callback=None,
                 acceptance_callback=None):
        self.transaction_id = str(transaction_id)
        self.output = Path(output)
        self.event_callback = event_callback
        self.acceptance_callback = acceptance_callback
        self.node = Node("phase4_p4e6c_pose_clamp")
        self.publisher = self.node.create_publisher(PoseWithCovarianceStamped,
                                                     "/initialpose", 10)
        self.node.create_subscription(Odometry, "/Odometry", self._odom_cb, 50)
        self._lock = threading.Lock()
        self._latest_odom = None
        self.anchor = None
        self.events = []
        self.acceptance_events = []
        self._executor_thread = None
        self._scheduler_thread = None
        self._stop = threading.Event()
        self._started = False
        self._first_tick = threading.Event()

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._latest_odom = msg
            anchor = self.anchor
        if anchor is None or not self.events:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        distance = math.hypot(float(p.x) - anchor["x"], float(p.y) - anchor["y"])
        yaw_error = abs(_yaw_from_quaternion(q) - anchor["yaw"])
        yaw_error = min(yaw_error, 2.0 * math.pi - yaw_error)
        acceptance = {
            "transaction_id": self.transaction_id,
            "monotonic_ns": time.monotonic_ns(),
            "distance_error_m": distance,
            "yaw_error_rad": yaw_error,
            "accepted": bool(distance <= 0.10 and yaw_error <= 0.05),
        }
        self.acceptance_events.append(acceptance)
        if self.acceptance_callback is not None:
            try:
                self.acceptance_callback(dict(acceptance))
            except Exception as exc:
                self.node.get_logger().error("acceptance evidence callback failed: %s", exc)

    def start_executor(self):
        if self._executor_thread is not None:
            return
        self._executor_thread = threading.Thread(target=rclpy.spin,
                                                  args=(self.node,), daemon=True)
        self._executor_thread.start()

    def capture_anchor(self, *, timeout_sec: float = 5.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                msg = self._latest_odom
            if msg is not None:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                values = (float(p.x), float(p.y), float(p.z),
                          float(q.x), float(q.y), float(q.z), float(q.w))
                if all(math.isfinite(v) for v in values) and str(msg.header.frame_id) == "odom":
                    self.anchor = {"x": values[0], "y": values[1], "yaw": _yaw_from_quaternion(q),
                                   "orientation": {"x": values[3], "y": values[4],
                                                   "z": values[5], "w": values[6]},
                                   "frame_id": "odom", "transaction_id": self.transaction_id,
                                   "capture_monotonic_ns": time.monotonic_ns()}
                    return self.anchor
            time.sleep(0.01)
        raise RuntimeError("STIMULATOR_ANCHOR_NOT_READY")

    def ready(self):
        if self.anchor is None:
            raise RuntimeError("STIMULATOR_NOT_READY")
        if self.anchor.get("frame_id") != "odom":
            raise RuntimeError("STIMULATOR_NOT_READY")
        if int(self.publisher.get_subscription_count()) < 1:
            raise RuntimeError("STIMULATOR_PUBLISHER_UNMATCHED")
        return {"persistent_publisher": True, "topic": "/initialpose",
                "message_type": "geometry_msgs/msg/PoseWithCovarianceStamped",
                "frame_id": "odom", "anchor": self.anchor,
                "scheduler_period_sec": PERIOD_SEC, "pass": True}

    def _message(self):
        if self.anchor is None or self.anchor.get("frame_id") != "odom":
            raise RuntimeError("STIMULATOR_NOT_READY")
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.anchor["x"]
        msg.pose.pose.position.y = self.anchor["y"]
        msg.pose.pose.position.z = 0.0
        o = self.anchor["orientation"]
        msg.pose.pose.orientation.x = o["x"]
        msg.pose.pose.orientation.y = o["y"]
        msg.pose.pose.orientation.z = o["z"]
        msg.pose.pose.orientation.w = o["w"]
        # The accepted fake-base reset contract does not require covariance;
        # retain the established zero covariance shape.
        msg.pose.covariance = [0.0] * 36
        return msg

    def _schedule(self, duration_sec: float):
        start = time.monotonic()
        deadline = start
        end = start + float(duration_sec)
        while not self._stop.is_set() and deadline <= end:
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            if self._stop.is_set():
                break
            actual = time.monotonic_ns()
            self.publisher.publish(self._message())
            with self._lock:
                previous = self.events[-1]["actual_publish_monotonic_ns"] if self.events else None
                event = {"transaction_id": self.transaction_id,
                                    "scheduled_monotonic_ns": int(deadline * 1e9),
                                    "actual_publish_monotonic_ns": actual,
                                    "actual_interval_sec": None if previous is None else
                                    (actual - previous) / 1e9,
                                    "frame_id": "odom", "anchor_x": self.anchor["x"],
                                    "anchor_y": self.anchor["y"]}
                self.events.append(event)
            if self.event_callback is not None:
                try:
                    self.event_callback(dict(event))
                except Exception as exc:
                    self.node.get_logger().error("clamp evidence callback failed: %s", exc)
            self._first_tick.set()
            deadline += PERIOD_SEC

    def run(self, duration_sec: float):
        self.ready()
        self._stop.clear()
        self._started = True
        self._scheduler_thread = threading.Thread(target=self._schedule,
                                                   args=(duration_sec,), daemon=True)
        self._scheduler_thread.start()
        return self._scheduler_thread

    def wait_first_tick(self, timeout_sec: float = 0.2):
        if not self._first_tick.wait(timeout_sec):
            raise RuntimeError("STIMULATOR_FIRST_TICK_FAILURE")

    def stop(self):
        self._stop.set()
        if self._scheduler_thread is not None:
            self._scheduler_thread.join(timeout=2.0)
            if self._scheduler_thread.is_alive():
                raise RuntimeError("STIMULATOR_THREAD_JOIN_TIMEOUT")
        return self.snapshot()

    def snapshot(self):
        with self._lock:
            events = list(self.events)
            acceptance = list(self.acceptance_events)
        intervals = [float(e["actual_interval_sec"]) for e in events
                      if e.get("actual_interval_sec") is not None]
        elapsed = ((events[-1]["actual_publish_monotonic_ns"] -
                    events[0]["actual_publish_monotonic_ns"]) / 1e9
                   if len(events) > 1 else 0.0)
        return {"transaction_id": self.transaction_id, "frame_id": "odom",
                "publication_count": len(events), "events": events,
                "acceptance_events": acceptance,
                "accepted_count": sum(1 for e in acceptance if e.get("accepted")),
                "anchor": self.anchor, "elapsed_sec": elapsed,
                "effective_frequency_hz": ((len(intervals) / elapsed) if elapsed > 0 else 0.0),
                "mean_interval_sec": (sum(intervals) / len(intervals) if intervals else None),
                "median_interval_sec": (__import__("statistics").median(intervals) if intervals else None),
                "max_interval_sec": (max(intervals) if intervals else None)}

    def destroy(self):
        self.stop()
        self.node.destroy_node()
