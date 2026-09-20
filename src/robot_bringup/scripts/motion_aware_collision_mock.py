#!/usr/bin/env python3
"""Qualified motion-aware command filter with explicit authority modes.

The default remains an isolated mock output. Physical-launch enforcement is
accepted only when an explicit parameter binds the output to ``/cmd_vel``,
which is the existing Generic Gate input. The node can never publish directly
to ``/vehicle_cmd_safe`` or a chassis/backend topic.
"""

from dataclasses import dataclass
import json
import math
import threading
import time
from typing import Iterable, Sequence, Tuple

try:
    from motion_aware_collision_geometry import (
        GeometryConfig, Point, ZoneResult, build_zones, point_in_polygon)
except ModuleNotFoundError:
    # CMake installs ROS executables without the source ``.py`` suffix. Load
    # the adjacent, already-qualified geometry executable explicitly there.
    import importlib.machinery
    import importlib.util
    from pathlib import Path

    geometry_path = Path(__file__).with_name("motion_aware_collision_geometry")
    loader = importlib.machinery.SourceFileLoader(
        "motion_aware_collision_geometry", str(geometry_path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    geometry_module = importlib.util.module_from_spec(spec)
    loader.exec_module(geometry_module)
    GeometryConfig = geometry_module.GeometryConfig
    Point = geometry_module.Point
    ZoneResult = geometry_module.ZoneResult
    build_zones = geometry_module.build_zones
    point_in_polygon = geometry_module.point_in_polygon


CLEAR = 0
SLOW = 1
STOP = 2
STATE_NAMES = ("CLEAR", "SLOW", "STOP")


@dataclass(frozen=True)
class MockConfig:
    max_points: int = 3
    slowdown_ratio: float = 0.30
    deescalation_observations: int = 2


@dataclass(frozen=True)
class StaleTimingConfig:
    """Monotonic deadman schedule with margin ahead of the policy limit."""

    stale_threshold_sec: float = 0.250
    zero_deadline_sec: float = 0.225
    watchdog_period_sec: float = 0.005

    def validate(self) -> None:
        values = (self.stale_threshold_sec, self.zero_deadline_sec,
                  self.watchdog_period_sec)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("stale timing values must be finite and positive")
        if self.zero_deadline_sec >= self.stale_threshold_sec:
            raise ValueError("zero deadline must retain margin before stale threshold")


@dataclass(frozen=True)
class Counts:
    mid_stop: int
    mid_slow: int
    tmini_stop: int
    tmini_slow: int

    @property
    def stop(self) -> int:
        return self.mid_stop + self.tmini_stop

    @property
    def slow(self) -> int:
        return self.mid_slow + self.tmini_slow


def count_points(points: Iterable[Point], zones: ZoneResult) -> Tuple[int, int]:
    stop = slow = 0
    for point in points:
        if not all(math.isfinite(value) for value in point):
            continue
        if point_in_polygon(point, zones.stop):
            stop += 1
        if point_in_polygon(point, zones.slow):
            slow += 1
    return stop, slow


def observation_counts(
    mid_points: Iterable[Point], tmini_points: Iterable[Point], zones: ZoneResult
) -> Counts:
    mid_stop, mid_slow = count_points(mid_points, zones)
    tmini_stop, tmini_slow = count_points(tmini_points, zones)
    return Counts(mid_stop, mid_slow, tmini_stop, tmini_slow)


def classify_counts(counts: Counts, max_points: int = 3) -> int:
    """Match Nav2 Collision Monitor semantics: action when count > max_points."""
    if counts.stop > max_points:
        return STOP
    if counts.slow > max_points:
        return SLOW
    return CLEAR


def filter_command(linear_x: float, angular_z: float, state: int,
                   slowdown_ratio: float = 0.30) -> Tuple[float, float]:
    if not all(math.isfinite(value) for value in (linear_x, angular_z)):
        return 0.0, 0.0
    if state == STOP:
        return 0.0, 0.0
    if state == SLOW:
        return linear_x * slowdown_ratio, angular_z * slowdown_ratio
    if state == CLEAR:
        return linear_x, angular_z
    return 0.0, 0.0


class RiskStatePolicy:
    """Immediate escalation; two stable observations before de-escalation."""

    def __init__(self, deescalation_observations: int = 2) -> None:
        if deescalation_observations < 1:
            raise ValueError("deescalation_observations must be >= 1")
        self.required = deescalation_observations
        self.state = CLEAR
        self.pending = None
        self.pending_count = 0

    def force_stop(self) -> int:
        self.state = STOP
        self.pending = None
        self.pending_count = 0
        return self.state

    def update(self, candidate: int) -> int:
        if candidate not in (CLEAR, SLOW, STOP):
            return self.force_stop()
        if candidate >= self.state:
            self.state = candidate
            self.pending = None
            self.pending_count = 0
            return self.state
        if self.pending != candidate:
            self.pending = candidate
            self.pending_count = 1
        else:
            self.pending_count += 1
        if self.pending_count >= self.required:
            self.state = candidate
            self.pending = None
            self.pending_count = 0
        return self.state


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    class MotionAwareCollisionMock(Node):
        def __init__(self) -> None:
            super().__init__("motion_aware_collision_mock")
            geometry_defaults = GeometryConfig()
            geometry_values = {}
            for name in geometry_defaults.__dataclass_fields__:
                self.declare_parameter(name, getattr(geometry_defaults, name))
                geometry_values[name] = self.get_parameter(name).value
            self.geometry = GeometryConfig(**geometry_values)

            defaults = {
                "input_cmd_topic": "/cmd_vel_nav",
                "mid_topic": "/cloud_registered_nav2_obstacles",
                "tmini_topic": "/scan_collision_experimental",
                "output_topic": "/cmd_vel_motion_aware_mock",
                "state_topic": "/motion_aware_collision_mock/state",
                "integration_mock_enabled": False,
                "physical_enforcement_enabled": False,
                "base_frame": "base_footprint",
                "max_points": 3,
                "slowdown_ratio": 0.30,
                "deescalation_observations": 2,
                "source_timeout_sec": 0.50,
                "watchdog_zero_deadline_sec": 0.225,
                "watchdog_period_sec": 0.005,
                "tmini_sensor_x": 0.703,
                "tmini_sensor_y": 0.0,
                "tmini_sensor_yaw": 0.0,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            value = lambda name: self.get_parameter(name).value
            self.mock = MockConfig(int(value("max_points")), float(value("slowdown_ratio")),
                                   int(value("deescalation_observations")))
            self.source_timeout = float(value("source_timeout_sec"))
            self.stale_timing = StaleTimingConfig(
                stale_threshold_sec=float(self.geometry.command_timeout_sec),
                zero_deadline_sec=float(value("watchdog_zero_deadline_sec")),
                watchdog_period_sec=float(value("watchdog_period_sec")))
            self.stale_timing.validate()
            self.base_frame = str(value("base_frame"))
            self.sensor_pose = (float(value("tmini_sensor_x")),
                                float(value("tmini_sensor_y")),
                                float(value("tmini_sensor_yaw")))
            output_topic = str(value("output_topic"))
            forbidden = {"/vehicle_cmd_safe", "/cmd_vel_collision_monitor"}
            integration_mock = bool(value("integration_mock_enabled"))
            physical_enforcement = bool(value("physical_enforcement_enabled"))
            integration_topic = "/r23_r5/collision_selected"
            if integration_mock and physical_enforcement:
                raise RuntimeError("mock and physical integration modes are exclusive")
            output_allowed = (
                (not integration_mock and not physical_enforcement and
                 output_topic.endswith("_mock")) or
                (integration_mock and output_topic == integration_topic) or
                (physical_enforcement and output_topic == "/cmd_vel")
            )
            if output_topic in forbidden or not output_allowed:
                raise RuntimeError("motion-aware output authority contract rejected")

            self.publisher = self.create_publisher(Twist, output_topic, 10)
            self.state_publisher = self.create_publisher(String, str(value("state_topic")), 10)
            self.command_group = MutuallyExclusiveCallbackGroup()
            self.sensor_group = ReentrantCallbackGroup()
            self.observation_group = MutuallyExclusiveCallbackGroup()
            self.watchdog_group = MutuallyExclusiveCallbackGroup()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.policy = RiskStatePolicy(self.mock.deescalation_observations)
            self.state_lock = threading.Lock()
            self.publish_lock = threading.Lock()
            self.command = None
            self.command_signature = None
            self.command_time = None
            self.command_accepted_monotonic_ns = None
            self.command_generation = 0
            self.watchdog_latched_generation = None
            self.mid_points: Tuple[Point, ...] = ()
            self.mid_time = None
            self.tmini_points: Tuple[Point, ...] = ()
            self.tmini_time = None
            self.mid_valid = False
            self.tmini_valid = False
            self.create_subscription(Twist, str(value("input_cmd_topic")),
                                     self.command_callback, 10,
                                     callback_group=self.command_group)
            self.create_subscription(PointCloud2, str(value("mid_topic")),
                                     self.mid_callback, qos_profile_sensor_data,
                                     callback_group=self.sensor_group)
            self.create_subscription(LaserScan, str(value("tmini_topic")),
                                     self.tmini_callback, qos_profile_sensor_data,
                                     callback_group=self.sensor_group)
            self.create_timer(0.05, self.tick,
                              callback_group=self.observation_group)
            # This short, independent watchdog never waits for a sensor callback
            # to invoke the command deadman.  The 0.225 s publication deadline
            # preserves 25 ms scheduling margin inside the exact 0.250 s stale
            # policy boundary.
            self.create_timer(self.stale_timing.watchdog_period_sec,
                              self.watchdog_tick,
                              callback_group=self.watchdog_group)
            authority = ("PHYSICAL_GATE_INPUT" if physical_enforcement else
                         "INTEGRATION_MOCK" if integration_mock else "ISOLATED_MOCK")
            self.get_logger().warning(
                f"ACTIVE_COLLISION_MODE=motion_aware_experimental "
                f"authority={authority} output={output_topic}")

        def command_callback(self, message: Twist) -> None:
            accepted_ns = time.monotonic_ns()
            signature = (
                float(message.linear.x), float(message.linear.y),
                float(message.linear.z), float(message.angular.x),
                float(message.angular.y), float(message.angular.z),
            )
            with self.state_lock:
                self.command = message
                self.command_time = self.get_clock().now()
                self.command_accepted_monotonic_ns = accepted_ns
                # Repeated controller refreshes of the same command must not
                # invalidate an in-flight live-cloud classification. A real
                # command change still advances the generation and discards
                # any result computed for the prior geometry.
                if signature != self.command_signature:
                    self.command_generation += 1
                    self.command_signature = signature
                self.watchdog_latched_generation = None

        @staticmethod
        def _transform(point, transform) -> Point:
            q = transform.transform.rotation
            # Full quaternion rotation, retaining only resulting x/y.
            vx, vy, vz = point
            tx = 2.0 * (q.y * vz - q.z * vy)
            ty = 2.0 * (q.z * vx - q.x * vz)
            tz = 2.0 * (q.x * vy - q.y * vx)
            rx = vx + q.w * tx + (q.y * tz - q.z * ty)
            ry = vy + q.w * ty + (q.z * tx - q.x * tz)
            return (rx + transform.transform.translation.x,
                    ry + transform.transform.translation.y)

        def mid_callback(self, message: PointCloud2) -> None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, message.header.frame_id, message.header.stamp,
                    timeout=Duration(seconds=0.05))
                points = []
                for xyz in point_cloud2.read_points(
                        message, field_names=("x", "y", "z"), skip_nans=True):
                    points.append(self._transform((float(xyz[0]), float(xyz[1]), float(xyz[2])),
                                                  transform))
                with self.state_lock:
                    self.mid_points = tuple(points)
                    self.mid_valid = True
                    self.mid_time = time.monotonic_ns()
            except (TransformException, ValueError, IndexError) as error:
                with self.state_lock:
                    self.mid_valid = False
                self.get_logger().error(f"MID observation rejected: {error}")

        def tmini_callback(self, message: LaserScan) -> None:
            if message.header.frame_id != "laser_frame":
                self.tmini_valid = False
                return
            sx, sy, syaw = self.sensor_pose
            points = []
            for index, value in enumerate(message.ranges):
                distance = float(value)
                if not math.isfinite(distance) or not message.range_min <= distance <= message.range_max:
                    continue
                angle = message.angle_min + index * message.angle_increment + syaw
                points.append((sx + distance * math.cos(angle),
                               sy + distance * math.sin(angle)))
            with self.state_lock:
                self.tmini_points = tuple(points)
                self.tmini_valid = True
                self.tmini_time = time.monotonic_ns()

        @staticmethod
        def _monotonic_age(timestamp_ns, now_ns) -> float:
            return math.inf if timestamp_ns is None else (now_ns - timestamp_ns) * 1.0e-9

        def _publish(self, state, fallback, input_linear, input_angular,
                     counts, zones, command_age, mid_age, tmini_age,
                     accepted_ns, fallback_decision_ns=0,
                     zero_publication_ns=0, reason="observation") -> None:
            output_linear, output_angular = filter_command(
                input_linear, input_angular, state, self.mock.slowdown_ratio)
            output = Twist()
            output.linear.x = output_linear
            output.angular.z = output_angular
            try:
                self.publisher.publish(output)
            except Exception:
                # SIGINT can invalidate the ROS context while the independent
                # watchdog callback is already in flight. Suppress only that
                # shutdown race; runtime publication errors still propagate.
                if not rclpy.ok():
                    return
                raise
            if state == STOP and output_linear == 0.0 and output_angular == 0.0:
                zero_publication_ns = time.monotonic_ns()
            threshold_ns = (0 if accepted_ns is None else
                            accepted_ns + int(self.stale_timing.stale_threshold_sec * 1e9))
            diagnostic = String()
            diagnostic.data = json.dumps({
                "state": STATE_NAMES[state], "fallback": fallback,
                "fallback_reason": reason,
                "input_v": input_linear, "input_w": input_angular,
                "output_v": output_linear, "output_w": output_angular,
                "mid_stop": counts.mid_stop, "mid_slow": counts.mid_slow,
                "tmini_stop": counts.tmini_stop, "tmini_slow": counts.tmini_slow,
                "command_age_sec": command_age, "mid_age_sec": mid_age,
                "tmini_age_sec": tmini_age, "polygon_state": zones.state,
                "last_command_accepted_monotonic_ns": accepted_ns or 0,
                "stale_threshold_crossing_monotonic_ns": threshold_ns,
                "fallback_decision_monotonic_ns": fallback_decision_ns,
                "zero_publication_monotonic_ns": zero_publication_ns,
                "zero_publication_command_age_sec": (
                    math.inf if accepted_ns is None or zero_publication_ns == 0 else
                    (zero_publication_ns - accepted_ns) * 1.0e-9),
            }, sort_keys=True)
            try:
                self.state_publisher.publish(diagnostic)
            except Exception:
                if not rclpy.ok():
                    return
                raise

        def watchdog_tick(self) -> None:
            decision_ns = time.monotonic_ns()
            with self.publish_lock:
                with self.state_lock:
                    accepted_ns = self.command_accepted_monotonic_ns
                    generation = self.command_generation
                    if accepted_ns is None:
                        return
                    age = (decision_ns - accepted_ns) * 1.0e-9
                    if (age < self.stale_timing.zero_deadline_sec or
                            self.watchdog_latched_generation == generation):
                        return
                    self.watchdog_latched_generation = generation
                    mid_points = self.mid_points
                    tmini_points = self.tmini_points
                    mid_age = self._monotonic_age(self.mid_time, decision_ns)
                    tmini_age = self._monotonic_age(self.tmini_time, decision_ns)
                zones = build_zones(0.0, 0.0, math.inf, self.geometry)
                counts = observation_counts(mid_points, tmini_points, zones)
                state = self.policy.force_stop()
                self._publish(
                    state, True, 0.0, 0.0, counts, zones, age,
                    mid_age, tmini_age, accepted_ns,
                    fallback_decision_ns=decision_ns,
                    reason="COMMAND_WATCHDOG_DEADLINE")

        def tick(self) -> None:
            now_ns = time.monotonic_ns()
            with self.state_lock:
                command = self.command
                accepted_ns = self.command_accepted_monotonic_ns
                generation = self.command_generation
                latched = self.watchdog_latched_generation == generation
                mid_points = self.mid_points
                tmini_points = self.tmini_points
                mid_valid = self.mid_valid
                tmini_valid = self.tmini_valid
                mid_time = self.mid_time
                tmini_time = self.tmini_time
            command_age = self._monotonic_age(accepted_ns, now_ns)
            mid_age = self._monotonic_age(mid_time, now_ns)
            tmini_age = self._monotonic_age(tmini_time, now_ns)
            sources_valid = (mid_valid and tmini_valid and
                             mid_age <= self.source_timeout and
                             tmini_age <= self.source_timeout)
            command_valid = (command is not None and not latched and
                             command_age < self.stale_timing.zero_deadline_sec)
            fallback = not (command_valid and sources_valid)
            if fallback:
                zones = build_zones(0.0, 0.0, math.inf, self.geometry)
                counts = observation_counts(mid_points, tmini_points, zones)
                state = self.policy.force_stop()
                input_linear = input_angular = 0.0
            else:
                input_linear = float(command.linear.x)
                input_angular = float(command.angular.z)
                zones = build_zones(input_linear, input_angular, command_age, self.geometry)
                if zones.state != "MOTION_AWARE":
                    fallback = True
                    state = self.policy.force_stop()
                counts = observation_counts(mid_points, tmini_points, zones)
                if not fallback:
                    state = self.policy.update(classify_counts(counts, self.mock.max_points))
            with self.publish_lock:
                # A watchdog may have latched while point classification was
                # running. Never publish a late nonzero sample after that latch.
                with self.state_lock:
                    still_current = generation == self.command_generation
                    latched = self.watchdog_latched_generation == generation
                if not still_current:
                    return
                if latched:
                    zones = build_zones(0.0, 0.0, math.inf, self.geometry)
                    counts = observation_counts(mid_points, tmini_points, zones)
                    state = self.policy.force_stop()
                    fallback = True
                    input_linear = input_angular = 0.0
                self._publish(
                    state, fallback, input_linear, input_angular, counts, zones,
                    command_age, mid_age, tmini_age, accepted_ns,
                    fallback_decision_ns=(now_ns if fallback else 0),
                    reason=("COMMAND_WATCHDOG_LATCHED" if latched else
                            "SOURCE_OR_COMMAND_INVALID" if fallback else "observation"))

    rclpy.init()
    node = MotionAwareCollisionMock()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
