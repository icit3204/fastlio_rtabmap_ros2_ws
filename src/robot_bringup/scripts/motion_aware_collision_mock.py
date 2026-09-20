#!/usr/bin/env python3
"""Experimental, isolated motion-aware command filter.

This node is deliberately incapable of commanding the MK-mini.  It consumes
the Nav2 command and the two qualified experimental perception branches, then
publishes only ``/cmd_vel_motion_aware_mock`` plus JSON diagnostics.  The
fixed-qualified Collision Monitor and Generic Gate remain authoritative.
"""

from dataclasses import dataclass
import json
import math
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
                "base_frame": "base_footprint",
                "max_points": 3,
                "slowdown_ratio": 0.30,
                "deescalation_observations": 2,
                "source_timeout_sec": 0.50,
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
            self.base_frame = str(value("base_frame"))
            self.sensor_pose = (float(value("tmini_sensor_x")),
                                float(value("tmini_sensor_y")),
                                float(value("tmini_sensor_yaw")))
            output_topic = str(value("output_topic"))
            forbidden = {"/vehicle_cmd_safe", "/cmd_vel", "/cmd_vel_collision_monitor"}
            if output_topic in forbidden or not output_topic.endswith("_mock"):
                raise RuntimeError("experimental output must be an isolated *_mock topic")

            self.publisher = self.create_publisher(Twist, output_topic, 10)
            self.state_publisher = self.create_publisher(String, str(value("state_topic")), 10)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.policy = RiskStatePolicy(self.mock.deescalation_observations)
            self.command = None
            self.command_time = None
            self.mid_points: Tuple[Point, ...] = ()
            self.mid_time = None
            self.tmini_points: Tuple[Point, ...] = ()
            self.tmini_time = None
            self.mid_valid = False
            self.tmini_valid = False
            self.create_subscription(Twist, str(value("input_cmd_topic")),
                                     self.command_callback, 10)
            self.create_subscription(PointCloud2, str(value("mid_topic")),
                                     self.mid_callback, qos_profile_sensor_data)
            self.create_subscription(LaserScan, str(value("tmini_topic")),
                                     self.tmini_callback, qos_profile_sensor_data)
            self.create_timer(0.05, self.tick)
            self.get_logger().warning(
                f"R23-R3 MOCK ONLY: publishing {output_topic}; no physical authority")

        def command_callback(self, message: Twist) -> None:
            self.command = message
            self.command_time = self.get_clock().now()

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
                self.mid_points = tuple(points)
                self.mid_valid = True
                self.mid_time = self.get_clock().now()
            except (TransformException, ValueError, IndexError) as error:
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
            self.tmini_points = tuple(points)
            self.tmini_valid = True
            self.tmini_time = self.get_clock().now()

        def _age(self, timestamp, now) -> float:
            return math.inf if timestamp is None else (now - timestamp).nanoseconds * 1.0e-9

        def tick(self) -> None:
            now = self.get_clock().now()
            command_age = self._age(self.command_time, now)
            mid_age = self._age(self.mid_time, now)
            tmini_age = self._age(self.tmini_time, now)
            sources_valid = (self.mid_valid and self.tmini_valid and
                             mid_age <= self.source_timeout and
                             tmini_age <= self.source_timeout)
            command_valid = (self.command is not None and
                             command_age <= self.geometry.command_timeout_sec)
            fallback = not (command_valid and sources_valid)
            if fallback:
                zones = build_zones(0.0, 0.0, math.inf, self.geometry)
                counts = observation_counts(self.mid_points, self.tmini_points, zones)
                state = self.policy.force_stop()
                input_linear = input_angular = 0.0
            else:
                input_linear = float(self.command.linear.x)
                input_angular = float(self.command.angular.z)
                zones = build_zones(input_linear, input_angular, command_age, self.geometry)
                if zones.state != "MOTION_AWARE":
                    fallback = True
                    state = self.policy.force_stop()
                counts = observation_counts(self.mid_points, self.tmini_points, zones)
                if not fallback:
                    state = self.policy.update(classify_counts(counts, self.mock.max_points))
            output_linear, output_angular = filter_command(
                input_linear, input_angular, state, self.mock.slowdown_ratio)
            output = Twist()
            output.linear.x = output_linear
            output.angular.z = output_angular
            self.publisher.publish(output)
            diagnostic = String()
            diagnostic.data = json.dumps({
                "state": STATE_NAMES[state], "fallback": fallback,
                "input_v": input_linear, "input_w": input_angular,
                "output_v": output_linear, "output_w": output_angular,
                "mid_stop": counts.mid_stop, "mid_slow": counts.mid_slow,
                "tmini_stop": counts.tmini_stop, "tmini_slow": counts.tmini_slow,
                "command_age_sec": command_age, "mid_age_sec": mid_age,
                "tmini_age_sec": tmini_age, "polygon_state": zones.state,
            }, sort_keys=True)
            self.state_publisher.publish(diagnostic)

    rclpy.init()
    node = MotionAwareCollisionMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
