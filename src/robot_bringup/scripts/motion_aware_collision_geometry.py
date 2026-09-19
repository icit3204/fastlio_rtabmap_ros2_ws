#!/usr/bin/env python3
"""Shadow-only short-horizon Ackermann collision-zone geometry.

This executable publishes experimental STOP/SLOW PolygonStamped geometry for
review. It does not subscribe to obstacle data and cannot modify cmd_vel. The
qualified fixed Collision Monitor remains the active safety authority.
"""

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple


Point = Tuple[float, float]


@dataclass(frozen=True)
class GeometryConfig:
    footprint_x_min: float = -0.145
    footprint_x_max: float = 0.755
    footprint_y_min: float = -0.300
    footprint_y_max: float = 0.300
    footprint_padding: float = 0.030
    minimum_turning_radius: float = 1.750
    stop_horizon_sec: float = 2.0
    slow_horizon_sec: float = 5.0
    stop_base_distance: float = 0.10
    slow_base_distance: float = 0.15
    stop_max_distance: float = 0.45
    slow_max_distance: float = 0.80
    stop_extra_margin: float = 0.02
    slow_extra_margin: float = 0.12
    sample_step: float = 0.025
    command_timeout_sec: float = 0.25
    zero_speed_epsilon: float = 1.0e-4


@dataclass(frozen=True)
class ZoneResult:
    stop: Tuple[Point, ...]
    slow: Tuple[Point, ...]
    state: str
    curvature: float
    stop_distance: float
    slow_distance: float


def _cross(origin: Point, a: Point, b: Point) -> float:
    return ((a[0] - origin[0]) * (b[1] - origin[1]) -
            (a[1] - origin[1]) * (b[0] - origin[0]))


def convex_hull(points: Iterable[Point]) -> Tuple[Point, ...]:
    """Return the counter-clockwise convex hull without repeating its start."""
    unique = sorted(set(points))
    if len(unique) <= 1:
        return tuple(unique)
    lower: List[Point] = []
    for point in unique:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: List[Point] = []
    for point in reversed(unique):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])


def _expanded_footprint(config: GeometryConfig, extra_margin: float) -> Tuple[Point, ...]:
    margin = config.footprint_padding + extra_margin
    return (
        (config.footprint_x_min - margin, config.footprint_y_min - margin),
        (config.footprint_x_max + margin, config.footprint_y_min - margin),
        (config.footprint_x_max + margin, config.footprint_y_max + margin),
        (config.footprint_x_min - margin, config.footprint_y_max + margin),
    )


def trajectory_pose(distance: float, curvature: float) -> Tuple[float, float, float]:
    if abs(curvature) < 1.0e-12:
        return distance, 0.0, 0.0
    yaw = distance * curvature
    return math.sin(yaw) / curvature, (1.0 - math.cos(yaw)) / curvature, yaw


def transform_polygon(points: Sequence[Point], pose: Tuple[float, float, float]) -> Tuple[Point, ...]:
    x, y, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple(
        (x + cosine * px - sine * py, y + sine * px + cosine * py)
        for px, py in points)


def swept_polygon(
    config: GeometryConfig, distance: float, curvature: float, extra_margin: float
) -> Tuple[Point, ...]:
    footprint = _expanded_footprint(config, extra_margin)
    steps = max(1, int(math.ceil(distance / config.sample_step)))
    vertices: List[Point] = []
    for index in range(steps + 1):
        travel = distance * index / steps
        vertices.extend(transform_polygon(footprint, trajectory_pose(travel, curvature)))
    return convex_hull(vertices)


def fallback_zones(config: GeometryConfig) -> ZoneResult:
    """Conservative fixed fallback: padded body union qualified R22 zones."""
    stop_points = list(_expanded_footprint(config, config.stop_extra_margin))
    stop_points.extend(((0.785, -0.53), (1.05, -0.53), (1.05, 0.53), (0.785, 0.53)))
    slow_points = list(_expanded_footprint(config, config.slow_extra_margin))
    slow_points.extend(((0.785, -0.53), (1.35, -0.53), (1.35, 0.53), (0.785, 0.53)))
    return ZoneResult(
        convex_hull(stop_points), convex_hull(slow_points),
        "CONSERVATIVE_FIXED_FALLBACK", 0.0, 0.0, 0.0)


def build_zones(
    linear_x: float,
    angular_z: float,
    command_age_sec: float,
    config: GeometryConfig = GeometryConfig(),
) -> ZoneResult:
    values = (linear_x, angular_z, command_age_sec)
    if not all(math.isfinite(value) for value in values):
        return fallback_zones(config)
    if command_age_sec < 0.0 or command_age_sec > config.command_timeout_sec:
        return fallback_zones(config)
    # Reverse is intentionally outside this forward-only MK-mini milestone.
    if linear_x <= config.zero_speed_epsilon:
        return fallback_zones(config)
    curvature = angular_z / linear_x
    max_curvature = 1.0 / config.minimum_turning_radius
    if not math.isfinite(curvature) or abs(curvature) > max_curvature + 1.0e-9:
        return fallback_zones(config)
    stop_distance = min(
        config.stop_max_distance,
        config.stop_base_distance + linear_x * config.stop_horizon_sec)
    slow_distance = min(
        config.slow_max_distance,
        config.slow_base_distance + linear_x * config.slow_horizon_sec)
    stop_distance = max(config.stop_base_distance, stop_distance)
    slow_distance = max(stop_distance, slow_distance)
    return ZoneResult(
        swept_polygon(config, stop_distance, curvature, config.stop_extra_margin),
        swept_polygon(config, slow_distance, curvature, config.slow_extra_margin),
        "MOTION_AWARE", curvature, stop_distance, slow_distance)


def point_in_polygon(point: Point, polygon: Sequence[Point], tolerance: float = 1.0e-9) -> bool:
    """Boundary-inclusive test for a convex counter-clockwise polygon."""
    for index, current in enumerate(polygon):
        following = polygon[(index + 1) % len(polygon)]
        if _cross(current, following, point) < -tolerance:
            return False
    return True


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Point32, PolygonStamped, Twist
    from rclpy.node import Node

    class MotionAwareGeometryPublisher(Node):
        def __init__(self) -> None:
            super().__init__("motion_aware_collision_geometry")
            defaults = GeometryConfig()
            entries = {
                name: getattr(defaults, name) for name in defaults.__dataclass_fields__
            }
            for name, default in entries.items():
                self.declare_parameter(name, default)
            self.config = GeometryConfig(**{
                name: self.get_parameter(name).value for name in entries
            })
            self.declare_parameter("input_cmd_topic", "/cmd_vel_nav")
            self.declare_parameter("stop_polygon_topic", "/collision_monitor/experimental_stop")
            self.declare_parameter("slow_polygon_topic", "/collision_monitor/experimental_slow")
            input_topic = self.get_parameter("input_cmd_topic").value
            self.stop_publisher = self.create_publisher(
                PolygonStamped, self.get_parameter("stop_polygon_topic").value, 10)
            self.slow_publisher = self.create_publisher(
                PolygonStamped, self.get_parameter("slow_polygon_topic").value, 10)
            self.last_command = None
            self.last_command_time = None
            self.create_subscription(Twist, input_topic, self.command_callback, 10)
            self.create_timer(0.05, self.publish_geometry)
            self.get_logger().warning(
                "R23 SHADOW ONLY: publishes geometry; fixed Collision Monitor remains authoritative")

        def command_callback(self, message: Twist) -> None:
            self.last_command = message
            self.last_command_time = self.get_clock().now()

        @staticmethod
        def message(points: Sequence[Point], stamp) -> PolygonStamped:
            message = PolygonStamped()
            message.header.frame_id = "base_footprint"
            message.header.stamp = stamp
            for x, y in points:
                point = Point32()
                point.x, point.y, point.z = float(x), float(y), 0.0
                message.polygon.points.append(point)
            return message

        def publish_geometry(self) -> None:
            now = self.get_clock().now()
            if self.last_command is None or self.last_command_time is None:
                result = fallback_zones(self.config)
            else:
                age = (now - self.last_command_time).nanoseconds * 1.0e-9
                result = build_zones(
                    self.last_command.linear.x, self.last_command.angular.z,
                    age, self.config)
            stamp = now.to_msg()
            self.stop_publisher.publish(self.message(result.stop, stamp))
            self.slow_publisher.publish(self.message(result.slow, stamp))

    rclpy.init()
    node = MotionAwareGeometryPublisher()
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
