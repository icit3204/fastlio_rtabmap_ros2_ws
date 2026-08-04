"""Deterministic synthetic Collision Monitor observations for Phase 4 P4-B."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Iterable

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2, PointField


MODES = {"CLEAR", "SLOW", "STOP", "SILENT"}


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


CLEAR_POINTS = (
    Point2D(1.50, -0.75),
    Point2D(1.50, 0.75),
    Point2D(1.80, -0.85),
    Point2D(1.80, 0.85),
)
SLOW_POINTS = (
    Point2D(0.62, -0.42),
    Point2D(0.66, -0.30),
    Point2D(0.70, 0.00),
    Point2D(0.74, 0.30),
    Point2D(0.78, 0.42),
)
STOP_POINTS = (
    Point2D(0.18, -0.22),
    Point2D(0.22, -0.10),
    Point2D(0.26, 0.00),
    Point2D(0.30, 0.10),
    Point2D(0.34, 0.22),
)


def points_for_mode(mode: str) -> tuple[Point2D, ...]:
    normalized = mode.upper()
    if normalized == "CLEAR":
        return CLEAR_POINTS
    if normalized == "SLOW":
        return SLOW_POINTS
    if normalized == "STOP":
        return STOP_POINTS
    if normalized == "SILENT":
        return ()
    raise ValueError(f"Unsupported synthetic obstacle mode: {mode}")


def _empty_ranges(count: int, value: float) -> list[float]:
    return [value for _ in range(count)]


def points_to_scan(points: Iterable[Point2D], frame_id: str, stamp) -> LaserScan:
    angle_min = -math.pi / 2.0
    angle_max = math.pi / 2.0
    angle_increment = math.radians(1.0)
    count = int(round((angle_max - angle_min) / angle_increment)) + 1

    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = frame_id
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.time_increment = 0.0
    scan.scan_time = 0.05
    scan.range_min = 0.02
    scan.range_max = 5.0
    scan.ranges = _empty_ranges(count, scan.range_max)
    scan.intensities = _empty_ranges(count, 1.0)

    for point in points:
        radius = math.hypot(point.x, point.y)
        angle = math.atan2(point.y, point.x)
        index = int(round((angle - angle_min) / angle_increment))
        if 0 <= index < count and scan.range_min <= radius <= scan.range_max:
            scan.ranges[index] = min(scan.ranges[index], radius)

    return scan


def points_to_cloud(points: Iterable[Point2D], frame_id: str, stamp) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    data = bytearray()
    point_count = 0
    for point in points:
        data.extend(struct.pack("<fff", float(point.x), float(point.y), 0.0))
        point_count += 1
    cloud.width = point_count
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = bytes(data)
    cloud.is_dense = True
    return cloud


class Phase4P4BSyntheticObstacles(Node):
    """Publish deterministic synthetic scan and pointcloud observations."""

    def __init__(self) -> None:
        super().__init__("phase4_p4b_synthetic_obstacles")
        self.declare_parameter("mode", "CLEAR")
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("publish_pointcloud", False)

        self._mode = str(self.get_parameter("mode").value).upper()
        if self._mode not in MODES:
            raise ValueError(f"mode must be one of {sorted(MODES)}")

        self._frame_id = str(self.get_parameter("frame_id").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz < 20.0:
            raise ValueError("publish_rate_hz must be at least 20.0")

        self._publish_scan = bool(self.get_parameter("publish_scan").value)
        self._publish_pointcloud = bool(self.get_parameter("publish_pointcloud").value)
        if not self._publish_scan and not self._publish_pointcloud:
            raise ValueError("At least one synthetic observation output must be enabled")

        self._scan_pub = self.create_publisher(LaserScan, "/phase4/synthetic_scan", 10)
        self._cloud_pub = self.create_publisher(PointCloud2, "/phase4/synthetic_points", 10)
        self.add_on_set_parameters_callback(self._set_parameters_cb)
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

    def _set_parameters_cb(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "mode":
                candidate = str(param.value).upper()
                if candidate not in MODES:
                    return SetParametersResult(
                        successful=False,
                        reason=f"mode must be one of {sorted(MODES)}",
                    )
                self._mode = candidate
        return SetParametersResult(successful=True)

    def _timer_cb(self) -> None:
        mode = self._mode
        if mode == "SILENT":
            return
        points = points_for_mode(mode)
        stamp = self.get_clock().now().to_msg()
        if self._publish_scan:
            self._scan_pub.publish(points_to_scan(points, self._frame_id, stamp))
        if self._publish_pointcloud:
            self._cloud_pub.publish(points_to_cloud(points, self._frame_id, stamp))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Phase4P4BSyntheticObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
