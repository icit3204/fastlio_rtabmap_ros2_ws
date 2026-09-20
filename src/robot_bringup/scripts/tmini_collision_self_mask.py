#!/usr/bin/env python3
"""Experimental Collision Monitor-only T-mini rigid-body self mask.

The raw LaserScan remains untouched. This node republishes a dedicated shadow
observer scan in which returns geometrically inside the measured MK-mini body
are set to +Inf. It must not be used by Nav2 costmaps or the fixed-qualified
Collision Monitor.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


@dataclass(frozen=True)
class SelfMaskConfig:
    footprint_x_min: float = -0.145
    footprint_x_max: float = 0.755
    footprint_y_min: float = -0.300
    footprint_y_max: float = 0.300
    sensor_x: float = 0.703
    sensor_y: float = 0.0
    sensor_yaw: float = 0.0


def point_in_physical_body(x: float, y: float, config: SelfMaskConfig) -> bool:
    """True only inside/on the unpadded measured physical chassis envelope."""
    return (config.footprint_x_min <= x <= config.footprint_x_max and
            config.footprint_y_min <= y <= config.footprint_y_max)


def point_for_beam(angle: float, distance: float, config: SelfMaskConfig) -> Tuple[float, float]:
    bearing = angle + config.sensor_yaw
    return (config.sensor_x + distance * math.cos(bearing),
            config.sensor_y + distance * math.sin(bearing))


def mask_ranges(
    ranges: Iterable[float], angle_min: float, angle_increment: float,
    range_min: float, range_max: float, config: SelfMaskConfig,
) -> Tuple[Tuple[float, ...], int]:
    output = []
    removed = 0
    for index, value in enumerate(ranges):
        distance = float(value)
        valid = math.isfinite(distance) and range_min <= distance <= range_max
        if valid:
            x, y = point_for_beam(angle_min + index * angle_increment, distance, config)
            if point_in_physical_body(x, y, config):
                output.append(math.inf)
                removed += 1
                continue
        output.append(distance)
    return tuple(output), removed


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class TminiCollisionSelfMask(Node):
        def __init__(self) -> None:
            super().__init__("tmini_collision_self_mask")
            defaults = SelfMaskConfig()
            values = {}
            for name in defaults.__dataclass_fields__:
                self.declare_parameter(name, getattr(defaults, name))
                values[name] = float(self.get_parameter(name).value)
            self.config = SelfMaskConfig(**values)
            self.declare_parameter("input_topic", "/scan")
            self.declare_parameter("output_topic", "/scan_collision_experimental")
            self.declare_parameter("expected_frame", "laser_frame")
            self.input_topic = str(self.get_parameter("input_topic").value)
            self.output_topic = str(self.get_parameter("output_topic").value)
            self.expected_frame = str(self.get_parameter("expected_frame").value)
            self.publisher = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)
            self.subscription = self.create_subscription(
                LaserScan, self.input_topic, self.callback, qos_profile_sensor_data)
            self.get_logger().warning(
                f"R23-R2 SHADOW ONLY T-mini physical-body mask "
                f"{self.input_topic} -> {self.output_topic}; raw scan unchanged")

        def callback(self, message: LaserScan) -> None:
            # Fail closed for the experimental observer branch: an unexpected
            # frame is not republished as though it had been safely masked.
            if message.header.frame_id != self.expected_frame:
                self.get_logger().error(
                    f"T-mini frame mismatch expected={self.expected_frame} "
                    f"got={message.header.frame_id}; suppressing shadow output")
                return
            filtered, _ = mask_ranges(
                message.ranges, message.angle_min, message.angle_increment,
                message.range_min, message.range_max, self.config)
            output = LaserScan()
            output.header = message.header
            output.angle_min = message.angle_min
            output.angle_max = message.angle_max
            output.angle_increment = message.angle_increment
            output.time_increment = message.time_increment
            output.scan_time = message.scan_time
            output.range_min = message.range_min
            output.range_max = message.range_max
            output.ranges = list(filtered)
            output.intensities = message.intensities
            self.publisher.publish(output)

    rclpy.init()
    node = TminiCollisionSelfMask()
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
