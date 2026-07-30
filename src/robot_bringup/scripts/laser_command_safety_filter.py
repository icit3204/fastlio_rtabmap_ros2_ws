#!/usr/bin/env python3
"""Filter wheelchair control commands with a low 2D-lidar safety stop."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String


class LaserCommandSafetyFilter(Node):
    def __init__(self):
        super().__init__('laser_command_safety_filter')

        self.declare_parameter('input_command_topic', '/wheelchair_control_command_raw')
        self.declare_parameter('output_command_topic', '/wheelchair_control_command')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('avoidance_state_topic', '/laser_avoidance_state')
        self.declare_parameter('enabled', True)
        self.declare_parameter('require_scan', False)
        self.declare_parameter('scan_timeout', 0.7)
        self.declare_parameter('front_sector_deg', 25.0)
        self.declare_parameter('front_channel_half_width', 0.5)
        self.declare_parameter('rear_self_filter_distance', 1.0)
        self.declare_parameter('slowdown_distance', 1.5)
        self.declare_parameter('emergency_stop_distance', 0.65)
        self.declare_parameter('hard_stop_distance', 0.30)
        self.declare_parameter('min_slowdown_scale', 0.45)
        self.declare_parameter('avoidance_turn_radius_max_mm', 20000.0)
        self.declare_parameter('avoidance_max_speed_mm_s', 1400.0)

        self.input_topic = str(self.get_parameter('input_command_topic').value)
        self.output_topic = str(self.get_parameter('output_command_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.avoidance_state_topic = str(self.get_parameter('avoidance_state_topic').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.require_scan = bool(self.get_parameter('require_scan').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.front_sector_deg = max(0.0, min(90.0, float(self.get_parameter('front_sector_deg').value)))
        self.front_channel_half_width = max(0.05, float(self.get_parameter('front_channel_half_width').value))
        self.rear_self_filter_distance = max(0.0, float(self.get_parameter('rear_self_filter_distance').value))
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.emergency_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.hard_stop_distance = float(self.get_parameter('hard_stop_distance').value)
        self.min_scale = float(self.get_parameter('min_slowdown_scale').value)
        self.avoidance_turn_radius_max = float(self.get_parameter('avoidance_turn_radius_max_mm').value)
        self.avoidance_max_speed = float(self.get_parameter('avoidance_max_speed_mm_s').value)

        self.latest_scan_time = 0.0
        self.front_min = math.inf
        self.avoidance_active = False

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.cmd_sub = self.create_subscription(Float32MultiArray, self.input_topic, self.command_callback, 10)
        self.avoidance_state_sub = self.create_subscription(String, self.avoidance_state_topic, self.avoidance_state_callback, 10)
        self.cmd_pub = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        self.state_pub = self.create_publisher(String, '/laser_safety_state', 10)

        self.get_logger().info(
            f'laser command safety filter: {self.input_topic} -> {self.output_topic}, '
            f'scan={self.scan_topic}, enabled={self.enabled}, require_scan={self.require_scan}')

    def scan_callback(self, scan: LaserScan):
        self.latest_scan_time = time.monotonic()
        self.front_min = self._front_channel_min(scan)

    def avoidance_state_callback(self, msg: String):
        self.avoidance_active = msg.data.startswith('AVOIDING_')

    def command_callback(self, msg: Float32MultiArray):
        if not self.enabled:
            self.cmd_pub.publish(msg)
            self._publish_state('DISABLED_PASS_THROUGH')
            return

        if len(msg.data) < 3:
            self.get_logger().warn('invalid command array size < 3', throttle_duration_sec=2.0)
            return

        scan_recent = (time.monotonic() - self.latest_scan_time) <= self.scan_timeout
        if not scan_recent:
            if self.require_scan:
                self.cmd_pub.publish(self._stop_msg())
                self._publish_state('STOP_NO_SCAN')
                self.get_logger().warn('No recent /scan, sending STOP', throttle_duration_sec=2.0)
                return
            self.cmd_pub.publish(msg)
            self._publish_state('NO_SCAN_PASS_THROUGH')
            return

        radius = float(msg.data[0])
        velocity = float(msg.data[1])
        if velocity <= 0.0:
            self.cmd_pub.publish(msg)
            self._publish_state(f'PASS_NON_FORWARD front={self._fmt_range(self.front_min)}')
            return

        if self.avoidance_active:
            if self.front_min <= self.hard_stop_distance:
                self.cmd_pub.publish(self._stop_msg())
                self._publish_state(f'STOP_HARD_AVOIDANCE front={self._fmt_range(self.front_min)}')
                return
            out = Float32MultiArray()
            out.data = list(msg.data)
            out.data[1] = self._limit_abs(float(out.data[1]), self.avoidance_max_speed)
            self.cmd_pub.publish(out)
            self._publish_state(
                f'AVOIDANCE_PASS front={self._fmt_range(self.front_min)} '
                f'radius={radius:.0f} speed={out.data[1]:.0f}')
            return

        if abs(radius) < 1e-3:
            self.cmd_pub.publish(msg)
            self._publish_state(f'PASS_TURN_IN_PLACE front={self._fmt_range(self.front_min)}')
            return

        if self.front_min <= self.emergency_distance:
            if self._can_continue_avoidance(radius):
                out = Float32MultiArray()
                out.data = list(msg.data)
                out.data[1] = self._limit_abs(float(out.data[1]), self.avoidance_max_speed)
                self.cmd_pub.publish(out)
                self._publish_state(
                    f'AVOIDANCE_SLOW_PASS front={self._fmt_range(self.front_min)} '
                    f'radius={radius:.0f} speed={out.data[1]:.0f}')
                return
            self.cmd_pub.publish(self._stop_msg())
            self._publish_state(f'STOP_EMERGENCY front={self._fmt_range(self.front_min)}')
            self.get_logger().warn(
                f'Front obstacle {self.front_min:.2f} m <= {self.emergency_distance:.2f} m, sending STOP',
                throttle_duration_sec=1.0)
            return

        if self.front_min < self.slowdown_distance:
            scale = self._slowdown_scale(self.front_min)
            out = Float32MultiArray()
            out.data = list(msg.data)
            out.data[1] = float(out.data[1]) * scale
            self.cmd_pub.publish(out)
            self._publish_state(f'SLOWDOWN scale={scale:.2f} front={self._fmt_range(self.front_min)}')
            return

        self.cmd_pub.publish(msg)
        self._publish_state(f'PASS front={self._fmt_range(self.front_min)}')

    def _can_continue_avoidance(self, radius: float) -> bool:
        if not self.avoidance_active:
            return False
        if self.front_min <= self.hard_stop_distance:
            return False
        return abs(radius) <= self.avoidance_turn_radius_max

    @staticmethod
    def _limit_abs(value: float, limit: float) -> float:
        if limit <= 0.0:
            return value
        return math.copysign(min(abs(value), limit), value)

    def _slowdown_scale(self, front_min: float) -> float:
        span = max(1e-3, self.slowdown_distance - self.emergency_distance)
        ratio = (front_min - self.emergency_distance) / span
        ratio = max(0.0, min(1.0, ratio))
        return self.min_scale + (1.0 - self.min_scale) * ratio

    @staticmethod
    def _stop_msg() -> Float32MultiArray:
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        return msg

    def _publish_state(self, text: str):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def _sector_min(self, scan: LaserScan, min_deg: float, max_deg: float) -> float:
        min_deg = max(-90.0, min(90.0, min_deg))
        max_deg = max(-90.0, min(90.0, max_deg))
        if min_deg > max_deg:
            min_deg, max_deg = max_deg, min_deg
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        upper_range = scan.range_max if scan.range_max > 0.0 else float('inf')
        best = math.inf
        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value <= max(0.0, scan.range_min) or value >= upper_range:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if self._is_rear_self_point(angle, float(value)):
                continue
            if min_rad <= angle <= max_rad:
                best = min(best, float(value))
        return best

    def _front_channel_min(self, scan: LaserScan) -> float:
        upper_range = scan.range_max if scan.range_max > 0.0 else float('inf')
        best = math.inf
        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            value = float(value)
            if value <= max(0.0, scan.range_min) or value >= upper_range:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            x = value * math.cos(angle)
            y = value * math.sin(angle)
            if x <= 0.0:
                continue
            if abs(y) > self.front_channel_half_width:
                continue
            best = min(best, x)
        return best

    def _is_rear_self_point(self, angle_rad: float, range_m: float) -> bool:
        return abs(angle_rad) > (math.pi * 0.5) and range_m <= self.rear_self_filter_distance

    @staticmethod
    def _fmt_range(value: float) -> str:
        return 'inf' if not math.isfinite(value) else f'{value:.2f}m'


def main():
    rclpy.init()
    node = LaserCommandSafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
