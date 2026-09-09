"""ROS producer for the Phase-5 localization validity safety permission."""

from __future__ import annotations

import time

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from vehicle_cmd_safety.localization_validity_core import (
    LocalizationValidityConfig,
    LocalizationValidityCore,
    Pose3,
)


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _pose3_from_pose(pose: Pose) -> Pose3:
    return Pose3(pose.position.x, pose.position.y, pose.position.z,
                 pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)


def _pose3_from_transform(msg: TransformStamped) -> Pose3:
    t, q = msg.transform.translation, msg.transform.rotation
    return Pose3(t.x, t.y, t.z, q.x, q.y, q.z, q.w)


class LocalizationValidityMonitor(Node):
    def __init__(self) -> None:
        super().__init__("localization_validity_monitor")
        self._declare_parameters()
        self._core = LocalizationValidityCore(self._config())
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._sequence = 0
        self._last_transform_key = None
        self._last_state = None

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._valid_pub = self.create_publisher(Bool, "/system/localization_valid", output_qos)
        self._reason_pub = self.create_publisher(String, "/system/localization_reason", output_qos)
        self._pose_age_pub = self.create_publisher(Float64, "/system/pose_age", output_qos)
        self._tf_age_pub = self.create_publisher(Float64, "/system/tf_age", output_qos)
        self._jump_pub = self.create_publisher(Bool, "/system/pose_jump_detected", output_qos)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, qos_profile_sensor_data
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._timer = self.create_timer(1.0 / rate, self._timer_cb, clock=self._steady_clock)

    def _declare_parameters(self) -> None:
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_freshness_sec", 0.50)
        self.declare_parameter("tf_freshness_sec", 0.50)
        self.declare_parameter("translation_jump_threshold_m", 1.0)
        self.declare_parameter("rotation_jump_threshold_rad", 0.7853981633974483)
        self.declare_parameter("quaternion_norm_tolerance", 1.0e-3)
        self.declare_parameter("stability_duration_sec", 1.0)
        self.declare_parameter("stability_min_observations", 3)
        self.declare_parameter("future_stamp_tolerance_sec", 0.05)
        self.declare_parameter("publish_rate_hz", 20.0)

    def _config(self) -> LocalizationValidityConfig:
        return LocalizationValidityConfig(
            odom_freshness_sec=float(self.get_parameter("odom_freshness_sec").value),
            tf_freshness_sec=float(self.get_parameter("tf_freshness_sec").value),
            translation_jump_threshold_m=float(self.get_parameter("translation_jump_threshold_m").value),
            rotation_jump_threshold_rad=float(self.get_parameter("rotation_jump_threshold_rad").value),
            quaternion_norm_tolerance=float(self.get_parameter("quaternion_norm_tolerance").value),
            stability_duration_sec=float(self.get_parameter("stability_duration_sec").value),
            stability_min_observations=int(self.get_parameter("stability_min_observations").value),
            future_stamp_tolerance_sec=float(self.get_parameter("future_stamp_tolerance_sec").value),
        )

    @staticmethod
    def _steady_now() -> float:
        return time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        self._core.set_odometry(_pose3_from_pose(msg.pose.pose), self._steady_now(), _stamp_sec(msg.header.stamp))

    def _lookup(self, target: str, source: str):
        return self._tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.0))

    def _observe_tf(self, now: float) -> None:
        try:
            map_odom = self._lookup(self._map_frame, self._odom_frame)
            self._core.set_map_odom(now, _stamp_sec(map_odom.header.stamp))
        except TransformException:
            self._core.set_map_odom_unavailable()
        try:
            complete = self._lookup(self._map_frame, self._base_frame)
            key = (complete.header.stamp.sec, complete.header.stamp.nanosec)
            if key != self._last_transform_key:
                self._sequence += 1
                self._last_transform_key = key
            self._core.set_complete_pose(
                _pose3_from_transform(complete), now, _stamp_sec(complete.header.stamp), self._sequence
            )
        except TransformException:
            self._core.set_complete_pose_unavailable()

    def _timer_cb(self) -> None:
        steady_now = self._steady_now()
        self._observe_tf(steady_now)
        status = self._core.tick(steady_now, self.get_clock().now().nanoseconds * 1.0e-9)
        self._valid_pub.publish(Bool(data=status.valid))
        self._reason_pub.publish(String(data=status.reason))
        self._pose_age_pub.publish(Float64(data=status.pose_age_sec))
        self._tf_age_pub.publish(Float64(data=status.tf_age_sec))
        self._jump_pub.publish(Bool(data=status.pose_jump_detected))
        state = (status.valid, status.reason)
        if state != self._last_state:
            self._log_transition(status)
            self._last_state = state

    def _log_transition(self, status) -> None:
        message = f"localization validity transition: valid={status.valid} reason={status.reason}"
        if status.valid:
            # Keep this call site permanently INFO for rclpy's caller context.
            self.get_logger().info(message)
        else:
            # Keep this call site permanently WARN for rclpy's caller context.
            self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationValidityMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
