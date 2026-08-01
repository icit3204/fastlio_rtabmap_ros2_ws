"""Phase 2 isolated fake base node.

The node subscribes only to a mock Phase 2 velocity topic and publishes
odometry plus odom->base_footprint TF. It has no hardware, file, CAN, UDP, or
legacy-controller access.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from parking_robot_bringup.phase2_fake_base_math import (
    Pose2D,
    Twist2D,
    command_for_integration,
    integrate_pose,
    is_finite_twist,
    quaternion_from_yaw,
    reset_pose,
    yaw_from_quaternion,
)


class Phase2FakeBase(Node):
    """Deterministic planar fake base for isolated Nav2 validation."""

    def __init__(self) -> None:
        super().__init__("phase2_fake_base")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel_phase2_mock")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("max_integration_dt_sec", 0.1)
        self.declare_parameter("initial_x", 5.425)
        self.declare_parameter("initial_y", -53.725)
        self.declare_parameter("initial_yaw", 0.0)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self._max_dt_sec = float(self.get_parameter("max_integration_dt_sec").value)

        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._lock = threading.Lock()
        self._pose = reset_pose(
            float(self.get_parameter("initial_x").value),
            float(self.get_parameter("initial_y").value),
            float(self.get_parameter("initial_yaw").value),
        )
        self._active_twist: Twist2D | None = None
        self._last_command_steady: float | None = None
        self._last_integration_steady = time.monotonic()
        self._shutting_down = False

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cmd_sub = self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_cb, 10)
        self._initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10
        )
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._timer_cb)

    def _cmd_cb(self, msg: Twist) -> None:
        twist = Twist2D(float(msg.linear.x), float(msg.angular.z))
        if not is_finite_twist(twist):
            self.get_logger().warn("Rejected non-finite cmd_vel command")
            return
        with self._lock:
            self._active_twist = twist
            self._last_command_steady = time.monotonic()

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        with self._lock:
            self._pose = reset_pose(
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw,
            )
            self._active_twist = None
            self._last_command_steady = None
            self._last_integration_steady = time.monotonic()

    def _ros_context_is_valid(self) -> bool:
        return self.context.ok()

    def stop_publication(self) -> None:
        """Stop future timer publication before node teardown."""
        self._shutting_down = True
        timer = getattr(self, "_timer", None)
        if timer is not None:
            timer.cancel()

    def _timer_cb(self) -> None:
        if self._shutting_down or not self._ros_context_is_valid():
            return

        now_steady = time.monotonic()
        with self._lock:
            dt = now_steady - self._last_integration_steady
            twist = command_for_integration(
                self._active_twist,
                now_steady,
                self._last_command_steady,
                self._cmd_timeout_sec,
            )
            self._pose = integrate_pose(self._pose, twist, dt, self._max_dt_sec)
            self._last_integration_steady = now_steady
            pose = Pose2D(self._pose.x, self._pose.y, self._pose.yaw)
            active_twist = twist

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = active_twist.vx
        odom.twist.twist.angular.z = active_twist.wz

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = pose.x
        tf.transform.translation.y = pose.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        if self._shutting_down or not self._ros_context_is_valid():
            return

        try:
            self._odom_pub.publish(odom)
            self._tf_broadcaster.sendTransform(tf)
        except RCLError:
            if self._ros_context_is_valid():
                raise


def _shutdown_context_once() -> None:
    if rclpy.ok():
        rclpy.shutdown()


def _spin_until_shutdown(node: Phase2FakeBase, spin_fn=rclpy.spin) -> None:
    try:
        spin_fn(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if node.context.ok():
            raise


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Phase2FakeBase()
    try:
        _spin_until_shutdown(node)
    finally:
        node.stop_publication()
        node.destroy_node()
        _shutdown_context_once()


if __name__ == "__main__":
    main()
