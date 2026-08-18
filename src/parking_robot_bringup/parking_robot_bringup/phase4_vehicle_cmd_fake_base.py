"""Phase-4-only generic fake base downstream of /vehicle_cmd_safe."""
from __future__ import annotations
import math
import threading
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from parking_robot_bringup.phase2_fake_base_math import Pose2D, integrate_pose, quaternion_from_yaw, reset_pose
from parking_robot_bringup.phase4_fake_base_core import (
    FakeBaseConfig, TwistValues, evaluate_command, validate_config, validate_initial_pose,
    validate_input_topic,
)

class Phase4VehicleCmdFakeBase(Node):
    """Steady-time planar simulator; it never owns map->odom."""
    def __init__(self) -> None:
        super().__init__("phase4_vehicle_cmd_fake_base")
        self._declare_parameters()
        self._load_parameters()  # Frozen input is checked before endpoints exist.
        self._integration_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._authority_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._receipt_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._lock = threading.Lock()
        self._pose = reset_pose(self._initial_x, self._initial_y, self._initial_yaw)
        self._latest_values = None
        self._latest_frame = ""
        self._last_receipt_ns = None
        self._input_publishers = 0
        self._last_integration_ns = self._integration_clock.now().nanoseconds
        self._shutting_down = False
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, self._diagnostics_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cmd_sub = self.create_subscription(TwistStamped, self._input_topic, self._command_cb, 10)
        self._initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10)
        self._publication_timer = self.create_timer(
            1.0 / self._publish_rate_hz, self._publication_cb, clock=self._integration_clock)
        self._authority_timer = self.create_timer(
            1.0 / self._authority_poll_hz, self._authority_cb, clock=self._authority_clock)
        self._authority_cb()

    def _declare_parameters(self) -> None:
        defaults = {"input_topic": "/vehicle_cmd_safe", "odom_topic": "/Odometry",
            "diagnostics_topic": "/phase4_fake_base/diagnostics",
            "expected_input_frame": "base_footprint", "odom_frame": "odom",
            "base_frame": "base_footprint", "publish_rate_hz": 50.0,
            "command_timeout_sec": 0.50, "max_integration_dt_sec": 0.10,
            "unsupported_axis_epsilon": 1.0e-6, "authority_poll_hz": 2.0,
            "initial_x": 5.425, "initial_y": -53.725, "initial_yaw": 0.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        self._input_topic = str(self.get_parameter("input_topic").value)
        validate_input_topic(self._input_topic)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._max_dt_sec = float(self.get_parameter("max_integration_dt_sec").value)
        self._authority_poll_hz = float(self.get_parameter("authority_poll_hz").value)
        self._config = FakeBaseConfig(str(self.get_parameter("expected_input_frame").value),
            float(self.get_parameter("command_timeout_sec").value),
            float(self.get_parameter("unsupported_axis_epsilon").value))
        validate_config(self._config)
        if not all(math.isfinite(v) and v > 0.0 for v in
                   (self._publish_rate_hz, self._max_dt_sec, self._authority_poll_hz)):
            raise ValueError("rates and max integration dt must be positive and finite")
        if self._odom_frame != "odom" or self._base_frame != "base_footprint":
            raise ValueError("output frames are frozen to odom and base_footprint")
        self._initial_x = float(self.get_parameter("initial_x").value)
        self._initial_y = float(self.get_parameter("initial_y").value)
        self._initial_yaw = float(self.get_parameter("initial_yaw").value)

    def _command_cb(self, msg: TwistStamped) -> None:
        t = msg.twist
        values = TwistValues(float(t.linear.x), float(t.linear.y), float(t.linear.z),
                             float(t.angular.x), float(t.angular.y), float(t.angular.z))
        with self._lock:
            self._latest_values = values
            self._latest_frame = msg.header.frame_id
            self._last_receipt_ns = self._receipt_clock.now().nanoseconds

    def _authority_cb(self) -> None:
        count = len(self.get_publishers_info_by_topic(self._input_topic))
        with self._lock:
            self._input_publishers = count

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            x, y, yaw = validate_initial_pose((float(p.x), float(p.y), float(p.z)),
                (float(q.x), float(q.y), float(q.z), float(q.w)), msg.header.frame_id)
        except ValueError as exc:
            self.get_logger().warn(f"Rejected initial pose: {exc}")
            return
        with self._lock:
            self._pose = reset_pose(x, y, yaw)
            self._latest_values, self._latest_frame, self._last_receipt_ns = None, "", None
            self._last_integration_ns = self._integration_clock.now().nanoseconds

    def _publication_cb(self) -> None:
        if self._shutting_down or not self.context.ok():
            return
        now_ns = self._integration_clock.now().nanoseconds
        with self._lock:
            age = None if self._last_receipt_ns is None else (now_ns - self._last_receipt_ns) / 1e9
            result = evaluate_command(self._latest_values, self._latest_frame,
                                      self._input_publishers, age, self._config)
            dt = (now_ns - self._last_integration_ns) / 1e9
            self._pose = integrate_pose(self._pose, result.applied, dt, self._max_dt_sec)
            self._last_integration_ns = now_ns
            pose = Pose2D(self._pose.x, self._pose.y, self._pose.yaw)
            values, publishers = self._latest_values or TwistValues(), self._input_publishers
        self._publish_outputs(pose, result)
        self._publish_diagnostics(pose, result, values, publishers)

    def _publish_outputs(self, pose, result) -> None:
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = pose.x, pose.y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = result.applied.vx, result.applied.wz
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, self._odom_frame, self._base_frame
        tf.transform.translation.x, tf.transform.translation.y = pose.x, pose.y
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        try:
            self._odom_pub.publish(odom)
            self._tf_broadcaster.sendTransform(tf)
        except RCLError:
            if self.context.ok():
                raise

    def _publish_diagnostics(self, pose, result, values, publishers) -> None:
        status = DiagnosticStatus(name="phase4_vehicle_cmd_fake_base",
            hardware_id="generic_fake_motion_only",
            level=DiagnosticStatus.OK if result.valid else DiagnosticStatus.WARN,
            message=result.condition.value)
        fields = {"condition": result.condition.value, "reason": result.reason,
            "command_age_sec": result.details["command_age_sec"],
            "command_publisher_count": publishers, "expected_frame": self._config.expected_frame,
            "observed_frame": result.details["observed_frame"],
            "received_linear_x": values.linear_x, "received_angular_z": values.angular_z,
            "applied_linear_x": result.applied.vx, "applied_angular_z": result.applied.wz,
            "pose_x": pose.x, "pose_y": pose.y, "pose_yaw": pose.yaw,
            "output_rate_hz": self._publish_rate_hz,
            "command_timeout_sec": self._config.command_timeout_sec, "steady_clock": True}
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in fields.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diag_pub.publish(array)

    def stop_publication(self) -> None:
        self._shutting_down = True
        for name in ("_publication_timer", "_authority_timer"):
            timer = getattr(self, name, None)
            if timer is not None:
                timer.cancel()

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Phase4VehicleCmdFakeBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_publication()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
