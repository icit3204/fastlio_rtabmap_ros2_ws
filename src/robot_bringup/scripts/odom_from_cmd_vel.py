#!/usr/bin/python3.10
"""Fake odometry from cmd_vel — simulates a robot moving without hardware.

Subscribes to /cmd_vel_nav, integrates with a diff-drive model,
publishes /Odometry and odom→base_footprint TF.
Also accepts /initialpose to set starting position.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster


class OdomFromCmdVel(Node):
    def __init__(self):
        super().__init__('odom_from_cmd_vel')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.yaw = self.get_parameter('initial_yaw').value

        self.vx = 0.0
        self.wz = 0.0
        self.last_cmd = self.get_clock().now()
        self.last_update = self.get_clock().now()
        self.last_motion_log = self.get_clock().now()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_cb, 10)
        self.init_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                  '/initialpose', self.init_cb, 10)

        # Update timer (50 Hz)
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info(f'Initial pose: ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f})')
        self.get_logger().info('Waiting for /cmd_vel_nav...')

    def cmd_cb(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        self.last_cmd = self.get_clock().now()
        if abs(self.vx) > 1e-3 or abs(self.wz) > 1e-3:
            self.get_logger().info(
                f'cmd_vel_nav: vx={self.vx:.3f} m/s, wz={self.wz:.3f} rad/s',
                throttle_duration_sec=2.0)

    def init_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.get_logger().info(f'Initial pose set: ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f})')

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        if dt <= 0:
            self.last_update = now
            return
        self.last_update = now

        # Timeout cmd_vel after 0.5s of no commands
        cmd_age = (now - self.last_cmd).nanoseconds * 1e-9
        if cmd_age > 0.5:
            self.vx = 0.0
            self.wz = 0.0

        # Diff-drive integration
        self.yaw += self.wz * dt
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = t.transform.rotation
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = 0.01
        msg.pose.covariance[21] = 0.01
        msg.pose.covariance[28] = 0.01
        msg.pose.covariance[35] = 0.01
        self.odom_pub.publish(msg)

        if abs(self.vx) > 1e-3 or abs(self.wz) > 1e-3:
            self.get_logger().info(
                f'odom pose: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}',
                throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = OdomFromCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
