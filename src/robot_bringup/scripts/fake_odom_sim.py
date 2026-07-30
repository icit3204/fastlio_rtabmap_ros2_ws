#!/usr/bin/python3
"""Fake odometry publisher for simulation without a real robot.

Publishes /fake_odom and odom->base_footprint TF so the robot
"moves" in RViz. The YDLIDAR provides real obstacle data.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


class FakeOdomSim(Node):
    def __init__(self):
        super().__init__('fake_odom_sim')
        self.declare_parameter('vx', 0.15)   # linear velocity m/s
        self.declare_parameter('wz', 0.1)    # angular velocity rad/s
        self.declare_parameter('rate', 10.0)  # Hz

        self.vx = self.get_parameter('vx').value
        self.wz = self.get_parameter('wz').value
        self.rate = self.get_parameter('rate').value

        self.pub = self.create_publisher(Odometry, '/fake_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.rate, self.publish)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info(f'Fake odom: vx={self.vx}m/s, wz={self.wz}rad/s, rate={self.rate}Hz')

    def publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        self.yaw += self.wz * dt
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        q = Quaternion()
        q.z = math.sin(self.yaw / 2.0)
        q.w = math.cos(self.yaw / 2.0)

        # Publish odom -> base_footprint TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish odometry
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = 0.01
        msg.pose.covariance[21] = 0.01
        msg.pose.covariance[28] = 0.01
        msg.pose.covariance[35] = 0.01
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeOdomSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
