#!/usr/bin/python3.10
"""Publish fake identity odometry to keep RTAB-Map processing loop alive in offline mode.

Without any sensor data, RTAB-Map's approximate-time synchronizer never fires,
so the main processing loop never runs and /map is never published.
This node publishes fake odometry at 2 Hz to trigger the processing pipeline.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.timer = self.create_timer(0.5, self.publish_odom)
        self.get_logger().info('Publishing fake odometry at 2 Hz for offline map viewing')

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.0
        # Minimal covariance for RTAB-Map's odom integration
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = 0.01
        msg.pose.covariance[21] = 0.01
        msg.pose.covariance[28] = 0.01
        msg.pose.covariance[35] = 0.01
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
