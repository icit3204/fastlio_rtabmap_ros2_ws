#!/usr/bin/python3
"""Publish a lookahead/preview point from the global plan for RViz visualization.

Subscribes to /plan, walks along the path to accumulate distance, and publishes
the first point beyond the configured lookahead distance as a PointStamped marker.
Add /lookahead_point to RViz as a red dot to see where the robot is heading.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path


class PreviewPointPublisher(Node):
    def __init__(self):
        super().__init__('preview_point_publisher')
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('plan_topic', '/plan')

        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        plan_topic = self.get_parameter('plan_topic').value

        self.pub = self.create_publisher(PointStamped, '/lookahead_point', 10)
        self.sub = self.create_subscription(Path, plan_topic, self.plan_cb, 10)

        self.get_logger().info(
            f'Publishing /lookahead_point at {self.lookahead_dist}m '
            f'from plan topic "{plan_topic}"')

    def plan_cb(self, msg: Path):
        if not msg.poses:
            return

        accumulated = 0.0
        prev = None
        for ps in msg.poses:
            pt = ps.pose.position
            if prev is None:
                prev = pt
                continue
            accumulated += math.hypot(pt.x - prev.x, pt.y - prev.y)
            if accumulated >= self.lookahead_dist:
                out = PointStamped()
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = msg.header.frame_id
                out.point = pt
                self.pub.publish(out)
                return
            prev = pt

        # Path shorter than lookahead → use endpoint
        if msg.poses:
            last = msg.poses[-1].pose.position
            out = PointStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = msg.header.frame_id
            out.point = last
            self.pub.publish(out)


def main():
    rclpy.init()
    node = PreviewPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
