#!/usr/bin/python3.10
"""Publish a simple robot body marker for RViz when no URDF is available."""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class RobotMarkerPublisher(Node):
    def __init__(self):
        super().__init__('robot_marker_publisher')
        self.declare_parameter('frame_id', 'base_footprint')
        self.pub = self.create_publisher(Marker, '/offline_robot_marker', 10)
        self.timer = self.create_timer(0.2, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'offline_robot'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.70
        marker.scale.y = 0.45
        marker.scale.z = 0.30
        marker.color.r = 0.0
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 0.95
        self.pub.publish(marker)


def main():
    rclpy.init()
    node = RobotMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
