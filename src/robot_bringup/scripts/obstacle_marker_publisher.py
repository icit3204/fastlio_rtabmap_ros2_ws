#!/usr/bin/python3.10
"""Publish baked obstacle rectangles as red RViz markers."""
import os

import rclpy
import yaml
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleMarkerPublisher(Node):
    def __init__(self):
        super().__init__('obstacle_marker_publisher')
        self.declare_parameter('input', '/tmp/obstacles.yaml')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)

        self.input_path = self.get_parameter('input').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('publish_rate').value)

        self.pub = self.create_publisher(MarkerArray, '/offline_obstacles', 10)
        self.markers = self.load_markers()
        self.timer = self.create_timer(1.0 / rate, self.publish_markers)

    def load_markers(self):
        if not os.path.exists(self.input_path):
            self.get_logger().warn(f'Obstacle file not found: {self.input_path}')
            return MarkerArray()

        with open(self.input_path, 'r') as f:
            data = yaml.safe_load(f) or {}

        marker_array = MarkerArray()
        for i, obs in enumerate(data.get('obstacles', [])):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.ns = 'offline_obstacles'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(obs['cx'])
            marker.pose.position.y = float(obs['cy'])
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(obs['width'])
            marker.scale.y = float(obs['height'])
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)

        self.get_logger().info(f'Loaded {len(marker_array.markers)} obstacle markers')
        return marker_array

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.pub.publish(self.markers)


def main():
    rclpy.init()
    node = ObstacleMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
