#!/usr/bin/python3.10
"""Create dynamic obstacles from RViz Publish Point clicks."""
import struct

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray


class ClickedObstaclePublisher(Node):
    def __init__(self):
        super().__init__('clicked_obstacle_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('obstacle_width', 2.2)
        self.declare_parameter('obstacle_height', 2.2)
        self.declare_parameter('obstacle_z', 0.5)
        self.declare_parameter('publish_rate', 5.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.width = float(self.get_parameter('obstacle_width').value)
        self.height = float(self.get_parameter('obstacle_height').value)
        self.z = float(self.get_parameter('obstacle_z').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.markers = []
        self.marker_pub = self.create_publisher(MarkerArray, '/clicked_obstacles', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/clicked_obstacle_points', 10)
        self.sub = self.create_subscription(PointStamped, '/clicked_point', self.clicked_cb, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info(
            'Click RViz "Publish Point" to add a dynamic obstacle')

    def clicked_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f'Ignoring clicked point in frame {msg.header.frame_id}; expected {self.frame_id}')
            return

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = 'clicked_obstacles'
        marker.id = len(self.markers)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(msg.point.x)
        marker.pose.position.y = float(msg.point.y)
        marker.pose.position.z = self.z / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.width
        marker.scale.y = self.height
        marker.scale.z = self.z
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.85
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.markers.append(marker)
        self.get_logger().info(
            f'Added clicked obstacle #{marker.id}: '
            f'({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f}), '
            f'size={self.width:.2f}x{self.height:.2f}m')

    def build_cloud(self, stamp):
        points = []
        step = 0.05
        for marker in self.markers:
            cx = marker.pose.position.x
            cy = marker.pose.position.y
            half_x = marker.scale.x / 2.0
            half_y = marker.scale.y / 2.0
            nx = max(1, int(marker.scale.x / step))
            ny = max(1, int(marker.scale.y / step))
            for ix in range(nx + 1):
                x = cx - half_x + marker.scale.x * ix / nx
                for iy in range(ny + 1):
                    y = cy - half_y + marker.scale.y * iy / ny
                    points.append((x, y, marker.pose.position.z))

        cloud = PointCloud2()
        cloud.header.frame_id = self.frame_id
        cloud.header.stamp = stamp
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        return cloud

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        for marker in self.markers:
            marker.header.stamp = stamp
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.cloud_pub.publish(self.build_cloud(stamp))


def main():
    rclpy.init()
    node = ClickedObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
