#!/usr/bin/python3
"""Convert sensor_msgs/LaserScan to sensor_msgs/PointCloud2.

Used when a 2D LiDAR (e.g. YDLIDAR T-mini Plus) needs to provide
PointCloud2 data to costmaps and/or RTAB-Map in place of a 3D LiDAR.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')

        self.declare_parameter('target_topic', '/cloud_registered_body')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 12.0)

        target_topic = self.get_parameter('target_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        # <原版> QoS默认=RELIABLE
        # <修改 version3 YDLIDAR发布BEST_EFFORT, 此处用BestEffort匹配>
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, scan_topic, self.callback, qos)
        self.pub = self.create_publisher(PointCloud2, target_topic, 10)
        self.get_logger().info(f'Converting {scan_topic} -> {target_topic}')

    def callback(self, scan: LaserScan):
        n = len(scan.ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=np.float64)

        valid = (
            (ranges > self.min_range)
            & (ranges < self.max_range)
            & np.isfinite(ranges)
        )

        angles_valid = angles[valid]
        ranges_valid = ranges[valid]

        x = (ranges_valid * np.cos(angles_valid)).astype(np.float32)
        y = (ranges_valid * np.sin(angles_valid)).astype(np.float32)
        z = np.zeros(len(x), dtype=np.float32)

        points = np.column_stack([x, y, z]).tobytes()

        header = Header()
        header.stamp = scan.header.stamp
        header.frame_id = scan.header.frame_id

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(x)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
