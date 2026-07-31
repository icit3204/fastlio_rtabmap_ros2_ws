#!/usr/bin/python3.10
"""Publish offline navigation trajectory as nav_msgs/Path for RViz display."""
import sqlite3
import struct
import math
import os
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.declare_parameter(
            'database_path',
            os.environ.get('PARKING_ROBOT_OFFLINE_PATH_DATABASE', ''),
        )
        self.declare_parameter('path_yaml', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)  # Hz, just once is enough but periodic for late subscribers

        self.pub = self.create_publisher(Path, '/mapping_path', 10)
        self.timer = self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_path)

        self.path_msg = None
        self.load_path()

    def load_path(self):
        db_path = self.get_parameter('database_path').value
        path_yaml = self.get_parameter('path_yaml').value
        frame_id = self.get_parameter('frame_id').value

        poses = []
        if path_yaml:
            if not os.path.isfile(path_yaml):
                self.get_logger().error(
                    f'path_yaml is not a readable file: {path_yaml}'
                )
                return
            if not path_yaml.lower().endswith(('.yaml', '.yml')):
                self.get_logger().error(
                    f'path_yaml must be a YAML file (.yaml/.yml): {path_yaml}'
                )
                return
            try:
                with open(path_yaml, 'r') as f:
                    data = yaml.safe_load(f)
                for wp in data.get('waypoints', []):
                    poses.append((float(wp['x']), float(wp['y']), 0.0, float(wp.get('yaw', 0.0))))
            except Exception as e:
                self.get_logger().error(f'Failed to load path YAML: {e}')
                return
        else:
            if not db_path:
                self.get_logger().error(
                    'No offline path input configured. Set path_yaml, database_path, '
                    'or PARKING_ROBOT_OFFLINE_PATH_DATABASE.'
                )
                return
            if not os.path.isfile(db_path):
                self.get_logger().error(
                    f'database_path is not a readable file: {db_path}'
                )
                return
            conn = sqlite3.connect(db_path)
            cur = conn.cursor()
            cur.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
            for row in cur.fetchall():
                pose_blob = row[1]
                if pose_blob and len(pose_blob) >= 48:
                    vals = struct.unpack(f'<{len(pose_blob)//4}f', pose_blob)
                    T = np.eye(4, dtype=np.float64)
                    T[:3, :] = np.array(vals[:12]).reshape(3, 4)
                    x, y, z = T[0, 3], T[1, 3], 0.0
                    yaw = math.atan2(T[1, 0], T[0, 0])
                    poses.append((x, y, z, yaw))
            conn.close()

        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id

        for x, y, z, yaw in poses:
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            self.path_msg.poses.append(ps)

        self.get_logger().info(f'Loaded {len(poses)} poses into path')

    def publish_path(self):
        if self.path_msg is not None:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
