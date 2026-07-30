#!/usr/bin/python3
"""Record robot path with timestamps from TF or Odometry.

During manual driving, records base_footprint pose in map frame
at a fixed rate. Saves waypoints to YAML on SIGINT.
"""
import os
import signal
import time
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        self.declare_parameter('output', '/data/paths/recorded_path.yaml')
        self.declare_parameter('rate', 5.0)  # Hz
        self.declare_parameter('source', 'odom')  # odom or tf
        # <修改 version3 支持指定里程计话题>
        self.declare_parameter('odom_topic', '/Odometry')

        self.output_path = self.get_parameter('output').value
        self.rate = self.get_parameter('rate').value
        self.source = self.get_parameter('source').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.waypoints = []
        self.start_time = None

        # <修改 version3 使用系统默认兼容QoS,匹配所有里程计发布方>
        if self.source == 'odom':
            self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.use_tf = True

        self.timer = self.create_timer(1.0 / self.rate, self.record)
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        self.get_logger().info(f'Recording at {self.rate} Hz to {self.output_path}')

    def odom_cb(self, msg):
        self.latest_pose = PoseStamped()
        self.latest_pose.header = msg.header
        self.latest_pose.pose = msg.pose.pose

    def record(self):
        try:
            if self.source == 'odom':
                if not hasattr(self, 'latest_pose'):
                    return
                pose = self.latest_pose.pose
                stamp = self.latest_pose.header.stamp
            else:
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                pose = t.transform
                stamp = t.header.stamp

            now = time.time()
            if self.start_time is None:
                self.start_time = now

            self.waypoints.append({
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'yaw': float(self._yaw_from_quat(pose.orientation)),
                't': now - self.start_time,
            })
        except Exception:
            pass

    @staticmethod
    def _yaw_from_quat(q):
        import math
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def save(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints recorded')
            return
        with open(self.output_path, 'w') as f:
            yaml.dump({'waypoints': self.waypoints}, f)
        self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {self.output_path}')


def main():
    rclpy.init()
    node = PathRecorder()

    def shutdown(sig, frame):
        node.save()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.save()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
