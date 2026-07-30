#!/usr/bin/python3
"""Replay a recorded path with obstacle avoidance.

Reads waypoints from YAML and sends them one at a time via
Nav2's NavigateToPose action. MPPI handles local obstacle
avoidance automatically for each segment.
"""
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw):
    return Quaternion(
        z=float(math.sin(yaw / 2.0)),
        w=float(math.cos(yaw / 2.0)),
    )


class PathReplayer(Node):
    def __init__(self):
        super().__init__('path_replayer')
        self.declare_parameter('input', '/home/dog/my_path.yaml')
        self.declare_parameter('waypoint_spacing', 1.0)  # meters between waypoints

        self.input_path = self.get_parameter('input').value
        self.spacing = self.get_parameter('waypoint_spacing').value

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = []
        self.current_idx = 0
        self.get_logger().info(f'Loading path from {self.input_path}')

    def load_and_replay(self):
        with open(self.input_path, 'r') as f:
            data = yaml.safe_load(f)

        raw = data['waypoints']
        self.get_logger().info(f'Loaded {len(raw)} raw waypoints')

        # Downsample to waypoint_spacing
        self.waypoints = []
        last = None
        for wp in raw:
            if last is not None:
                dx = wp['x'] - last['x']
                dy = wp['y'] - last['y']
                if (dx * dx + dy * dy) < self.spacing * self.spacing:
                    continue
            last = wp
            self.waypoints.append(wp)

        self.get_logger().info(f'Filtered to {len(self.waypoints)} waypoints (spacing={self.spacing}m)')

        self.get_logger().info('Waiting for NavigateToPose action server...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().fatal('Action server (navigate_to_pose) not available after 15s.')
            self.get_logger().fatal('Check: ros2 action list | grep navigate')
            raise RuntimeError('Action server not available')

        self.get_logger().info('Connected. Sending first waypoint...')
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints completed!')
            raise SystemExit(0)

        wp = self.waypoints[self.current_idx]
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(wp['x'])
        p.pose.position.y = float(wp['y'])
        p.pose.orientation = yaw_to_quat(float(wp['yaw']))

        goal = NavigateToPose.Goal()
        goal.pose = p

        self.get_logger().info(f'Waypoint {self.current_idx + 1}/{len(self.waypoints)}: ({p.pose.position.x:.2f}, {p.pose.position.y:.2f})')
        self._send_goal_future = self.client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Waypoint {self.current_idx + 1} rejected, skipping...')
            self.current_idx += 1
            self.send_next_waypoint()
            return
        self.get_logger().info('Goal accepted, navigating...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f'Waypoint {self.current_idx + 1} done (status={status})')
        self.current_idx += 1
        self.send_next_waypoint()


def main():
    rclpy.init()
    node = PathReplayer()
    try:
        node.load_and_replay()
    except Exception as e:
        node.get_logger().error(str(e))
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
