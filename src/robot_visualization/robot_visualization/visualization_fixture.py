"""Offline visualization-only fixture; it publishes no motion authority."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from parking_robot_interfaces.msg import MissionState, RouteMission
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class VisualizationFixture(Node):
    """Publish synthetic map/path/mission/TF data for offline marker tests."""

    def __init__(self) -> None:
        super().__init__('visualization_fixture')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self._plan_pub = self.create_publisher(Path, '/plan', qos)
        self._route_pub = self.create_publisher(RouteMission, '/mission/route', qos)
        self._state_pub = self.create_publisher(MissionState, '/mission/state', qos)
        self._tf = TransformBroadcaster(self)
        self._timer = self.create_timer(0.2, self._publish)

    def _pose(self, x: float, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, 'map'
        grid.info.resolution, grid.info.width, grid.info.height = 0.1, 100, 100
        grid.info.origin.position.x, grid.info.origin.position.y = -5.0, -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)

        path = Path()
        path.header.stamp, path.header.frame_id = stamp, 'map'
        path.poses = [self._pose(i * 0.2) for i in range(26)]

        route = RouteMission()
        route.header.stamp, route.header.frame_id = stamp, 'map'
        route.mission_id, route.route_id, route.topology_version = 'fixture', 'fixture-route', 'v1'
        route.node_ids, route.edge_ids, route.edge_directions = ['start', 'goal'], ['edge-0'], [0]
        route.poses = [self._pose(0.0), self._pose(5.0)]

        state = MissionState()
        state.header.stamp, state.header.frame_id = stamp, 'map'
        state.mission_id, state.route_id = route.mission_id, route.route_id
        state.state = MissionState.NAVIGATING
        state.current_waypoint_index, state.total_waypoint_count = 0, 2
        state.progress, state.reason_code = 0.0, 'FIXTURE'

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
        transform.transform.rotation.w = 1.0

        self._map_pub.publish(grid)
        self._plan_pub.publish(path)
        self._route_pub.publish(route)
        self._state_pub.publish(state)
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualizationFixture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
