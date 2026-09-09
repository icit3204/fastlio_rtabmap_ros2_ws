"""Publish observation-only nearest-path and current-goal RViz markers.

This node has no command publishers, action clients, service clients, or CAN
access.  It consumes the already-authoritative planner path, TF, RouteMission,
and MissionState streams only.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from parking_robot_interfaces.msg import MissionState, RouteMission
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


Point2 = Tuple[float, float]


def nearest_index(points: Sequence[Point2], robot_xy: Point2) -> Optional[int]:
    """Return the nearest sampled path pose, or None for an invalid path."""
    if not points or not all(math.isfinite(v) for p in points for v in p):
        return None
    rx, ry = robot_xy
    if not math.isfinite(rx) or not math.isfinite(ry):
        return None
    return min(range(len(points)), key=lambda i: (points[i][0] - rx) ** 2 + (points[i][1] - ry) ** 2)


def indexed_goal(poses: Sequence, index: int) -> Optional[object]:
    """Return the current metric goal pose only for an in-range index."""
    if index < 0 or index >= len(poses):
        return None
    return poses[index]


def _delete_marker(namespace: str, marker_id: int, frame_id: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.action = Marker.DELETE
    return marker


class NavigationVisualizationHelper(Node):
    """Observation-only marker publisher for the canonical navigation view."""

    def __init__(self) -> None:
        super().__init__('navigation_visualization_helper')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('nearest_topic', '/visualization/nearest_path_point')
        self.declare_parameter('goal_topic', '/visualization/current_navigation_goal')
        self.declare_parameter('publish_rate_hz', 10.0)

        map_frame = str(self.get_parameter('map_frame').value)
        self._map_frame = map_frame
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._plan: Optional[Path] = None
        self._route: Optional[RouteMission] = None
        self._state: Optional[MissionState] = None

        marker_qos = QoSProfile(depth=10)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._nearest_pub = self.create_publisher(
            Marker, str(self.get_parameter('nearest_topic').value), marker_qos)
        self._goal_pub = self.create_publisher(
            Marker, str(self.get_parameter('goal_topic').value), marker_qos)
        self.create_subscription(Path, str(self.get_parameter('plan_topic').value), self._plan_cb, 10)
        self.create_subscription(RouteMission, '/mission/route', self._route_cb, qos)
        self.create_subscription(MissionState, '/mission/state', self._state_cb, qos)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate_hz').value), self._publish)

    def _plan_cb(self, msg: Path) -> None:
        self._plan = msg

    def _route_cb(self, msg: RouteMission) -> None:
        self._route = msg

    def _state_cb(self, msg: MissionState) -> None:
        self._state = msg

    def _lookup_robot_xy(self) -> Optional[Point2]:
        try:
            transform: TransformStamped = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return None
        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        return (x, y) if math.isfinite(x) and math.isfinite(y) else None

    def _nearest_marker(self) -> Marker:
        if self._plan is None or self._plan.header.frame_id != self._map_frame:
            return _delete_marker('nearest_path_point', 0, self._map_frame)
        robot_xy = self._lookup_robot_xy()
        points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in self._plan.poses]
        index = nearest_index(points, robot_xy) if robot_xy is not None else None
        if index is None:
            return _delete_marker('nearest_path_point', 0, self._map_frame)
        pose = self._plan.poses[index].pose
        marker = Marker()
        marker.header = self._plan.header
        marker.header.frame_id = self._map_frame
        marker.ns = 'nearest_path_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = pose.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.18
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.1, 0.9, 1.0
        return marker

    def _goal_marker(self) -> Marker:
        if self._route is None or self._state is None:
            return _delete_marker('current_navigation_goal', 0, self._map_frame)
        if self._route.header.frame_id != self._map_frame:
            return _delete_marker('current_navigation_goal', 0, self._map_frame)
        pose = indexed_goal(self._route.poses, int(self._state.current_waypoint_index))
        if pose is None or pose.header.frame_id not in ('', self._map_frame):
            return _delete_marker('current_navigation_goal', 0, self._map_frame)
        marker = Marker()
        marker.header = pose.header
        marker.header.frame_id = self._map_frame
        marker.ns = 'current_navigation_goal'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x, marker.scale.y, marker.scale.z = 0.55, 0.12, 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.1, 1.0
        return marker

    def _publish(self) -> None:
        self._nearest_pub.publish(self._nearest_marker())
        self._goal_pub.publish(self._goal_marker())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationVisualizationHelper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
