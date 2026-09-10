"""Typed RouteMission publication bridge for PlanNav route intent."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import uuid

from PyQt5.QtCore import QThread, pyqtSignal

from core.pathfinder import build_graph, find_path
from core.topology_identity import (
    MANIFEST_NAME,
    RouteSpec,
    build_sparse_route_spec,
    load_topology,
    read_manifest,
)


@dataclass(frozen=True)
class RoutePreparationResult:
    valid: bool
    reason_code: str
    detail: str
    route: RouteSpec | None = None
    path_node_ids: list[int] | None = None


def prepare_route_spec_from_topology(
    *,
    work_dir: str | Path,
    start_node_id: int,
    end_node_id: int,
    mission_id: str | None = None,
) -> RoutePreparationResult:
    """Build the P3-B.1 sparse RouteSpec for one selected Dijkstra route."""
    work_path = Path(work_dir)
    nodes, edges, _legacy = load_topology(work_path)
    manifest = read_manifest(work_path)
    if manifest is None:
        return RoutePreparationResult(False, "TOPOLOGY_MANIFEST_MISSING", f"{MANIFEST_NAME} is missing")
    waypoints = [
        {
            "id": node.node_id,
            "x": node.x,
            "y": node.y,
            "z": node.z,
            "yaw": node.yaw,
            "timestamp": node.timestamp,
            "traj_idx": node.traj_idx,
        }
        for node in nodes
    ]
    graph_edges = [
        {
            "edge_id": edge.edge_id,
            "from_id": edge.from_id,
            "to_id": edge.to_id,
            "length": edge.length_m,
            "direction": edge.direction,
            "traj_file": edge.traj_file,
        }
        for edge in edges
    ]
    path_ids, _length = find_path(build_graph(waypoints, graph_edges), start_node_id, end_node_id)
    if not path_ids:
        return RoutePreparationResult(False, "NO_TOPOLOGICAL_ROUTE", f"no Dijkstra route {start_node_id}->{end_node_id}")
    result = build_sparse_route_spec(
        mission_id=mission_id or f"mission-{uuid.uuid4().hex}",
        ordered_node_ids=[int(item) for item in path_ids],
        nodes=nodes,
        edges=edges,
        topology_manifest=manifest,
    )
    return RoutePreparationResult(result.valid, result.reason_code, result.detail, result.route, list(path_ids))


def verify_route_topology_current(work_dir: str | Path, route: RouteSpec) -> tuple[bool, str, str]:
    manifest = read_manifest(Path(work_dir))
    if manifest is None:
        return False, "TOPOLOGY_MANIFEST_MISSING", f"{MANIFEST_NAME} is missing"
    current = str(manifest.get("topology_version", ""))
    if current != route.topology_version:
        return False, "TOPOLOGY_CHANGED_AFTER_PLANNING", (
            f"planned {route.topology_version}, current {current}"
        )
    return True, "TOPOLOGY_CURRENT", "topology version still matches planned RouteSpec"


def route_spec_to_msg(route: RouteSpec, node) -> object:
    """Convert a RouteSpec into parking_robot_interfaces/RouteMission."""
    from geometry_msgs.msg import PoseStamped
    from parking_robot_interfaces.msg import RouteMission

    msg = RouteMission()
    msg.header.frame_id = route.header_frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.mission_id = route.mission_id
    msg.route_id = route.route_id
    msg.topology_version = route.topology_version
    msg.node_ids = list(route.node_ids)
    msg.edge_ids = list(route.edge_ids)
    msg.edge_directions = [int(item) for item in route.edge_directions]
    for pose_data in route.poses:
        pose = PoseStamped()
        pose.header.frame_id = str(pose_data.get("frame_id", route.header_frame_id))
        pose.header.stamp = msg.header.stamp
        position = pose_data["position"]
        orientation = pose_data["orientation"]
        pose.pose.position.x = float(position["x"])
        pose.pose.position.y = float(position["y"])
        pose.pose.position.z = float(position["z"])
        pose.pose.orientation.x = float(orientation["x"])
        pose.pose.orientation.y = float(orientation["y"])
        pose.pose.orientation.z = float(orientation["z"])
        pose.pose.orientation.w = float(orientation["w"])
        msg.poses.append(pose)
    return msg


class MissionBridgeNode:
    """Publish route intent only; runtime mission control belongs to Operator GUI."""

    def __init__(self, *, node_name: str = "plan_nav_route_publisher"):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from parking_robot_interfaces.msg import RouteMission

        self.node = Node(node_name)
        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._route_pub = self.node.create_publisher(RouteMission, "/mission/route", route_qos)
        self.published_missions: list[str] = []

    def destroy(self) -> None:
        self.node.destroy_node()

    def publish_route(self, route: RouteSpec) -> object:
        msg = route_spec_to_msg(route, self.node)
        self._route_pub.publish(msg)
        self.published_missions.append(msg.mission_id)
        return msg

class MissionBridgeThread(QThread):
    """QThread wrapper for explicit, queued RouteMission publication."""

    route_published = pyqtSignal(str, str)
    connected = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._bridge: MissionBridgeNode | None = None
        self._lock = threading.Lock()
        self._pending_routes: list[RouteSpec] = []

    def run(self) -> None:
        self._running = True
        try:
            from rclpy.executors import SingleThreadedExecutor
            from core.ros_runtime import ensure_rclpy_initialized

            ensure_rclpy_initialized(args=[])
            bridge = MissionBridgeNode()
            executor = SingleThreadedExecutor()
            executor.add_node(bridge.node)
            with self._lock:
                self._bridge = bridge
            self.connected.emit("/mission/route publisher ready")
            while self._running:
                with self._lock:
                    pending = list(self._pending_routes)
                    self._pending_routes.clear()
                for route in pending:
                    bridge.publish_route(route)
                    self.route_published.emit(route.mission_id, route.route_id)
                executor.spin_once(timeout_sec=0.05)
            executor.remove_node(bridge.node)
            bridge.destroy()
        except Exception as exc:
            self.error_occurred.emit(f"MissionBridge: {type(exc).__name__}: {exc}")
        finally:
            with self._lock:
                self._bridge = None

    def publish_route(self, route: RouteSpec) -> bool:
        with self._lock:
            self._pending_routes.append(route)
        return True

    def stop(self) -> None:
        self._running = False
        self.wait(3000)
