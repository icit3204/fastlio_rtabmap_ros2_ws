"""Typed Mission Manager transport bridge for plan_nav mission mode."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import threading
import time
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


class AuthorityMode(str, Enum):
    LEGACY = "legacy"
    MISSION_NAV2 = "mission_nav2"


TERMINAL_MISSION_STATES = {
    7,   # CANCELLED
    8,   # SUCCEEDED
    10,  # BLOCKED
    11,  # FAILED
    12,  # HELP_REQUIRED
}


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
    """Thin ROS transport adapter. Mission Manager remains authoritative."""

    def __init__(self, *, state_callback=None, service_callback=None, node_name: str = "plan_nav_mission_bridge"):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_srvs.srv import SetBool, Trigger
        from parking_robot_interfaces.msg import MissionState, RouteMission

        self._Node = Node
        self._Trigger = Trigger
        self._SetBool = SetBool
        self._MissionState = MissionState
        self.node = Node(node_name)
        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_callback = state_callback
        self._service_callback = service_callback
        self._route_pub = self.node.create_publisher(RouteMission, "/mission/route", route_qos)
        self._state_sub = self.node.create_subscription(MissionState, "/mission/state", self._on_state, state_qos)
        self._start_client = self.node.create_client(Trigger, "/mission/start")
        self._cancel_client = self.node.create_client(Trigger, "/mission/cancel")
        self._pause_client = self.node.create_client(SetBool, "/mission/pause")
        self.futures = []
        self.published_missions: list[str] = []

    def destroy(self) -> None:
        self.node.destroy_node()

    def publish_route(self, route: RouteSpec) -> object:
        msg = route_spec_to_msg(route, self.node)
        self._route_pub.publish(msg)
        self.published_missions.append(msg.mission_id)
        return msg

    def request_start(self):
        return self._call_trigger(self._start_client, "start")

    def request_cancel(self):
        return self._call_trigger(self._cancel_client, "cancel")

    def request_pause(self, pause: bool):
        req = self._SetBool.Request()
        req.data = bool(pause)
        return self._call_async(self._pause_client, req, "pause" if pause else "resume")

    def services_ready(self) -> bool:
        return (
            self._start_client.service_is_ready()
            and self._cancel_client.service_is_ready()
            and self._pause_client.service_is_ready()
        )

    def _call_trigger(self, client, name: str):
        req = self._Trigger.Request()
        return self._call_async(client, req, name)

    def _call_async(self, client, req, name: str):
        if not client.service_is_ready():
            if self._service_callback:
                self._service_callback(name, False, "SERVICE_UNAVAILABLE", f"{name} service unavailable")
            return None
        future = client.call_async(req)
        self.futures.append(future)
        future.add_done_callback(lambda done, service_name=name: self._on_service_result(service_name, done))
        return future

    def _on_service_result(self, name: str, future) -> None:
        try:
            result = future.result()
            success = bool(getattr(result, "success", False))
            message = str(getattr(result, "message", ""))
            reason = "REQUEST_ACCEPTED" if success else "REQUEST_REJECTED"
        except Exception as exc:
            success = False
            reason = "SERVICE_EXCEPTION"
            message = f"{type(exc).__name__}: {exc}"
        if self._service_callback:
            self._service_callback(name, success, reason, message)

    def _on_state(self, msg) -> None:
        if self._state_callback:
            self._state_callback(msg)

    def prune_futures(self) -> None:
        self.futures = [future for future in self.futures if not future.done()]


class MissionBridgeThread(QThread):
    """QThread wrapper that emits Qt signals from ROS callbacks."""

    mission_state_received = pyqtSignal(object)
    service_result_received = pyqtSignal(str, bool, str, str)
    connected = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._bridge: MissionBridgeNode | None = None
        self._lock = threading.Lock()

    def run(self) -> None:
        self._running = True
        try:
            from rclpy.executors import SingleThreadedExecutor
            from core.ros_runtime import ensure_rclpy_initialized

            ensure_rclpy_initialized(args=[])
            bridge = MissionBridgeNode(
                state_callback=self.mission_state_received.emit,
                service_callback=self.service_result_received.emit,
            )
            executor = SingleThreadedExecutor()
            executor.add_node(bridge.node)
            with self._lock:
                self._bridge = bridge
            self.connected.emit("/mission bridge ready")
            while self._running:
                executor.spin_once(timeout_sec=0.05)
                bridge.prune_futures()
            executor.remove_node(bridge.node)
            bridge.destroy()
        except Exception as exc:
            self.error_occurred.emit(f"MissionBridge: {type(exc).__name__}: {exc}")
        finally:
            with self._lock:
                self._bridge = None

    def publish_route(self, route: RouteSpec) -> bool:
        with self._lock:
            bridge = self._bridge
        if bridge is None:
            self.error_occurred.emit("MissionBridge: bridge not ready")
            return False
        bridge.publish_route(route)
        return True

    def request_start(self) -> bool:
        return self._request("start")

    def request_cancel(self) -> bool:
        return self._request("cancel")

    def request_pause(self) -> bool:
        return self._request("pause", True)

    def request_resume(self) -> bool:
        return self._request("pause", False)

    def _request(self, name: str, value=None) -> bool:
        with self._lock:
            bridge = self._bridge
        if bridge is None:
            self.error_occurred.emit("MissionBridge: bridge not ready")
            return False
        if name == "start":
            bridge.request_start()
        elif name == "cancel":
            bridge.request_cancel()
        elif name == "pause":
            bridge.request_pause(bool(value))
        return True

    def stop(self) -> None:
        self._running = False
        self.wait(3000)
