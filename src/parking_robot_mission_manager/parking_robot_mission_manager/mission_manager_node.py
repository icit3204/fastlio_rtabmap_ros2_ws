"""ROS transport adapter for the typed Phase 3 Mission Manager."""

from __future__ import annotations

from functools import partial
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav2_msgs.action import NavigateToPose
from parking_robot_interfaces.msg import MissionState, RouteMission
from std_srvs.srv import SetBool, Trigger

from .mission_state_machine import (
    GoalOutcome,
    GoalResultCode,
    MissionGoalExecutor,
    MissionSnapshot,
    MissionStateCode,
    MissionStateMachine,
)


def _status_name(status: int) -> str:
    names = {
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
    }
    return names.get(status, f"STATUS_{status}")


def _goal_result_code(status: int) -> GoalResultCode:
    if status == GoalStatus.STATUS_SUCCEEDED:
        return GoalResultCode.SUCCEEDED
    if status == GoalStatus.STATUS_CANCELED:
        return GoalResultCode.CANCELED
    return GoalResultCode.ABORTED


class _NavigateToPoseTransport(MissionGoalExecutor):
    """Action transport used by MissionManagerNode.

    This class owns ROS action handles and Futures. It never owns mission
    state; every state-visible event is forwarded to MissionStateMachine.
    """

    def __init__(self, node: "MissionManagerNode", action_name: str) -> None:
        self.node = node
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.goal_handles: Dict[str, object] = {}
        self.futures: List[object] = []
        self.cancel_request_count = 0

    def server_available(self) -> bool:
        return self.client.server_is_ready()

    def send_goal(self, pose, result_callback) -> GoalOutcome:
        del result_callback
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.header.frame_id = pose.frame_id
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        goal.pose.pose.position.z = pose.z
        goal.pose.pose.orientation.x = pose.qx
        goal.pose.pose.orientation.y = pose.qy
        goal.pose.pose.orientation.z = pose.qz
        goal.pose.pose.orientation.w = pose.qw
        future = self.client.send_goal_async(goal)
        self.futures.append(future)
        future.add_done_callback(partial(self._goal_response_cb, waypoint_id=str(getattr(pose, "waypoint_id", ""))))
        return GoalOutcome(True, reason_code="GOAL_PENDING", detail="goal request sent")

    def cancel_goal(self, goal_uuid: str, timeout_sec: float) -> bool:
        del timeout_sec
        goal_handle = self.goal_handles.get(goal_uuid)
        if goal_handle is None:
            return False
        self.cancel_request_count += 1
        future = goal_handle.cancel_goal_async()
        self.futures.append(future)
        future.add_done_callback(self._cancel_response_cb)
        return True

    def prune_futures(self) -> None:
        self.futures = [future for future in self.futures if not future.done()]

    def _goal_response_cb(self, future, waypoint_id: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.apply_core_event(
                lambda core: core.on_goal_rejected("GOAL_RESPONSE_EXCEPTION", f"{type(exc).__name__}: {exc}")
            )
            return
        if not goal_handle.accepted:
            self.node.apply_core_event(lambda core: core.on_goal_rejected("GOAL_REJECTED", f"waypoint {waypoint_id} rejected"))
            return
        goal_uuid = bytes(goal_handle.goal_id.uuid).hex()
        self.goal_handles[goal_uuid] = goal_handle
        self.node.apply_core_event(lambda core: core.on_goal_accepted(goal_uuid))
        result_future = goal_handle.get_result_async()
        self.futures.append(result_future)
        result_future.add_done_callback(partial(self._result_cb, goal_uuid=goal_uuid))

    def _result_cb(self, future, goal_uuid: str) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self.node.apply_core_event(
                lambda core: core.on_goal_result(GoalResultCode.ABORTED, f"RESULT_EXCEPTION: {type(exc).__name__}: {exc}")
            )
            return
        status = int(wrapped.status)
        self.goal_handles.pop(goal_uuid, None)
        self.node.apply_core_event(lambda core: core.on_goal_result(_goal_result_code(status), _status_name(status)))

    def _cancel_response_cb(self, future) -> None:
        try:
            response = future.result()
        except Exception:
            self.node.apply_core_event(lambda core: core.on_cancel_response_rejected())
            return
        if getattr(response, "goals_canceling", []):
            self.node.apply_core_event(lambda core: core.on_cancel_response_accepted())
        else:
            self.node.apply_core_event(lambda core: core.on_cancel_response_rejected())


class MissionManagerNode(Node):
    """Sequential typed RouteMission executor.

    This node publishes no Twist or velocity-equivalent command. Navigation is
    delegated exclusively to the standard NavigateToPose action interface.
    """

    def __init__(
        self,
        *,
        goal_executor: Optional[MissionGoalExecutor] = None,
        steady_clock=time.monotonic,
        parameter_overrides=None,
    ) -> None:
        super().__init__("mission_manager", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("mission_topic", "/mission/route")
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("expected_topology_version", "v1")
        self.declare_parameter("goal_xy_tolerance_m", 0.25)
        self.declare_parameter("waypoint_separation_margin_m", 0.05)
        self.declare_parameter("min_waypoint_separation_m", 0.55)
        self.declare_parameter("cancel_response_timeout_sec", 2.0)
        self.declare_parameter("cancel_result_timeout_sec", 5.0)

        self._lock = threading.RLock()
        self._steady_clock = steady_clock

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(MissionState, str(self.get_parameter("state_topic").value), state_qos)
        self._status_pub = self.create_publisher(DiagnosticStatus, "/mission/status", state_qos)
        self._block_reason_pub = self.create_publisher(DiagnosticStatus, "/mission/block_reason", state_qos)
        self._mission_sub = self.create_subscription(
            RouteMission,
            str(self.get_parameter("mission_topic").value),
            self._mission_cb,
            QoSProfile(depth=10),
        )
        self._start_srv = self.create_service(Trigger, "/mission/start", self._start_cb)
        self._cancel_srv = self.create_service(Trigger, "/mission/cancel", self._cancel_cb)
        self._pause_srv = self.create_service(SetBool, "/mission/pause", self._pause_cb)

        self._transport = goal_executor
        if self._transport is None:
            self._transport = _NavigateToPoseTransport(self, str(self.get_parameter("navigate_to_pose_action").value))
        self._action_client = getattr(self._transport, "client", None)

        self._latest_snapshot: Optional[MissionSnapshot] = None
        self._core = MissionStateMachine(
            self._transport,
            expected_topology_version=str(self.get_parameter("expected_topology_version").value),
            state_callback=self._publish_snapshot,
            goal_xy_tolerance_m=float(self.get_parameter("goal_xy_tolerance_m").value),
            waypoint_separation_margin_m=float(self.get_parameter("waypoint_separation_margin_m").value),
            min_waypoint_separation_m=float(self.get_parameter("min_waypoint_separation_m").value),
            cancel_response_timeout_sec=float(self.get_parameter("cancel_response_timeout_sec").value),
            cancel_result_timeout_sec=float(self.get_parameter("cancel_result_timeout_sec").value),
            steady_clock=self._steady_clock,
        )
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_cb)

    @property
    def mission_state_machine(self) -> MissionStateMachine:
        return self._core

    def apply_core_event(self, callback) -> None:
        with self._lock:
            callback(self._core)

    def _publish_snapshot(self, snapshot: MissionSnapshot) -> None:
        self._latest_snapshot = snapshot
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.mission_id = snapshot.mission_id
        msg.route_id = snapshot.route_id
        msg.state = int(snapshot.state)
        msg.current_waypoint_index = snapshot.current_waypoint_index
        msg.completed_waypoint_count = snapshot.completed_waypoint_count
        msg.total_waypoint_count = snapshot.total_waypoint_count
        msg.progress = float(snapshot.progress)
        msg.active_goal_uuid = snapshot.active_goal_uuid
        msg.reason_code = snapshot.reason_code
        msg.detail = snapshot.detail
        self._state_pub.publish(msg)

        diagnostic = DiagnosticStatus()
        diagnostic.name = "mission_manager"
        diagnostic.level = DiagnosticStatus.ERROR if snapshot.state in (
            MissionStateCode.FAILED,
            MissionStateCode.BLOCKED,
            MissionStateCode.HELP_REQUIRED,
        ) else DiagnosticStatus.OK
        diagnostic.message = snapshot.reason_code or snapshot.detail or snapshot.state.name
        diagnostic.values = [
            KeyValue(key="state", value=snapshot.state.name),
            KeyValue(key="mission_id", value=snapshot.mission_id),
            KeyValue(key="route_id", value=snapshot.route_id),
            KeyValue(key="reason_code", value=snapshot.reason_code),
            KeyValue(key="active_goal_uuid", value=snapshot.active_goal_uuid),
        ]
        self._status_pub.publish(diagnostic)
        if snapshot.state in (MissionStateCode.TEMPORARILY_BLOCKED, MissionStateCode.BLOCKED, MissionStateCode.HELP_REQUIRED):
            self._block_reason_pub.publish(diagnostic)

    def _mission_cb(self, msg: RouteMission) -> None:
        with self._lock:
            self._core.receive_mission(msg)

    def _start_cb(self, request, response):
        del request
        with self._lock:
            result = self._core.start()
            response.success = bool(result.valid and self._core.state != MissionStateCode.FAILED)
            response.message = result.reason_code if result.reason_code else ("mission started" if response.success else "start refused")
        return response

    def _cancel_cb(self, request, response):
        del request
        with self._lock:
            accepted = self._core.request_cancel()
            response.success = bool(accepted)
            response.message = "cancel request accepted" if accepted else "cancel not accepted in current state"
        return response

    def _pause_cb(self, request, response):
        with self._lock:
            if request.data:
                accepted = self._core.request_pause()
                response.message = "pause request accepted" if accepted else "pause not accepted in current state"
            else:
                accepted = self._core.resume()
                response.message = "resume accepted" if accepted else "resume requires PAUSED"
            response.success = bool(accepted)
        return response

    def _watchdog_cb(self) -> None:
        with self._lock:
            self._core.tick(self._steady_clock())
            if hasattr(self._transport, "prune_futures"):
                self._transport.prune_futures()

    def destroy_node(self) -> bool:
        if hasattr(self, "_watchdog_timer"):
            self.destroy_timer(self._watchdog_timer)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
