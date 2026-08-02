"""ROS node for the typed Phase 3 Mission Manager."""

from __future__ import annotations

from functools import partial
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from parking_robot_interfaces.msg import MissionState, RouteMission
from std_srvs.srv import Trigger

from .route_contract import NormalizedRouteMission, validate_route_mission


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


class MissionManagerNode(Node):
    """Sequential typed RouteMission executor.

    This node publishes no Twist or velocity-equivalent command. Navigation is
    delegated exclusively to the standard NavigateToPose action interface.
    """

    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("mission_topic", "/mission_manager/route_mission")
        self.declare_parameter("state_topic", "/mission_manager/state")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("min_waypoint_separation_m", 0.05)
        self.declare_parameter("cancel_timeout_sec", 2.0)

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(MissionState, str(self.get_parameter("state_topic").value), state_qos)
        self._mission_sub = self.create_subscription(
            RouteMission,
            str(self.get_parameter("mission_topic").value),
            self._mission_cb,
            QoSProfile(depth=10),
        )
        self._pause_srv = self.create_service(Trigger, "/mission_manager/pause", self._pause_cb)
        self._resume_srv = self.create_service(Trigger, "/mission_manager/resume", self._resume_cb)
        self._cancel_srv = self.create_service(Trigger, "/mission_manager/cancel", self._cancel_cb)
        self._action_client = ActionClient(self, NavigateToPose, str(self.get_parameter("navigate_to_pose_action").value))

        self._state = MissionState.IDLE
        self._mission: Optional[NormalizedRouteMission] = None
        self._current_index = 0
        self._completed_count = 0
        self._active_goal_uuid = ""
        self._active_goal_handle = None
        self._goal_futures: List[object] = []
        self._result_futures: List[object] = []
        self._cancel_futures: List[object] = []
        self._canceling_for_pause = False
        self._terminal_published = False
        self._publish_state("IDLE", "")

    def _publish_state(self, reason_code: str = "", detail: str = "") -> None:
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.mission_id = self._mission.mission_id if self._mission else ""
        msg.route_id = self._mission.route_id if self._mission else ""
        msg.state = self._state
        msg.current_waypoint_index = self._current_index
        msg.completed_waypoint_count = self._completed_count
        total = len(self._mission.waypoints) if self._mission else 0
        msg.total_waypoint_count = total
        msg.progress = float(self._completed_count / total) if total else 0.0
        msg.active_goal_uuid = self._active_goal_uuid
        msg.reason_code = reason_code
        msg.detail = detail
        self._state_pub.publish(msg)

    def _set_state(self, state: int, reason_code: str = "", detail: str = "") -> None:
        self._state = state
        self._publish_state(reason_code, detail)

    def _mission_cb(self, msg: RouteMission) -> None:
        if self._state not in (MissionState.IDLE, MissionState.SUCCEEDED, MissionState.FAILED, MissionState.REJECTED):
            self._set_state(MissionState.REJECTED, "MISSION_ALREADY_ACTIVE", "only one mission may be active")
            return
        self._mission = None
        self._current_index = 0
        self._completed_count = 0
        self._active_goal_uuid = ""
        self._active_goal_handle = None
        self._terminal_published = False
        self._set_state(MissionState.VALIDATING)
        validation = validate_route_mission(
            msg,
            min_waypoint_separation_m=float(self.get_parameter("min_waypoint_separation_m").value),
        )
        if not validation.valid:
            self._set_state(MissionState.REJECTED, validation.reason_code, validation.detail)
            return
        self._mission = validation.mission
        self._set_state(MissionState.READY)
        self._dispatch_current_waypoint()

    def _dispatch_current_waypoint(self) -> None:
        if self._mission is None:
            self._set_state(MissionState.FAILED, "NO_ACTIVE_MISSION", "missing mission")
            return
        if self._current_index >= len(self._mission.waypoints):
            self._active_goal_uuid = ""
            self._set_state(MissionState.SUCCEEDED, "MISSION_SUCCEEDED", "all waypoints completed")
            return
        waypoint = self._mission.waypoints[self._current_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = waypoint.pose.frame_id
        goal.pose.pose.position.x = waypoint.pose.x
        goal.pose.pose.position.y = waypoint.pose.y
        goal.pose.pose.position.z = waypoint.pose.z
        goal.pose.pose.orientation.x = waypoint.pose.qx
        goal.pose.pose.orientation.y = waypoint.pose.qy
        goal.pose.pose.orientation.z = waypoint.pose.qz
        goal.pose.pose.orientation.w = waypoint.pose.qw
        self._set_state(MissionState.RUNNING)
        if not self._action_client.server_is_ready():
            self._set_state(MissionState.FAILED, "ACTION_SERVER_UNAVAILABLE", "NavigateToPose server unavailable")
            return
        future = self._action_client.send_goal_async(goal)
        self._goal_futures.append(future)
        future.add_done_callback(partial(self._goal_response_cb, waypoint_id=waypoint.waypoint_id))

    def _goal_response_cb(self, future, waypoint_id: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._set_state(MissionState.FAILED, "GOAL_RESPONSE_EXCEPTION", f"{type(exc).__name__}: {exc}")
            return
        if not goal_handle.accepted:
            self._set_state(MissionState.FAILED, "GOAL_REJECTED", f"waypoint {waypoint_id} rejected")
            return
        self._active_goal_handle = goal_handle
        self._active_goal_uuid = bytes(goal_handle.goal_id.uuid).hex()
        self._publish_state("GOAL_ACCEPTED", waypoint_id)
        result_future = goal_handle.get_result_async()
        self._result_futures.append(result_future)
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        if self._state == MissionState.PAUSED:
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self._set_state(MissionState.FAILED, "RESULT_EXCEPTION", f"{type(exc).__name__}: {exc}")
            return
        status = int(wrapped.status)
        if self._canceling_for_pause:
            self._canceling_for_pause = False
            self._active_goal_uuid = ""
            self._active_goal_handle = None
            self._set_state(MissionState.PAUSED, "MISSION_PAUSED", "active goal canceled; waypoint retained")
            return
        if self._state == MissionState.CANCELING:
            self._active_goal_uuid = ""
            self._active_goal_handle = None
            self._set_state(MissionState.IDLE, "MISSION_CANCELED", "mission canceled")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._completed_count += 1
            self._current_index += 1
            self._active_goal_uuid = ""
            self._active_goal_handle = None
            self._publish_state("WAYPOINT_SUCCEEDED", _status_name(status))
            self._dispatch_current_waypoint()
            return
        self._set_state(MissionState.FAILED, f"GOAL_{_status_name(status)}", f"waypoint {self._current_index} failed")

    def _pause_cb(self, request, response):
        del request
        if self._state == MissionState.PAUSED:
            response.success = True
            response.message = "already paused"
            return response
        if self._state != MissionState.RUNNING:
            response.success = False
            response.message = "pause requires RUNNING"
            return response
        if self._active_goal_handle is None:
            self._set_state(MissionState.PAUSED, "MISSION_PAUSED", "no active goal handle")
            response.success = True
            response.message = "paused"
            return response
        self._canceling_for_pause = True
        cancel_future = self._active_goal_handle.cancel_goal_async()
        self._cancel_futures.append(cancel_future)
        response.success = True
        response.message = "pause cancellation requested"
        return response

    def _resume_cb(self, request, response):
        del request
        if self._state == MissionState.RUNNING:
            response.success = True
            response.message = "already running"
            return response
        if self._state != MissionState.PAUSED:
            response.success = False
            response.message = "resume requires PAUSED"
            return response
        self._set_state(MissionState.RUNNING, "MISSION_RESUMED", "current waypoint will be re-sent")
        self._dispatch_current_waypoint()
        response.success = True
        response.message = "resumed"
        return response

    def _cancel_cb(self, request, response):
        del request
        if self._state == MissionState.IDLE:
            response.success = True
            response.message = "already idle"
            return response
        if self._state not in (MissionState.READY, MissionState.RUNNING, MissionState.PAUSED):
            response.success = False
            response.message = "cancel not accepted in current state"
            return response
        self._set_state(MissionState.CANCELING, "MISSION_CANCEL_REQUESTED", "cancel requested")
        if self._active_goal_handle is None:
            self._set_state(MissionState.IDLE, "MISSION_CANCELED", "mission canceled")
        else:
            cancel_future = self._active_goal_handle.cancel_goal_async()
            self._cancel_futures.append(cancel_future)
        response.success = True
        response.message = "cancel requested"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
