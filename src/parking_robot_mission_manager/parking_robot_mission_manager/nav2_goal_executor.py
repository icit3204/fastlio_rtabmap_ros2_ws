"""NavigateToPose goal executor implementations."""

from __future__ import annotations

import itertools
import uuid
from typing import Callable, List, Optional

from .mission_state_machine import GoalOutcome, GoalResultCode, MissionGoalExecutor


class ScriptedFakeGoalExecutor(MissionGoalExecutor):
    """Deterministic executor for unit and synthetic tests."""

    def __init__(self, outcomes: Optional[List[str]] = None) -> None:
        self.outcomes = list(outcomes or ["succeeded"])
        self.sent_waypoint_ids: List[str] = []
        self.goal_uuids: List[str] = []
        self.cancel_count = 0
        self.cancel_requested_goal_uuids: List[str] = []
        self._callbacks = {}
        self._counter = itertools.count(1)

    def send_goal(self, waypoint, result_callback: Callable[[GoalResultCode, str], None]) -> GoalOutcome:
        outcome = self.outcomes.pop(0) if self.outcomes else "succeeded"
        self.sent_waypoint_ids.append(waypoint.waypoint_id)
        if outcome == "server_unavailable":
            return GoalOutcome(False, reason_code="ACTION_SERVER_UNAVAILABLE", detail="NavigateToPose unavailable")
        if outcome == "rejected":
            return GoalOutcome(False, reason_code="GOAL_REJECTED", detail="NavigateToPose rejected goal")
        goal_uuid = uuid.uuid5(uuid.NAMESPACE_URL, f"p3a-fake-{next(self._counter)}-{waypoint.waypoint_id}").hex
        self.goal_uuids.append(goal_uuid)
        self._callbacks[goal_uuid] = result_callback
        if outcome == "succeeded":
            return GoalOutcome(True, goal_uuid, GoalResultCode.SUCCEEDED)
        if outcome == "aborted":
            return GoalOutcome(True, goal_uuid, GoalResultCode.ABORTED)
        if outcome == "canceled":
            return GoalOutcome(True, goal_uuid, GoalResultCode.CANCELED)
        if outcome == "exception":
            raise RuntimeError("scripted executor exception")
        if outcome == "delayed":
            return GoalOutcome(True, goal_uuid, None)
        raise ValueError(f"unknown scripted outcome {outcome}")

    def complete_active(self, status: GoalResultCode = GoalResultCode.SUCCEEDED) -> None:
        if not self.goal_uuids:
            raise RuntimeError("no active fake goal")
        goal_uuid = self.goal_uuids[-1]
        callback = self._callbacks[goal_uuid]
        callback(status, status.name)

    def cancel_goal(self, goal_uuid: str, timeout_sec: float) -> bool:
        del timeout_sec
        self.cancel_count += 1
        self.cancel_requested_goal_uuids.append(goal_uuid)
        return True


class NavigateToPoseGoalExecutor(MissionGoalExecutor):
    """Real action-client wrapper for later Phase 3 runtime use."""

    def __init__(self, node, action_name: str = "/navigate_to_pose") -> None:
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose

        self.node = node
        self.action_name = action_name
        self._NavigateToPose = NavigateToPose
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._goal_handles = {}
        self._futures = []
        self._result_callbacks = {}

    def server_available(self) -> bool:
        return self._client.server_is_ready()

    def send_goal(self, waypoint, result_callback: Callable[[GoalResultCode, str], None]) -> GoalOutcome:
        goal = self._NavigateToPose.Goal()
        goal.pose.header.frame_id = waypoint.pose.frame_id
        goal.pose.pose.position.x = waypoint.pose.x
        goal.pose.pose.position.y = waypoint.pose.y
        goal.pose.pose.position.z = waypoint.pose.z
        goal.pose.pose.orientation.x = waypoint.pose.qx
        goal.pose.pose.orientation.y = waypoint.pose.qy
        goal.pose.pose.orientation.z = waypoint.pose.qz
        goal.pose.pose.orientation.w = waypoint.pose.qw
        future = self._client.send_goal_async(goal)
        self._futures.append(future)
        future.add_done_callback(lambda done: self._goal_response_cb(done, result_callback))
        return GoalOutcome(True, goal_uuid="", terminal_status=None, reason_code="GOAL_PENDING", detail="goal request sent")

    def cancel_goal(self, goal_uuid: str, timeout_sec: float) -> bool:
        del timeout_sec
        goal_handle = self._goal_handles.get(goal_uuid)
        if goal_handle is None:
            return False
        future = goal_handle.cancel_goal_async()
        self._futures.append(future)
        return True

    def _goal_response_cb(self, future, result_callback: Callable[[GoalResultCode, str], None]) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.get_logger().error(f"NavigateToPose goal response exception: {exc!r}")
            return
        if not goal_handle.accepted:
            result_callback(GoalResultCode.ABORTED, "goal rejected")
            return
        goal_uuid = bytes(goal_handle.goal_id.uuid).hex()
        self._goal_handles[goal_uuid] = goal_handle
        self._result_callbacks[goal_uuid] = result_callback
        result_future = goal_handle.get_result_async()
        self._futures.append(result_future)
        result_future.add_done_callback(lambda done: self._result_cb(goal_uuid, done))

    def _result_cb(self, goal_uuid: str, future) -> None:
        callback = self._result_callbacks.get(goal_uuid)
        if callback is None:
            return
        try:
            wrapped = future.result()
            status = GoalResultCode(int(wrapped.status))
        except ValueError:
            status = GoalResultCode.ABORTED
        except Exception as exc:
            callback(GoalResultCode.ABORTED, f"result exception: {exc!r}")
            return
        callback(status, status.name)
        self._result_callbacks.pop(goal_uuid, None)
        return False
