"""Mission sequencing core with no ROS transport dependency."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import traceback
from typing import Callable, List, Optional

from .route_contract import NormalizedRouteMission, ValidationResult, validate_route_mission


class MissionStateCode(IntEnum):
    IDLE = 0
    VALIDATING = 1
    READY = 2
    RUNNING = 3
    PAUSED = 4
    CANCELING = 5
    SUCCEEDED = 6
    FAILED = 7
    BLOCKED = 8
    REJECTED = 9


TERMINAL_STATES = {
    MissionStateCode.IDLE,
    MissionStateCode.SUCCEEDED,
    MissionStateCode.FAILED,
    MissionStateCode.REJECTED,
}


VALID_TRANSITIONS = {
    MissionStateCode.IDLE: {MissionStateCode.VALIDATING},
    MissionStateCode.VALIDATING: {MissionStateCode.READY, MissionStateCode.REJECTED, MissionStateCode.FAILED},
    MissionStateCode.READY: {MissionStateCode.RUNNING, MissionStateCode.CANCELING, MissionStateCode.SUCCEEDED, MissionStateCode.FAILED},
    MissionStateCode.RUNNING: {MissionStateCode.RUNNING, MissionStateCode.PAUSED, MissionStateCode.CANCELING, MissionStateCode.SUCCEEDED, MissionStateCode.FAILED},
    MissionStateCode.PAUSED: {MissionStateCode.RUNNING, MissionStateCode.CANCELING},
    MissionStateCode.CANCELING: {MissionStateCode.IDLE, MissionStateCode.FAILED},
    MissionStateCode.SUCCEEDED: {MissionStateCode.IDLE},
    MissionStateCode.FAILED: {MissionStateCode.IDLE},
    MissionStateCode.BLOCKED: {MissionStateCode.CANCELING, MissionStateCode.FAILED},
    MissionStateCode.REJECTED: {MissionStateCode.IDLE},
}


class GoalResultCode(IntEnum):
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6


@dataclass(frozen=True)
class GoalOutcome:
    accepted: bool
    goal_uuid: str = ""
    terminal_status: Optional[GoalResultCode] = None
    reason_code: str = ""
    detail: str = ""


@dataclass(frozen=True)
class MissionSnapshot:
    state: MissionStateCode
    mission_id: str = ""
    route_id: str = ""
    current_waypoint_index: int = 0
    completed_waypoint_count: int = 0
    total_waypoint_count: int = 0
    progress: float = 0.0
    active_goal_uuid: str = ""
    reason_code: str = ""
    detail: str = ""


class MissionGoalExecutor:
    """Interface implemented by real and fake NavigateToPose executors."""

    def send_goal(self, waypoint, result_callback: Callable[[GoalResultCode, str], None]) -> GoalOutcome:
        raise NotImplementedError

    def cancel_goal(self, goal_uuid: str, timeout_sec: float) -> bool:
        raise NotImplementedError

    def server_available(self) -> bool:
        return True


class MissionStateMachine:
    def __init__(
        self,
        executor: MissionGoalExecutor,
        *,
        state_callback: Optional[Callable[[MissionSnapshot], None]] = None,
        min_waypoint_separation_m: float = 0.05,
        cancel_timeout_sec: float = 2.0,
    ) -> None:
        self.executor = executor
        self._state_callback = state_callback
        self.min_waypoint_separation_m = min_waypoint_separation_m
        self.cancel_timeout_sec = cancel_timeout_sec
        self.state = MissionStateCode.IDLE
        self.mission: Optional[NormalizedRouteMission] = None
        self.current_waypoint_index = 0
        self.completed_waypoint_count = 0
        self.active_goal_uuid = ""
        self.reason_code = ""
        self.detail = ""
        self.transition_errors: List[str] = []
        self.snapshots: List[MissionSnapshot] = []
        self._canceling = False
        self._terminal_written = False
        self._publish()

    @property
    def total_waypoint_count(self) -> int:
        return len(self.mission.waypoints) if self.mission is not None else 0

    def _progress(self) -> float:
        total = self.total_waypoint_count
        if total == 0:
            return 0.0
        return min(1.0, self.completed_waypoint_count / total)

    def snapshot(self) -> MissionSnapshot:
        return MissionSnapshot(
            state=self.state,
            mission_id=self.mission.mission_id if self.mission else "",
            route_id=self.mission.route_id if self.mission else "",
            current_waypoint_index=self.current_waypoint_index,
            completed_waypoint_count=self.completed_waypoint_count,
            total_waypoint_count=self.total_waypoint_count,
            progress=self._progress(),
            active_goal_uuid=self.active_goal_uuid,
            reason_code=self.reason_code,
            detail=self.detail,
        )

    def _publish(self) -> None:
        snap = self.snapshot()
        self.snapshots.append(snap)
        if self._state_callback is not None:
            self._state_callback(snap)

    def validate_transition(self, next_state: MissionStateCode) -> bool:
        if next_state not in VALID_TRANSITIONS[self.state]:
            self.transition_errors.append(f"invalid transition {self.state.name}->{next_state.name}")
            return False
        return True

    def _transition(self, next_state: MissionStateCode, reason_code: str = "", detail: str = "") -> bool:
        if not self.validate_transition(next_state):
            self.reason_code = "ILLEGAL_TRANSITION"
            self.detail = f"{self.state.name}->{next_state.name}"
            self._publish()
            return False
        self.state = next_state
        self.reason_code = reason_code
        self.detail = detail
        self._publish()
        return True

    def submit_mission(self, msg) -> ValidationResult:
        if self.state not in TERMINAL_STATES:
            self._publish_rejected("MISSION_ALREADY_ACTIVE", "only one mission may be active")
            return ValidationResult(False, "MISSION_ALREADY_ACTIVE", "only one mission may be active")
        if self.state != MissionStateCode.IDLE:
            self._transition(MissionStateCode.IDLE)
        self.mission = None
        self.current_waypoint_index = 0
        self.completed_waypoint_count = 0
        self.active_goal_uuid = ""
        self._transition(MissionStateCode.VALIDATING)
        result = validate_route_mission(msg, min_waypoint_separation_m=self.min_waypoint_separation_m)
        if not result.valid:
            self._transition(MissionStateCode.REJECTED, result.reason_code, result.detail)
            return result
        self.mission = result.mission
        self._transition(MissionStateCode.READY)
        self._dispatch_current_waypoint()
        return result

    def _publish_rejected(self, reason_code: str, detail: str) -> None:
        old_state = self.state
        self.state = MissionStateCode.REJECTED
        self.reason_code = reason_code
        self.detail = detail
        self._publish()
        self.state = old_state

    def _dispatch_current_waypoint(self) -> None:
        if self.mission is None:
            self._transition(MissionStateCode.FAILED, "NO_ACTIVE_MISSION", "cannot dispatch without a mission")
            return
        if self.completed_waypoint_count > self.total_waypoint_count:
            self._transition(MissionStateCode.FAILED, "COMPLETED_COUNT_OVERFLOW", "completed waypoint count exceeded total")
            return
        if self.current_waypoint_index >= self.total_waypoint_count:
            self.active_goal_uuid = ""
            self._transition(MissionStateCode.SUCCEEDED, "MISSION_SUCCEEDED", "all waypoints completed")
            return
        if self.state == MissionStateCode.READY:
            if not self._transition(MissionStateCode.RUNNING):
                return
        elif self.state != MissionStateCode.RUNNING:
            self._transition(MissionStateCode.FAILED, "DISPATCH_FROM_INVALID_STATE", self.state.name)
            return
        try:
            waypoint = self.mission.waypoints[self.current_waypoint_index]
            outcome = self.executor.send_goal(waypoint, self.on_goal_result)
        except Exception as exc:
            self._transition(
                MissionStateCode.FAILED,
                "EXECUTOR_EXCEPTION",
                f"{type(exc).__name__}: {exc}\n{traceback.format_exc(limit=6)}",
            )
            return
        if not outcome.accepted:
            self._transition(MissionStateCode.FAILED, outcome.reason_code or "GOAL_REJECTED", outcome.detail or "goal rejected")
            return
        self.active_goal_uuid = outcome.goal_uuid
        self._publish()
        if outcome.terminal_status is not None:
            self.on_goal_result(outcome.terminal_status, outcome.detail)

    def on_goal_result(self, status: GoalResultCode, detail: str = "") -> None:
        if self.state != MissionStateCode.RUNNING:
            return
        if status == GoalResultCode.SUCCEEDED:
            self.completed_waypoint_count += 1
            self.current_waypoint_index += 1
            self.active_goal_uuid = ""
            self._publish()
            self._dispatch_current_waypoint()
            return
        self._transition(MissionStateCode.FAILED, f"GOAL_{status.name}", detail or status.name)

    def cancel(self) -> bool:
        if self.state == MissionStateCode.IDLE:
            return True
        if self.state not in {MissionStateCode.READY, MissionStateCode.RUNNING, MissionStateCode.PAUSED}:
            return False
        self._transition(MissionStateCode.CANCELING, "MISSION_CANCEL_REQUESTED", "cancel requested")
        if self.active_goal_uuid:
            self.executor.cancel_goal(self.active_goal_uuid, self.cancel_timeout_sec)
        self.active_goal_uuid = ""
        self._transition(MissionStateCode.IDLE, "MISSION_CANCELED", "mission canceled")
        return True

    def pause(self) -> bool:
        if self.state == MissionStateCode.PAUSED:
            return True
        if self.state != MissionStateCode.RUNNING:
            return False
        if self.active_goal_uuid:
            self.executor.cancel_goal(self.active_goal_uuid, self.cancel_timeout_sec)
        self.active_goal_uuid = ""
        return self._transition(MissionStateCode.PAUSED, "MISSION_PAUSED", "active goal canceled; waypoint retained")

    def resume(self) -> bool:
        if self.state != MissionStateCode.PAUSED:
            return self.state == MissionStateCode.RUNNING
        if not self._transition(MissionStateCode.RUNNING, "MISSION_RESUMED", "current waypoint will be re-sent"):
            return False
        self._dispatch_current_waypoint()
        return True
