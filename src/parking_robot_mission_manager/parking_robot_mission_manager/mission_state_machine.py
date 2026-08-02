"""Authority-aligned Mission Manager sequencing core with no ROS dependency."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import traceback
from typing import Callable, List, Optional

from .route_contract import NormalizedRouteMission, ValidationResult, validate_route_mission


class MissionStateCode(IntEnum):
    IDLE = 0
    RECEIVED = 1
    VALIDATING = 2
    PLANNING = 3
    NAVIGATING = 4
    PAUSED = 5
    CANCELLING = 6
    CANCELLED = 7
    SUCCEEDED = 8
    TEMPORARILY_BLOCKED = 9
    BLOCKED = 10
    FAILED = 11
    HELP_REQUIRED = 12


TERMINAL_STATES = {
    MissionStateCode.IDLE,
    MissionStateCode.CANCELLED,
    MissionStateCode.SUCCEEDED,
    MissionStateCode.BLOCKED,
    MissionStateCode.FAILED,
    MissionStateCode.HELP_REQUIRED,
}


VALID_TRANSITIONS = {
    MissionStateCode.IDLE: {MissionStateCode.RECEIVED},
    MissionStateCode.RECEIVED: {MissionStateCode.VALIDATING, MissionStateCode.FAILED},
    MissionStateCode.VALIDATING: {MissionStateCode.PLANNING, MissionStateCode.FAILED},
    MissionStateCode.PLANNING: {MissionStateCode.NAVIGATING, MissionStateCode.SUCCEEDED, MissionStateCode.FAILED},
    MissionStateCode.NAVIGATING: {
        MissionStateCode.PLANNING,
        MissionStateCode.PAUSED,
        MissionStateCode.CANCELLING,
        MissionStateCode.SUCCEEDED,
        MissionStateCode.TEMPORARILY_BLOCKED,
        MissionStateCode.BLOCKED,
        MissionStateCode.FAILED,
        MissionStateCode.HELP_REQUIRED,
    },
    MissionStateCode.PAUSED: {MissionStateCode.PLANNING, MissionStateCode.CANCELLING},
    MissionStateCode.CANCELLING: {MissionStateCode.CANCELLED, MissionStateCode.FAILED},
    MissionStateCode.CANCELLED: {MissionStateCode.IDLE, MissionStateCode.RECEIVED},
    MissionStateCode.SUCCEEDED: {MissionStateCode.IDLE, MissionStateCode.RECEIVED},
    MissionStateCode.TEMPORARILY_BLOCKED: {MissionStateCode.PLANNING, MissionStateCode.CANCELLING, MissionStateCode.BLOCKED, MissionStateCode.FAILED},
    MissionStateCode.BLOCKED: {MissionStateCode.IDLE, MissionStateCode.RECEIVED},
    MissionStateCode.FAILED: {MissionStateCode.IDLE, MissionStateCode.RECEIVED},
    MissionStateCode.HELP_REQUIRED: {MissionStateCode.IDLE, MissionStateCode.RECEIVED},
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
    def send_goal(self, pose, result_callback: Callable[[GoalResultCode, str], None]) -> GoalOutcome:
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
        expected_topology_version: str,
        state_callback: Optional[Callable[[MissionSnapshot], None]] = None,
        goal_xy_tolerance_m: float = 0.25,
        waypoint_separation_margin_m: float = 0.05,
        min_waypoint_separation_m: float = 0.55,
        cancel_timeout_sec: float = 2.0,
    ) -> None:
        self.executor = executor
        self.expected_topology_version = expected_topology_version
        self._state_callback = state_callback
        self.goal_xy_tolerance_m = goal_xy_tolerance_m
        self.waypoint_separation_margin_m = waypoint_separation_margin_m
        self.min_waypoint_separation_m = min_waypoint_separation_m
        self.cancel_timeout_sec = cancel_timeout_sec
        self.state = MissionStateCode.IDLE
        self.raw_mission = None
        self.mission: Optional[NormalizedRouteMission] = None
        self.current_waypoint_index = 0
        self.completed_waypoint_count = 0
        self.active_goal_uuid = ""
        self.reason_code = ""
        self.detail = ""
        self.transition_errors: List[str] = []
        self.snapshots: List[MissionSnapshot] = []
        self._publish()

    @property
    def total_waypoint_count(self) -> int:
        if self.mission is not None:
            return len(self.mission.poses)
        if self.raw_mission is not None:
            return len(getattr(self.raw_mission, "poses", []))
        return 0

    def _progress(self) -> float:
        total = self.total_waypoint_count
        if total == 0:
            return 0.0
        return min(1.0, self.completed_waypoint_count / total)

    def snapshot(self) -> MissionSnapshot:
        mission_id = ""
        route_id = ""
        if self.mission is not None:
            mission_id = self.mission.mission_id
            route_id = self.mission.route_id
        elif self.raw_mission is not None:
            mission_id = str(getattr(self.raw_mission, "mission_id", ""))
            route_id = str(getattr(self.raw_mission, "route_id", ""))
        return MissionSnapshot(
            state=self.state,
            mission_id=mission_id,
            route_id=route_id,
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

    def receive_mission(self, msg) -> ValidationResult:
        if self.state not in TERMINAL_STATES:
            self.reason_code = "MISSION_ALREADY_ACTIVE"
            self.detail = "only one mission may be stored or active"
            self._publish()
            return ValidationResult(False, self.reason_code, self.detail)
        if self.state != MissionStateCode.IDLE:
            self._transition(MissionStateCode.IDLE)
        self.raw_mission = msg
        self.mission = None
        self.current_waypoint_index = 0
        self.completed_waypoint_count = 0
        self.active_goal_uuid = ""
        self._transition(MissionStateCode.RECEIVED, "MISSION_RECEIVED", "mission stored; awaiting start")
        return ValidationResult(True, "RECEIVED", "mission received")

    def start(self) -> ValidationResult:
        if self.state == MissionStateCode.NAVIGATING:
            return ValidationResult(False, "MISSION_ALREADY_NAVIGATING", "mission is already navigating")
        if self.state != MissionStateCode.RECEIVED:
            return ValidationResult(False, "NO_RECEIVED_MISSION", "start requires a received mission")
        self._transition(MissionStateCode.VALIDATING)
        try:
            result = validate_route_mission(
                self.raw_mission,
                expected_topology_version=self.expected_topology_version,
                goal_xy_tolerance_m=self.goal_xy_tolerance_m,
                waypoint_separation_margin_m=self.waypoint_separation_margin_m,
                min_waypoint_separation_m=self.min_waypoint_separation_m,
            )
        except Exception as exc:
            self._transition(MissionStateCode.FAILED, "VALIDATION_EXCEPTION", f"{type(exc).__name__}: {exc}")
            return ValidationResult(False, "VALIDATION_EXCEPTION", str(exc))
        if not result.valid:
            self._transition(MissionStateCode.FAILED, result.reason_code, result.detail)
            return result
        self.mission = result.mission
        self._transition(MissionStateCode.PLANNING)
        self._dispatch_current_waypoint()
        return result

    def _dispatch_current_waypoint(self) -> None:
        if self.mission is None:
            self._transition(MissionStateCode.FAILED, "NO_ACTIVE_MISSION", "cannot dispatch without a validated mission")
            return
        if self.completed_waypoint_count > self.total_waypoint_count:
            self._transition(MissionStateCode.FAILED, "COMPLETED_COUNT_OVERFLOW", "completed waypoint count exceeded total")
            return
        if self.current_waypoint_index >= self.total_waypoint_count:
            self.active_goal_uuid = ""
            self._transition(MissionStateCode.SUCCEEDED, "MISSION_SUCCEEDED", "all waypoints completed")
            return
        if self.state == MissionStateCode.PLANNING:
            if not self._transition(MissionStateCode.NAVIGATING):
                return
        elif self.state != MissionStateCode.NAVIGATING:
            self._transition(MissionStateCode.FAILED, "DISPATCH_FROM_INVALID_STATE", self.state.name)
            return
        try:
            pose = self.mission.poses[self.current_waypoint_index]
            outcome = self.executor.send_goal(pose, self.on_goal_result)
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
        if self.state != MissionStateCode.NAVIGATING:
            return
        if status == GoalResultCode.SUCCEEDED:
            self.completed_waypoint_count += 1
            self.current_waypoint_index += 1
            self.active_goal_uuid = ""
            self._publish()
            self._transition(MissionStateCode.PLANNING, "WAYPOINT_SUCCEEDED", detail or "waypoint succeeded")
            self._dispatch_current_waypoint()
            return
        self._transition(MissionStateCode.FAILED, f"GOAL_{status.name}", detail or status.name)

    def cancel(self) -> bool:
        if self.state == MissionStateCode.CANCELLED:
            return True
        if self.state not in {MissionStateCode.RECEIVED, MissionStateCode.PLANNING, MissionStateCode.NAVIGATING, MissionStateCode.PAUSED}:
            return False
        self._transition(MissionStateCode.CANCELLING, "MISSION_CANCEL_REQUESTED", "cancel requested")
        if self.active_goal_uuid:
            acknowledged = self.executor.cancel_goal(self.active_goal_uuid, self.cancel_timeout_sec)
            if not acknowledged:
                self._transition(MissionStateCode.FAILED, "CANCEL_ACK_TIMEOUT", "active goal cancellation was not acknowledged")
                return False
        self.active_goal_uuid = ""
        self._transition(MissionStateCode.CANCELLED, "MISSION_CANCELLED", "mission cancelled")
        return True

    def pause(self) -> bool:
        if self.state == MissionStateCode.PAUSED:
            return True
        if self.state != MissionStateCode.NAVIGATING:
            return False
        if self.active_goal_uuid:
            acknowledged = self.executor.cancel_goal(self.active_goal_uuid, self.cancel_timeout_sec)
            if not acknowledged:
                self._transition(MissionStateCode.FAILED, "PAUSE_CANCEL_ACK_TIMEOUT", "active goal cancellation was not acknowledged")
                return False
        self.active_goal_uuid = ""
        return self._transition(MissionStateCode.PAUSED, "MISSION_PAUSED", "active goal cancelled; waypoint retained")

    def resume(self) -> bool:
        if self.state == MissionStateCode.NAVIGATING:
            return True
        if self.state != MissionStateCode.PAUSED:
            return False
        if not self._transition(MissionStateCode.PLANNING, "MISSION_RESUMED", "current waypoint will be re-sent"):
            return False
        self._dispatch_current_waypoint()
        return True
