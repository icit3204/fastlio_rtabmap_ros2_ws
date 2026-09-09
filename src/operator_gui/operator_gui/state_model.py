"""Pure presentation state model for the Operator GUI.

This module contains no ROS node, publisher, action client, or widget access.
It is deliberately testable with small message-like objects.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Any, Optional


MISSION_NAMES = {
    0: 'IDLE', 1: 'RECEIVED', 2: 'VALIDATING', 3: 'PLANNING',
    4: 'NAVIGATING', 5: 'PAUSED', 6: 'CANCELLING', 7: 'CANCELLED',
    8: 'SUCCEEDED', 9: 'TEMPORARILY_BLOCKED', 10: 'BLOCKED',
    11: 'FAILED', 12: 'HELP_REQUIRED',
}

MISSION_BUTTON_STATES = {0, 1, 2, 3, 4, 5, 6, 9}


@dataclass
class _Source:
    value: Any = None
    received_at: Optional[float] = None

    def update(self, value: Any, now: Optional[float] = None) -> None:
        self.value = value
        self.received_at = time.monotonic() if now is None else float(now)

    def age(self, now: float) -> Optional[float]:
        if self.received_at is None:
            return None
        return max(0.0, now - self.received_at)


@dataclass(frozen=True)
class GuiView:
    mission_id: str = 'UNKNOWN'
    start_node: str = 'UNKNOWN'
    goal_node: str = 'UNKNOWN'
    current_edge: str = '—'
    current_goal: str = 'UNKNOWN'
    progress: str = 'UNKNOWN'
    mission_state: str = 'UNKNOWN'
    nav2_state: str = 'UNKNOWN'
    controller_valid: str = 'UNKNOWN'
    safe_speed: str = 'UNKNOWN'
    perception_valid: str = 'UNKNOWN'
    localization_valid: str = 'UNKNOWN'
    collision_validity: str = 'UNKNOWN'
    gate_state: str = 'UNKNOWN'
    gate_reason: str = 'UNKNOWN'
    fault_reason: str = 'NONE'
    route_ready: bool = False
    state_code: Optional[int] = None
    state_age: Optional[float] = None
    status_ages: dict[str, Optional[float]] = field(default_factory=dict)


def _bool_text(source: _Source, now: float, stale_sec: float = 1.0) -> str:
    if source.received_at is None:
        return 'UNKNOWN'
    age = source.age(now)
    if age is None or age > stale_sec:
        return 'STALE'
    return 'YES' if bool(source.value) else 'NO'


def goal_status_label(status: Optional[int]) -> str:
    return {
        0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'ACTIVE', 3: 'CANCELING',
        4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
    }.get(status, 'UNKNOWN')


class GuiStateModel:
    """Join authoritative messages without creating a safety authority."""

    def __init__(self) -> None:
        self.mission = _Source()
        self.route = _Source()
        self.controller_valid = _Source()
        self.perception_valid = _Source()
        self.localization_valid = _Source()
        self.safe_speed = _Source()
        self.gate = _Source()
        self.nav_status = _Source()

    def update_mission(self, msg: Any, now: Optional[float] = None) -> None:
        self.mission.update(msg, now)

    def update_route(self, msg: Any, now: Optional[float] = None) -> None:
        self.route.update(msg, now)

    def update_bool(self, name: str, value: bool, now: Optional[float] = None) -> None:
        getattr(self, name).update(bool(value), now)

    def update_safe_speed(self, value: float, now: Optional[float] = None) -> None:
        self.safe_speed.update(float(value), now)

    def update_gate(self, msg: Any, now: Optional[float] = None) -> None:
        self.gate.update(msg, now)

    def update_nav_status(self, status: Optional[int], now: Optional[float] = None) -> None:
        self.nav_status.update(status, now)

    def view(self, now: Optional[float] = None) -> GuiView:
        now = time.monotonic() if now is None else float(now)
        mission = self.mission.value
        route = self.route.value
        state_code = None if mission is None else int(getattr(mission, 'state', -1))
        mission_state = MISSION_NAMES.get(state_code, 'UNKNOWN')
        state_age = self.mission.age(now)
        if state_age is None:
            mission_state = 'UNKNOWN'
        elif state_age > 2.0:
            mission_state = 'STALE'

        route_ready = bool(
            mission is not None and route is not None
            and str(getattr(mission, 'mission_id', '')) == str(getattr(route, 'mission_id', ''))
            and str(getattr(mission, 'route_id', '')) == str(getattr(route, 'route_id', ''))
        )
        start_node = goal_node = current_edge = current_goal = 'UNKNOWN'
        if route_ready:
            node_ids = list(getattr(route, 'node_ids', []) or [])
            edge_ids = list(getattr(route, 'edge_ids', []) or [])
            poses = list(getattr(route, 'poses', []) or [])
            index = int(getattr(mission, 'current_waypoint_index', -1))
            start_node = str(node_ids[0]) if node_ids else '—'
            goal_node = str(node_ids[-1]) if node_ids else '—'
            current_edge = str(edge_ids[index]) if 0 <= index < len(edge_ids) else '—'
            if 0 <= index < len(poses):
                node = str(node_ids[index]) if index < len(node_ids) else ''
                pose = poses[index].pose.position
                if all(math.isfinite(float(v)) for v in (pose.x, pose.y)):
                    current_goal = f'{node} ({pose.x:.2f}, {pose.y:.2f})' if node else f'({pose.x:.2f}, {pose.y:.2f})'
                else:
                    current_goal = node or 'UNKNOWN'
            else:
                current_goal = 'UNKNOWN'

        if mission is not None and state_age is not None and state_age <= 2.0:
            completed = int(getattr(mission, 'completed_waypoint_count', 0))
            total = int(getattr(mission, 'total_waypoint_count', 0))
            progress = f'{completed} / {total} ({float(getattr(mission, "progress", 0.0)) * 100:.0f}%)'
            mission_id = str(getattr(mission, 'mission_id', '')) or 'UNKNOWN'
        else:
            progress = mission_id = 'UNKNOWN'

        nav_age = self.nav_status.age(now)
        nav2 = (
            goal_status_label(self.nav_status.value)
            if self.nav_status.received_at is not None and nav_age is not None and nav_age <= 1.0
            else 'UNKNOWN'
        )
        if mission_state in ('NAVIGATING', 'TEMPORARILY_BLOCKED') and nav2 == 'UNKNOWN':
            nav2 = 'ACTIVE'
        elif mission_state == 'PAUSED' and nav2 == 'UNKNOWN':
            nav2 = 'PAUSED'
        elif mission_state == 'CANCELLING' and nav2 == 'UNKNOWN':
            nav2 = 'CANCELING'

        gate_state, gate_reason = self._gate_text(now)
        fault = self._fault_text(mission, gate_reason)
        speed = 'UNKNOWN'
        speed_age = self.safe_speed.age(now)
        if self.safe_speed.received_at is not None:
            speed = 'STALE' if speed_age is None or speed_age > 0.5 else f'{self.safe_speed.value:.3f} m/s'

        return GuiView(
            mission_id=mission_id, start_node=start_node, goal_node=goal_node,
            current_edge=current_edge, current_goal=current_goal, progress=progress,
            mission_state=mission_state, nav2_state=nav2,
            controller_valid=_bool_text(self.controller_valid, now), safe_speed=speed,
            perception_valid=_bool_text(self.perception_valid, now),
            localization_valid=_bool_text(self.localization_valid, now),
            collision_validity=_bool_text(self.perception_valid, now),
            gate_state=gate_state, gate_reason=gate_reason, fault_reason=fault,
            route_ready=route_ready, state_code=state_code, state_age=state_age,
            status_ages={name: source.age(now) for name, source in self._sources().items()},
        )

    def _sources(self) -> dict[str, _Source]:
        return {
            'mission': self.mission, 'controller': self.controller_valid,
            'perception': self.perception_valid, 'localization': self.localization_valid,
            'safe_speed': self.safe_speed, 'gate': self.gate, 'nav2': self.nav_status,
        }

    def _gate_text(self, now: float) -> tuple[str, str]:
        if self.gate.received_at is None:
            return 'UNKNOWN', 'UNKNOWN'
        age = self.gate.age(now)
        if age is None or age > 1.0:
            return 'STALE', 'STALE'
        msg = self.gate.value
        values = {str(item.key): str(item.value) for item in getattr(msg, 'values', [])}
        state = values.get('state', '')
        reason = str(getattr(msg, 'message', '') or '')
        if not state and reason == 'ARMED_COMMAND':
            state = 'ARMED'
        return state or 'UNKNOWN', reason or 'NONE'

    @staticmethod
    def _fault_text(mission: Any, gate_reason: str) -> str:
        if gate_reason not in ('UNKNOWN', 'NONE', 'ARMED_COMMAND'):
            return gate_reason
        reason = str(getattr(mission, 'reason_code', '') or '') if mission is not None else ''
        return reason or 'NONE'


def button_enabled(view: GuiView, button: str) -> bool:
    """Mirror the audited Mission Manager legal-state contract."""
    if view.state_code is None or not view.route_ready:
        return False
    if button == 'start':
        return view.state_code == 1
    if button == 'pause':
        return view.state_code in (4, 9)
    if button == 'resume':
        return view.state_code == 5
    if button == 'cancel':
        return view.state_code in (1, 3, 4, 5, 6, 9)
    return False
