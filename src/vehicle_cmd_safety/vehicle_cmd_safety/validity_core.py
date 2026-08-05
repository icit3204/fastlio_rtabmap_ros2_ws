"""Deterministic core for Collision Monitor observation validity."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class ValidityConfig:
    source_type: str = "scan"
    source_topic: str = "/phase4/synthetic_scan"
    expected_frame: str = "base_footprint"
    source_freshness_sec: float = 0.50
    recovery_stability_sec: float = 0.50
    heartbeat_hz: float = 20.0
    collision_monitor_node_name: str = "collision_monitor"
    expected_observation_source_name: str = "scan"
    expected_observation_source_type: str = "scan"
    expected_observation_source_topic: str = "/phase4/synthetic_scan"


@dataclass
class ObservationState:
    stamp: Optional[float] = None
    frame_id: str = ""
    valid_structure: bool = False


@dataclass
class CollisionMonitorState:
    lifecycle_reachable: bool = False
    lifecycle_active: bool = False
    config_reachable: bool = False
    configured_source_present: bool = False
    configured_source_type_matches: bool = False
    configured_source_topic_matches: bool = False
    output_publisher_count: int = 1


@dataclass
class ValidityStatus:
    valid: bool
    reason_code: str
    diagnostics: Dict[str, str] = field(default_factory=dict)


class CollisionMonitorValidityCore:
    def __init__(self, config: ValidityConfig) -> None:
        self.config = config
        self.observation = ObservationState()
        self.cm_state = CollisionMonitorState()
        self.valid_publisher_count = 1
        self.healthy_since: Optional[float] = None

    def set_observation(self, frame_id: str, valid_structure: bool, now: float) -> None:
        self.observation = ObservationState(stamp=now, frame_id=frame_id, valid_structure=valid_structure)

    def set_collision_monitor_state(self, state: CollisionMonitorState) -> None:
        self.cm_state = state

    def set_valid_publisher_count(self, count: int) -> None:
        self.valid_publisher_count = count

    def tick(self, now: float) -> ValidityStatus:
        reason = self._health_reason(now)
        healthy = reason == "HEALTHY"
        if healthy:
            if self.healthy_since is None:
                self.healthy_since = now
            if now - self.healthy_since >= self.config.recovery_stability_sec:
                return self._status(True, "VALID", now)
            return self._status(False, "RECOVERY_STABILITY_WAIT", now)
        self.healthy_since = None
        return self._status(False, reason, now)

    def _health_reason(self, now: float) -> str:
        if self.config.source_type not in {"scan", "pointcloud"}:
            return "INVALID_SOURCE_TYPE"
        if self.config.source_type != self.config.expected_observation_source_type:
            return "SOURCE_TYPE_MISMATCH"
        if self.config.source_topic != self.config.expected_observation_source_topic:
            return "SOURCE_TOPIC_MISMATCH"
        if self.valid_publisher_count != 1:
            return "COLLISION_VALID_AUTHORITY_CONFLICT"
        if self.observation.stamp is None:
            return "SOURCE_NEVER_RECEIVED"
        if self.observation.frame_id != self.config.expected_frame:
            return "SOURCE_FRAME_MISMATCH"
        if not self.observation.valid_structure:
            return "SOURCE_MESSAGE_INVALID"
        if not self.cm_state.lifecycle_reachable:
            return "COLLISION_MONITOR_UNREACHABLE"
        if not self.cm_state.lifecycle_active:
            return "COLLISION_MONITOR_NOT_ACTIVE"
        if not self.cm_state.config_reachable:
            return "COLLISION_MONITOR_CONFIG_UNREACHABLE"
        if not self.cm_state.configured_source_present:
            return "CONFIGURED_SOURCE_MISSING"
        if not self.cm_state.configured_source_type_matches:
            return "CONFIGURED_SOURCE_TYPE_MISMATCH"
        if not self.cm_state.configured_source_topic_matches:
            return "CONFIGURED_SOURCE_TOPIC_MISMATCH"
        if now - self.observation.stamp > self.config.source_freshness_sec:
            return "SOURCE_STALE"
        return "HEALTHY"

    def _status(self, valid: bool, reason: str, now: float) -> ValidityStatus:
        age = "none"
        if self.observation.stamp is not None:
            age = f"{max(0.0, now - self.observation.stamp):.6f}"
        stable = 0.0
        if self.healthy_since is not None:
            stable = max(0.0, now - self.healthy_since)
        diagnostics = {
            "state": "VALID" if valid else "INVALID",
            "reason_code": reason,
            "source_type": self.config.source_type,
            "source_topic": self.config.source_topic,
            "expected_frame": self.config.expected_frame,
            "observed_frame": self.observation.frame_id,
            "source_age_sec": age,
            "healthy_stable_sec": f"{stable:.6f}",
            "lifecycle_reachable": str(self.cm_state.lifecycle_reachable).lower(),
            "lifecycle_active": str(self.cm_state.lifecycle_active).lower(),
            "configured_source_present": str(self.cm_state.configured_source_present).lower(),
            "configured_source_type_matches": str(self.cm_state.configured_source_type_matches).lower(),
            "configured_source_topic_matches": str(self.cm_state.configured_source_topic_matches).lower(),
            "collision_valid_publisher_count": str(self.valid_publisher_count),
        }
        return ValidityStatus(valid=valid, reason_code=reason, diagnostics=diagnostics)
