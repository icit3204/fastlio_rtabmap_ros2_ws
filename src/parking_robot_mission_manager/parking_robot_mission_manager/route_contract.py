"""ROS-independent validation for typed route missions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence


MAP_FRAME = "map"
DEFAULT_QUATERNION_TOLERANCE = 1.0e-3
DEFAULT_MIN_WAYPOINT_SEPARATION_M = 0.05


@dataclass(frozen=True)
class NormalizedPose:
    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class NormalizedWaypoint:
    waypoint_id: str
    pose: NormalizedPose


@dataclass(frozen=True)
class NormalizedRouteMission:
    mission_id: str
    route_id: str
    route_version: str
    direction_id: str
    frame_id: str
    waypoints: Sequence[NormalizedWaypoint]


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    reason_code: str
    detail: str
    mission: Optional[NormalizedRouteMission] = None


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _frame(value: str, inherited: str) -> str:
    return value if value else inherited


def _distance_xy(a: NormalizedPose, b: NormalizedPose) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _pose_from_msg(pose_stamped, mission_frame: str) -> NormalizedPose:
    frame_id = _frame(str(pose_stamped.header.frame_id), mission_frame)
    pose = pose_stamped.pose
    return NormalizedPose(
        frame_id=frame_id,
        x=float(pose.position.x),
        y=float(pose.position.y),
        z=float(pose.position.z),
        qx=float(pose.orientation.x),
        qy=float(pose.orientation.y),
        qz=float(pose.orientation.z),
        qw=float(pose.orientation.w),
    )


def validate_route_mission(
    msg,
    *,
    min_waypoint_separation_m: float = DEFAULT_MIN_WAYPOINT_SEPARATION_M,
    quaternion_tolerance: float = DEFAULT_QUATERNION_TOLERANCE,
) -> ValidationResult:
    """Validate a typed RouteMission message without altering its waypoint list.

    The default waypoint separation is intentionally below the accepted Phase 2
    0.25 m goal tolerance. It catches accidental duplicate route points while
    avoiding site-specific route spacing rules.
    """

    mission_frame = str(getattr(msg.header, "frame_id", ""))
    if mission_frame != MAP_FRAME:
        return ValidationResult(False, "MISSION_FRAME_NOT_MAP", "RouteMission header.frame_id must be map")

    mission_id = str(getattr(msg, "mission_id", ""))
    route_id = str(getattr(msg, "route_id", ""))
    route_version = str(getattr(msg, "route_version", ""))
    direction_id = str(getattr(msg, "direction_id", ""))
    if not mission_id:
        return ValidationResult(False, "EMPTY_MISSION_ID", "mission_id must be nonempty")
    if not route_id:
        return ValidationResult(False, "EMPTY_ROUTE_ID", "route_id must be nonempty")
    if not route_version:
        return ValidationResult(False, "EMPTY_ROUTE_VERSION", "route_version must be nonempty")
    if not direction_id:
        return ValidationResult(False, "EMPTY_DIRECTION_ID", "direction_id must be nonempty")

    raw_waypoints = list(getattr(msg, "waypoints", []))
    if not raw_waypoints:
        return ValidationResult(False, "ZERO_WAYPOINTS", "at least one waypoint is required")

    if min_waypoint_separation_m < 0.0:
        return ValidationResult(False, "INVALID_SEPARATION_CONFIG", "min waypoint separation must be nonnegative")

    waypoint_ids = set()
    normalized: List[NormalizedWaypoint] = []
    previous_pose: Optional[NormalizedPose] = None
    for index, waypoint in enumerate(raw_waypoints):
        waypoint_id = str(getattr(waypoint, "waypoint_id", ""))
        if not waypoint_id:
            return ValidationResult(False, "EMPTY_WAYPOINT_ID", f"waypoint {index} has empty waypoint_id")
        if waypoint_id in waypoint_ids:
            return ValidationResult(False, "DUPLICATE_WAYPOINT_ID", f"duplicate waypoint_id {waypoint_id}")
        waypoint_ids.add(waypoint_id)

        pose = _pose_from_msg(waypoint.pose, mission_frame)
        if pose.frame_id != MAP_FRAME:
            return ValidationResult(False, "WAYPOINT_FRAME_NOT_MAP", f"waypoint {waypoint_id} frame must be map")
        if not _finite([pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw]):
            return ValidationResult(False, "NONFINITE_POSE", f"waypoint {waypoint_id} contains nonfinite pose data")
        norm = math.sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw)
        if norm == 0.0:
            return ValidationResult(False, "ZERO_QUATERNION", f"waypoint {waypoint_id} has zero quaternion")
        if abs(norm - 1.0) > quaternion_tolerance:
            return ValidationResult(False, "QUATERNION_NOT_NORMALIZED", f"waypoint {waypoint_id} quaternion norm {norm:.6f}")
        if previous_pose is not None and _distance_xy(previous_pose, pose) < min_waypoint_separation_m:
            return ValidationResult(
                False,
                "CONSECUTIVE_WAYPOINTS_TOO_CLOSE",
                f"waypoint {waypoint_id} is closer than {min_waypoint_separation_m:.3f} m to previous waypoint",
            )
        normalized.append(NormalizedWaypoint(waypoint_id=waypoint_id, pose=pose))
        previous_pose = pose

    mission = NormalizedRouteMission(
        mission_id=mission_id,
        route_id=route_id,
        route_version=route_version,
        direction_id=direction_id,
        frame_id=mission_frame,
        waypoints=tuple(normalized),
    )
    return ValidationResult(True, "VALID", "route mission is valid", mission)
