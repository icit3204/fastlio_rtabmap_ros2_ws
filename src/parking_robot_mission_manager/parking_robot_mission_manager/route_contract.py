"""ROS-independent validation for authority-aligned RouteMission messages."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence


MAP_FRAME = "map"
DEFAULT_GOAL_XY_TOLERANCE_M = 0.25
DEFAULT_WAYPOINT_SEPARATION_MARGIN_M = 0.05
DEFAULT_MIN_WAYPOINT_SEPARATION_M = (
    2.0 * DEFAULT_GOAL_XY_TOLERANCE_M + DEFAULT_WAYPOINT_SEPARATION_MARGIN_M
)
DEFAULT_QUATERNION_TOLERANCE = 1.0e-3


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
class NormalizedRouteMission:
    mission_id: str
    route_id: str
    topology_version: str
    frame_id: str
    node_ids: Sequence[str]
    edge_ids: Sequence[str]
    edge_directions: Sequence[int]
    poses: Sequence[NormalizedPose]


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    reason_code: str
    detail: str
    mission: Optional[NormalizedRouteMission] = None


def configured_min_waypoint_separation_m(
    goal_xy_tolerance_m: float = DEFAULT_GOAL_XY_TOLERANCE_M,
    waypoint_separation_margin_m: float = DEFAULT_WAYPOINT_SEPARATION_MARGIN_M,
) -> float:
    return 2.0 * goal_xy_tolerance_m + waypoint_separation_margin_m


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
    expected_topology_version: str,
    goal_xy_tolerance_m: float = DEFAULT_GOAL_XY_TOLERANCE_M,
    waypoint_separation_margin_m: float = DEFAULT_WAYPOINT_SEPARATION_MARGIN_M,
    min_waypoint_separation_m: Optional[float] = None,
    quaternion_tolerance: float = DEFAULT_QUATERNION_TOLERANCE,
) -> ValidationResult:
    """Validate the public RouteMission contract without repairing arrays."""

    mission_frame = str(getattr(msg.header, "frame_id", ""))
    if mission_frame != MAP_FRAME:
        return ValidationResult(False, "MISSION_FRAME_NOT_MAP", "RouteMission header.frame_id must be map")

    mission_id = str(getattr(msg, "mission_id", ""))
    route_id = str(getattr(msg, "route_id", ""))
    topology_version = str(getattr(msg, "topology_version", ""))
    if not mission_id:
        return ValidationResult(False, "EMPTY_MISSION_ID", "mission_id must be nonempty")
    if not route_id:
        return ValidationResult(False, "EMPTY_ROUTE_ID", "route_id must be nonempty")
    if not topology_version:
        return ValidationResult(False, "EMPTY_TOPOLOGY_VERSION", "topology_version must be nonempty")
    if topology_version != expected_topology_version:
        return ValidationResult(
            False,
            "TOPOLOGY_VERSION_MISMATCH",
            f"mission topology_version {topology_version!r} does not match expected {expected_topology_version!r}",
        )

    node_ids = [str(value) for value in getattr(msg, "node_ids", [])]
    edge_ids = [str(value) for value in getattr(msg, "edge_ids", [])]
    edge_directions = [int(value) for value in getattr(msg, "edge_directions", [])]
    raw_poses = list(getattr(msg, "poses", []))
    pose_count = len(raw_poses)
    if pose_count == 0:
        return ValidationResult(False, "ZERO_POSES", "at least one pose is required")
    if len(node_ids) != pose_count:
        return ValidationResult(False, "NODE_POSE_LENGTH_MISMATCH", "node_ids size must equal poses size")
    expected_edge_count = max(pose_count - 1, 0)
    if len(edge_ids) != expected_edge_count:
        return ValidationResult(False, "EDGE_COUNT_MISMATCH", "edge_ids size must equal max(poses size - 1, 0)")
    if len(edge_directions) != len(edge_ids):
        return ValidationResult(False, "DIRECTION_COUNT_MISMATCH", "edge_directions size must equal edge_ids size")
    for index, node_id in enumerate(node_ids):
        if not node_id:
            return ValidationResult(False, "EMPTY_NODE_ID", f"node_ids[{index}] is empty")
    for index, edge_id in enumerate(edge_ids):
        if not edge_id:
            return ValidationResult(False, "EMPTY_EDGE_ID", f"edge_ids[{index}] is empty")
    for index, direction in enumerate(edge_directions):
        if direction not in (-1, 0, 1):
            return ValidationResult(False, "INVALID_EDGE_DIRECTION", f"edge_directions[{index}] must be -1, 0 or 1")

    if min_waypoint_separation_m is None:
        min_waypoint_separation_m = configured_min_waypoint_separation_m(
            goal_xy_tolerance_m, waypoint_separation_margin_m
        )
    if min_waypoint_separation_m < 0.0:
        return ValidationResult(False, "INVALID_SEPARATION_CONFIG", "min waypoint separation must be nonnegative")

    poses = []
    previous_pose: Optional[NormalizedPose] = None
    for index, pose_stamped in enumerate(raw_poses):
        pose = _pose_from_msg(pose_stamped, mission_frame)
        if pose.frame_id != MAP_FRAME:
            return ValidationResult(False, "POSE_FRAME_NOT_MAP", f"poses[{index}] frame must be map")
        if not _finite([pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw]):
            return ValidationResult(False, "NONFINITE_POSE", f"poses[{index}] contains nonfinite pose data")
        norm = math.sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw)
        if norm == 0.0:
            return ValidationResult(False, "ZERO_QUATERNION", f"poses[{index}] has zero quaternion")
        if abs(norm - 1.0) > quaternion_tolerance:
            return ValidationResult(False, "QUATERNION_NOT_NORMALIZED", f"poses[{index}] quaternion norm {norm:.6f}")
        if previous_pose is not None and _distance_xy(previous_pose, pose) < min_waypoint_separation_m:
            return ValidationResult(
                False,
                "CONSECUTIVE_WAYPOINTS_TOO_CLOSE",
                f"poses[{index}] is closer than {min_waypoint_separation_m:.3f} m to the previous pose",
            )
        poses.append(pose)
        previous_pose = pose

    mission = NormalizedRouteMission(
        mission_id=mission_id,
        route_id=route_id,
        topology_version=topology_version,
        frame_id=mission_frame,
        node_ids=tuple(node_ids),
        edge_ids=tuple(edge_ids),
        edge_directions=tuple(edge_directions),
        poses=tuple(poses),
    )
    return ValidationResult(True, "VALID", "route mission is valid", mission)
