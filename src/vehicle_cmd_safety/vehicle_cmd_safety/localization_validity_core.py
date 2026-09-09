"""Deterministic fail-closed core for Phase-5 localization validity."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class Pose3:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class LocalizationValidityConfig:
    odom_freshness_sec: float = 0.50
    tf_freshness_sec: float = 0.50
    translation_jump_threshold_m: float = 1.0
    rotation_jump_threshold_rad: float = math.pi / 4.0
    quaternion_norm_tolerance: float = 1.0e-3
    stability_duration_sec: float = 1.0
    stability_min_observations: int = 3
    future_stamp_tolerance_sec: float = 0.05

    def validate(self) -> None:
        positive = (
            self.odom_freshness_sec,
            self.tf_freshness_sec,
            self.translation_jump_threshold_m,
            self.rotation_jump_threshold_rad,
            self.quaternion_norm_tolerance,
            self.stability_duration_sec,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("localization thresholds must be finite and positive")
        if self.stability_min_observations < 2:
            raise ValueError("stability_min_observations must be at least two")
        if not math.isfinite(self.future_stamp_tolerance_sec) or self.future_stamp_tolerance_sec < 0.0:
            raise ValueError("future_stamp_tolerance_sec must be finite and nonnegative")


@dataclass
class TimedPose:
    pose: Pose3
    steady_receipt_sec: float
    ros_stamp_sec: float
    sequence: int = 0


@dataclass
class TimedTransform:
    steady_receipt_sec: float
    ros_stamp_sec: float


@dataclass(frozen=True)
class LocalizationValidityStatus:
    valid: bool
    reason: str
    pose_age_sec: float
    tf_age_sec: float
    pose_jump_detected: bool
    stable_duration_sec: float
    stable_observations: int


class LocalizationValidityCore:
    """Pure state machine; steady time controls deadmen and ROS time controls source age."""

    def __init__(self, config: LocalizationValidityConfig) -> None:
        config.validate()
        self.config = config
        self.odometry: Optional[TimedPose] = None
        self.map_odom: Optional[TimedTransform] = None
        self.complete_pose: Optional[TimedPose] = None
        self._trusted_pose: Optional[Pose3] = None
        self._processed_sequence: Optional[int] = None
        self._stable_since: Optional[float] = None
        self._stable_observations = 0
        self._jump_latched = False

    def set_odometry(self, pose: Pose3, steady_receipt_sec: float, ros_stamp_sec: float) -> None:
        self.odometry = TimedPose(pose, steady_receipt_sec, ros_stamp_sec)

    def set_map_odom(self, steady_receipt_sec: float, ros_stamp_sec: float) -> None:
        self.map_odom = TimedTransform(steady_receipt_sec, ros_stamp_sec)

    def set_map_odom_unavailable(self) -> None:
        self.map_odom = None

    def set_complete_pose(
        self, pose: Pose3, steady_receipt_sec: float, ros_stamp_sec: float, sequence: int
    ) -> None:
        self.complete_pose = TimedPose(pose, steady_receipt_sec, ros_stamp_sec, sequence)

    def set_complete_pose_unavailable(self) -> None:
        self.complete_pose = None

    @staticmethod
    def _age(now: float, stamp: Optional[float]) -> float:
        return -1.0 if stamp is None else max(0.0, now - stamp)

    @staticmethod
    def _finite_position(pose: Pose3) -> bool:
        return all(math.isfinite(v) for v in (pose.x, pose.y, pose.z))

    @staticmethod
    def _finite_orientation(pose: Pose3) -> bool:
        return all(math.isfinite(v) for v in (pose.qx, pose.qy, pose.qz, pose.qw))

    @staticmethod
    def _quaternion_norm(pose: Pose3) -> float:
        return math.sqrt(pose.qx**2 + pose.qy**2 + pose.qz**2 + pose.qw**2)

    def _valid_quaternion(self, pose: Pose3) -> bool:
        norm = self._quaternion_norm(pose)
        return norm > 1.0e-12 and abs(norm - 1.0) <= self.config.quaternion_norm_tolerance

    @staticmethod
    def _translation_distance(a: Pose3, b: Pose3) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    @staticmethod
    def _rotation_distance(a: Pose3, b: Pose3) -> float:
        dot = abs(a.qx*b.qx + a.qy*b.qy + a.qz*b.qz + a.qw*b.qw)
        return 2.0 * math.acos(min(1.0, max(0.0, dot)))

    def _base_reason(self, steady_now: float, ros_now: float) -> str:
        if self.odometry is None:
            return "ODOM_NOT_RECEIVED"
        if ros_now + self.config.future_stamp_tolerance_sec < self.odometry.ros_stamp_sec:
            return "ODOM_TIMESTAMP_IN_FUTURE"
        if (steady_now - self.odometry.steady_receipt_sec > self.config.odom_freshness_sec
                or ros_now - self.odometry.ros_stamp_sec > self.config.odom_freshness_sec):
            return "ODOM_STALE"
        if self.map_odom is None:
            return "MAP_ODOM_MISSING"
        if ros_now + self.config.future_stamp_tolerance_sec < self.map_odom.ros_stamp_sec:
            return "MAP_ODOM_TIMESTAMP_IN_FUTURE"
        if (steady_now - self.map_odom.steady_receipt_sec > self.config.tf_freshness_sec
                or ros_now - self.map_odom.ros_stamp_sec > self.config.tf_freshness_sec):
            return "MAP_ODOM_STALE"
        if self.complete_pose is None:
            return "MAP_BASE_UNRESOLVABLE"
        poses = (self.odometry.pose, self.complete_pose.pose)
        if not all(self._finite_position(pose) for pose in poses):
            return "NONFINITE_POSITION"
        if not all(self._finite_orientation(pose) for pose in poses):
            return "NONFINITE_ORIENTATION"
        if not all(self._valid_quaternion(pose) for pose in poses):
            return "INVALID_QUATERNION"
        return "HEALTHY"

    def tick(self, steady_now: float, ros_now: float) -> LocalizationValidityStatus:
        reason = self._base_reason(steady_now, ros_now)
        pose_age = self._age(ros_now, None if self.odometry is None else self.odometry.ros_stamp_sec)
        tf_age = self._age(ros_now, None if self.map_odom is None else self.map_odom.ros_stamp_sec)
        if reason != "HEALTHY":
            self._stable_since = None
            self._stable_observations = 0
            return self._status(False, reason, pose_age, tf_age, steady_now)

        assert self.complete_pose is not None
        if self.complete_pose.sequence != self._processed_sequence:
            pose = self.complete_pose.pose
            jumped = False
            jump_reason = ""
            if self._trusted_pose is not None:
                if self._translation_distance(self._trusted_pose, pose) > self.config.translation_jump_threshold_m:
                    jumped, jump_reason = True, "TRANSLATION_JUMP"
                elif self._rotation_distance(self._trusted_pose, pose) > self.config.rotation_jump_threshold_rad:
                    jumped, jump_reason = True, "ROTATION_JUMP"
            self._trusted_pose = pose
            self._processed_sequence = self.complete_pose.sequence
            if self._stable_since is None or jumped:
                self._stable_since = steady_now
                self._stable_observations = 1
            else:
                self._stable_observations += 1
            if jumped:
                self._jump_latched = True
                return self._status(False, jump_reason, pose_age, tf_age, steady_now)

        stable_for = 0.0 if self._stable_since is None else max(0.0, steady_now - self._stable_since)
        if (stable_for < self.config.stability_duration_sec
                or self._stable_observations < self.config.stability_min_observations):
            return self._status(False, "STABILITY_WAIT", pose_age, tf_age, steady_now)
        self._jump_latched = False
        return self._status(True, "VALID", pose_age, tf_age, steady_now)

    def _status(self, valid: bool, reason: str, pose_age: float, tf_age: float,
                steady_now: float) -> LocalizationValidityStatus:
        stable_for = 0.0 if self._stable_since is None else max(0.0, steady_now - self._stable_since)
        return LocalizationValidityStatus(
            valid=valid,
            reason=reason,
            pose_age_sec=pose_age,
            tf_age_sec=tf_age,
            pose_jump_detected=self._jump_latched,
            stable_duration_sec=stable_for,
            stable_observations=self._stable_observations,
        )
