"""Pure math helpers for the Phase 2 fake base.

This module intentionally has no ROS imports and no side effects on import.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Twist2D:
    vx: float
    wz: float


def normalize_yaw(yaw: float) -> float:
    """Normalize yaw to [-pi, pi)."""
    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def is_finite_twist(twist: Twist2D) -> bool:
    return math.isfinite(twist.vx) and math.isfinite(twist.wz)


def clamp_dt(dt_sec: float, max_dt_sec: float) -> float:
    if not math.isfinite(dt_sec) or dt_sec <= 0.0:
        return 0.0
    if max_dt_sec <= 0.0 or not math.isfinite(max_dt_sec):
        raise ValueError("max_dt_sec must be positive and finite")
    return min(dt_sec, max_dt_sec)


def integrate_pose(pose: Pose2D, twist: Twist2D, dt_sec: float, max_dt_sec: float) -> Pose2D:
    """Integrate planar differential-drive motion with timestep clamping."""
    if not is_finite_twist(twist):
        raise ValueError("twist must be finite")
    dt = clamp_dt(dt_sec, max_dt_sec)
    if dt == 0.0:
        return Pose2D(pose.x, pose.y, normalize_yaw(pose.yaw))

    theta = normalize_yaw(pose.yaw)
    v = twist.vx
    w = twist.wz

    if abs(w) < 1.0e-9:
        x = pose.x + v * math.cos(theta) * dt
        y = pose.y + v * math.sin(theta) * dt
        yaw = theta
    else:
        new_theta = theta + w * dt
        radius = v / w
        x = pose.x + radius * (math.sin(new_theta) - math.sin(theta))
        y = pose.y - radius * (math.cos(new_theta) - math.cos(theta))
        yaw = new_theta

    return Pose2D(float(x), float(y), normalize_yaw(yaw))


def stale_or_missing_command(
    now_steady_sec: float,
    last_command_steady_sec: float | None,
    timeout_sec: float,
) -> bool:
    if last_command_steady_sec is None:
        return True
    if timeout_sec < 0.0 or not math.isfinite(timeout_sec):
        raise ValueError("timeout_sec must be non-negative and finite")
    return (now_steady_sec - last_command_steady_sec) > timeout_sec


def command_for_integration(
    twist: Twist2D | None,
    now_steady_sec: float,
    last_command_steady_sec: float | None,
    timeout_sec: float,
) -> Twist2D:
    if twist is None or stale_or_missing_command(now_steady_sec, last_command_steady_sec, timeout_sec):
        return Twist2D(0.0, 0.0)
    if not is_finite_twist(twist):
        raise ValueError("twist must be finite")
    return twist


def reset_pose(x: float, y: float, yaw: float) -> Pose2D:
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
        raise ValueError("reset pose must be finite")
    return Pose2D(float(x), float(y), normalize_yaw(float(yaw)))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    yaw = normalize_yaw(yaw)
    half = 0.5 * yaw
    qz = math.sin(half)
    qw = math.cos(half)
    norm = math.sqrt(qz * qz + qw * qw)
    return (0.0, 0.0, qz / norm, qw / norm)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError("quaternion must be finite")
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return normalize_yaw(math.atan2(siny_cosp, cosy_cosp))

