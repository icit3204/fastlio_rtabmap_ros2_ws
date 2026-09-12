"""Static contract for the R3A planar-odometry/free-space mapping repair."""

import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _transform(translation, rpy):
    result = np.eye(4)
    result[:3, :3] = _rotation(*rpy)
    result[:3, 3] = translation
    return result


def test_rtabmap_mapping_uses_native_chassis_tf_odometry_and_ray_tracing():
    bringup = (ROOT / "launch" / "bringup.launch.py").read_text()
    bridge = (ROOT / "launch" / "rtabmap_bridge.launch.py").read_text()

    assert "DeclareLaunchArgument('rtabmap_odom_frame_id', default_value='odom_chassis')" in bringup
    assert "'odom_frame_id': LaunchConfiguration('rtabmap_odom_frame_id')" in bringup
    assert "DeclareLaunchArgument('odom_frame_id', default_value='odom_chassis')" in bridge
    assert "'odom_frame_id': LaunchConfiguration('odom_frame_id')" in bridge
    assert "--Grid/RayTracing true" in bringup
    assert "--Grid/RayTracing true" in bridge
    assert "DeclareLaunchArgument(\n            'rtabmap_imu_topic'" in bringup
    assert "'rtabmap_imu_topic': LaunchConfiguration('rtabmap_imu_topic')" in bringup
    assert "DeclareLaunchArgument('rtabmap_imu_topic', default_value='/unused_imu')" in bridge
    assert "'imu_topic': LaunchConfiguration('rtabmap_imu_topic')" in bridge

    calibrated = (ROOT / "launch" / "mkmini_calibrated_localization.launch.py").read_text()
    assert '"odom_chassis", "odom", base_to_body' in calibrated
    assert '"odom", "odom_chassis", body_to_base' not in calibrated


def test_existing_chassis_basis_removes_r3_sensor_pitch_vertical_coupling():
    # Recorded R3 raw FAST-LIO odom->body poses.  The test intentionally
    # leaves that raw odometry unchanged and only evaluates the existing
    # odom_chassis/base_footprint TF composition.
    body_to_base = _transform(
        (0.297163177464, -0.054016303975, -0.774031175447),
        tuple(math.radians(value) for value in (-0.489726635719, -31.408864239310, -0.877933945254)),
    )
    raw_before = _transform((0.028, 0.0, 0.0), (0.0, 0.0, math.radians(-0.033)))
    raw_after = _transform((1.229, -0.048, 0.721), (0.0, 0.0, math.radians(-2.14)))

    raw_delta = np.linalg.inv(raw_before) @ raw_after
    chassis_before = np.linalg.inv(body_to_base) @ raw_before @ body_to_base
    chassis_after = np.linalg.inv(body_to_base) @ raw_after @ body_to_base
    chassis_delta = np.linalg.inv(chassis_before) @ chassis_after

    assert abs(raw_delta[2, 3] - 0.721) < 0.001
    assert abs(chassis_delta[2, 3]) < 0.02
    assert math.hypot(chassis_delta[0, 3], chassis_delta[1, 3]) > 1.0


def test_fastlio_raw_odom_contract_is_not_rewritten_by_mapping_repair():
    source = (ROOT.parent / "FAST_LIO_ROS2" / "src" / "laserMapping.cpp").read_text()
    assert 'odomAftMapped.header.frame_id = "odom"' in source
    assert 'odomAftMapped.child_frame_id = "body"' in source
