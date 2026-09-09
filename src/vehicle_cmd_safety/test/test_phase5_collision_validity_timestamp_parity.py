"""P5-3F2 source-timestamp parity and callback regressions."""

import pytest

from vehicle_cmd_safety.validity_core import (
    CollisionMonitorState,
    CollisionMonitorValidityCore,
    ValidityConfig,
)


def healthy_state() -> CollisionMonitorState:
    return CollisionMonitorState(
        lifecycle_reachable=True,
        lifecycle_active=True,
        config_reachable=True,
        configured_source_present=True,
        configured_source_type_matches=True,
        configured_source_topic_matches=True,
    )


def healthy_core() -> CollisionMonitorValidityCore:
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_collision_monitor_state(healthy_state())
    return core


def make_observation(core, receipt: float, source: float) -> None:
    core.set_observation("base_footprint", True, receipt, source)


def test_healthy_receipt_and_header_recover_to_valid():
    core = healthy_core()
    make_observation(core, receipt=10.0, source=9.8)
    assert core.tick(10.0, 10.0).reason_code == "RECOVERY_STABILITY_WAIT"
    make_observation(core, receipt=10.5, source=10.5)
    assert core.tick(10.5, 10.5).valid


def test_p5_3f_parity_fresh_receipt_with_1_6_second_old_header_is_invalid():
    core = healthy_core()
    make_observation(core, receipt=20.0, source=18.4)
    status = core.tick(20.01, 20.0)
    assert not status.valid
    assert status.reason_code == "SOURCE_TIMESTAMP_STALE"
    assert status.diagnostics["source_receipt_age_sec"] == "0.010000"
    assert status.diagnostics["source_header_age_sec"] == "1.600000"


def test_exact_header_boundary_is_accepted_by_one_sided_cm_contract():
    core = healthy_core()
    make_observation(core, receipt=30.0, source=29.5)
    assert core.tick(30.0, 30.0).reason_code == "RECOVERY_STABILITY_WAIT"


def test_header_just_over_boundary_is_source_timestamp_stale():
    core = healthy_core()
    make_observation(core, receipt=30.0, source=29.499999)
    assert core.tick(30.0, 30.0).reason_code == "SOURCE_TIMESTAMP_STALE"


def test_gross_old_and_zero_headers_are_source_timestamp_stale():
    for source in (25.0, 0.0):
        core = healthy_core()
        make_observation(core, receipt=30.0, source=source)
        assert core.tick(30.0, 30.0).reason_code == "SOURCE_TIMESTAMP_STALE"


def test_receipt_stale_remains_fail_closed_when_header_is_fresh():
    core = healthy_core()
    make_observation(core, receipt=29.499999, source=30.0)
    assert core.tick(30.0, 30.0).reason_code == "SOURCE_STALE"


def test_both_stale_uses_existing_receipt_stale_reason_first():
    core = healthy_core()
    make_observation(core, receipt=29.0, source=28.0)
    assert core.tick(30.0, 30.0).reason_code == "SOURCE_STALE"


def test_future_header_is_accepted_by_exact_cm_parity_rule():
    core = healthy_core()
    make_observation(core, receipt=40.0, source=40.1)
    assert core.tick(40.0, 40.0).reason_code == "RECOVERY_STABILITY_WAIT"


def test_header_stale_recovery_requires_full_stability_window():
    core = healthy_core()
    make_observation(core, receipt=50.0, source=48.0)
    assert core.tick(50.0, 50.0).reason_code == "SOURCE_TIMESTAMP_STALE"
    make_observation(core, receipt=50.1, source=50.0)
    assert core.tick(50.1, 50.1).reason_code == "RECOVERY_STABILITY_WAIT"
    make_observation(core, receipt=50.59, source=50.59)
    assert core.tick(50.59, 50.59).reason_code == "RECOVERY_STABILITY_WAIT"
    make_observation(core, receipt=50.60, source=50.60)
    assert core.tick(50.60, 50.60).valid


def test_old_header_during_recovery_resets_stability():
    core = healthy_core()
    make_observation(core, receipt=60.0, source=60.0)
    assert core.tick(60.0, 60.0).reason_code == "RECOVERY_STABILITY_WAIT"
    assert core.tick(60.3, 60.3).reason_code == "RECOVERY_STABILITY_WAIT"
    make_observation(core, receipt=60.31, source=58.0)
    assert core.tick(60.31, 60.31).reason_code == "SOURCE_TIMESTAMP_STALE"
    make_observation(core, receipt=60.4, source=60.4)
    assert core.tick(60.4, 60.4).reason_code == "RECOVERY_STABILITY_WAIT"
    assert core.tick(60.9, 60.9).valid


def test_repeated_healthy_headers_keep_validity_stable():
    core = healthy_core()
    make_observation(core, receipt=70.0, source=70.0)
    assert not core.tick(70.0, 70.0).valid
    assert core.tick(70.5, 70.5).valid
    make_observation(core, receipt=70.6, source=70.6)
    assert core.tick(70.9, 70.9).valid


@pytest.fixture(scope="module")
def monitor_node():
    rclpy = pytest.importorskip("rclpy")
    from vehicle_cmd_safety.collision_monitor_validity_monitor import CollisionMonitorValidityMonitor

    rclpy.init(args=[])
    node = CollisionMonitorValidityMonitor()
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_pointcloud_callback_retains_ros_header_and_receipt(monkeypatch, monitor_node):
    from sensor_msgs.msg import PointCloud2

    monkeypatch.setattr(monitor_node, "_now_steady", lambda: 100.0)
    msg = PointCloud2()
    msg.header.frame_id = "base_footprint"
    msg.header.stamp.sec = 98
    msg.header.stamp.nanosec = 400000000
    msg.width = 1
    msg.height = 1
    msg.point_step = 1
    msg.row_step = 1
    msg.data = b"\x00"
    monitor_node._cloud_cb(msg)
    assert monitor_node._core.observation.receipt_stamp_steady == 100.0
    assert monitor_node._core.observation.source_stamp_ros == pytest.approx(98.4)


def test_laserscan_callback_retains_ros_header_and_receipt(monkeypatch, monitor_node):
    from sensor_msgs.msg import LaserScan

    monkeypatch.setattr(monitor_node, "_now_steady", lambda: 200.0)
    msg = LaserScan()
    msg.header.frame_id = "base_footprint"
    msg.header.stamp.sec = 199
    msg.header.stamp.nanosec = 500000000
    msg.angle_increment = 0.1
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = [1.0]
    monitor_node._scan_cb(msg)
    assert monitor_node._core.observation.receipt_stamp_steady == 200.0
    assert monitor_node._core.observation.source_stamp_ros == pytest.approx(199.5)
