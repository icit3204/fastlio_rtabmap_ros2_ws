from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE.parent
BOUNDARY = SRC / "phase5_sensor_freshness_boundary"
BOUNDARY_CPP = BOUNDARY / "src" / "sensor_freshness_boundary.cpp"
CORE_HPP = BOUNDARY / "include" / "phase5_sensor_freshness_boundary" / "boundary_core.hpp"
SEMANTIC_HPP = BOUNDARY / "include" / "phase5_sensor_freshness_boundary" / "semantic_compare.hpp"
PHASE5_LAUNCH = SRC / "robot_bringup" / "launch" / "phase5_calibrated_localization.launch.py"
MAPPING_LAUNCH = SRC / "FAST_LIO_ROS2" / "launch" / "mapping.launch.py"
FAST_LIO_SOURCE = SRC / "FAST_LIO_ROS2" / "src" / "laserMapping.cpp"


def test_phase5_boundary_package_and_node_are_project_owned():
    assert (BOUNDARY / "package.xml").is_file()
    assert (BOUNDARY / "CMakeLists.txt").is_file()
    source = BOUNDARY_CPP.read_text()
    assert 'Node("phase5_sensor_freshness_boundary")' in source
    assert '"/livox/lidar"' in source
    assert '"/livox/imu"' in source
    assert '"/phase5/livox/lidar_fresh"' in source
    assert '"/phase5/livox/imu_fresh"' in source


def test_phase5_launch_routes_fast_lio_only_through_boundary_topics():
    launch = PHASE5_LAUNCH.read_text()
    mapping = MAPPING_LAUNCH.read_text()
    assert 'package="phase5_sensor_freshness_boundary"' in launch
    assert 'executable="sensor_freshness_boundary"' in launch
    assert '"lidar_topic": "/phase5/livox/lidar_fresh"' in launch
    assert '"imu_topic": "/phase5/livox/imu_fresh"' in launch
    assert '"lidar_input_topic": "/livox/lidar"' in launch
    assert '"imu_input_topic": "/livox/imu"' in launch
    assert "'lidar_topic', default_value='/livox/lidar'" in mapping
    assert "'imu_topic', default_value='/livox/imu'" in mapping
    assert "common.lid_topic': lidar_topic" in mapping
    assert "common.imu_topic': imu_topic" in mapping
    assert launch.count('"/phase5/livox/lidar_fresh"') == 2
    assert launch.count('"/phase5/livox/imu_fresh"') == 2
    assert launch.index("actions.extend([freshness_boundary") < launch.index("livox, rtabmap")


def test_phase5_boundary_parameters_and_fail_closed_contract_are_static():
    launch = PHASE5_LAUNCH.read_text()
    core = CORE_HPP.read_text()
    assert 'DeclareLaunchArgument("startup_freshness_boundary_enabled", default_value="true")' in launch
    assert 'DeclareLaunchArgument("startup_lidar_max_age_sec", default_value="0.5")' in launch
    assert 'DeclareLaunchArgument("startup_imu_max_age_sec", default_value="0.05")' in launch
    assert 'DeclareLaunchArgument("startup_stable_window_sec", default_value="2.0")' in launch
    assert "DROP_CLOSED" in core
    assert "state_ = BoundaryState::OPEN" in core
    assert "config_.stable_window_sec" in core
    assert "state_ == BoundaryState::OPEN" in core


def test_boundary_open_path_is_transparent_and_not_runtime_filtered():
    source = BOUNDARY_CPP.read_text()
    core = CORE_HPP.read_text()
    assert "publish(std::move(msg))" in source
    assert "if (decision == phase5_sensor_freshness_boundary::Decision::FORWARD_OPEN)" in source
    assert "state_ == BoundaryState::OPEN" in core
    assert "++counters_.open_lidar_received" in core
    assert "++counters_.open_lidar_forwarded" in core
    assert "++counters_.open_imu_received" in core
    assert "++counters_.open_imu_forwarded" in core
    assert "restamp" not in source.lower()
    assert "transform" not in source.lower()


def test_canonical_semantic_comparison_covers_exact_sensor_content():
    semantic = SEMANTIC_HPP.read_text()
    for token in (
        "a.timebase", "a.point_num", "a.lidar_id", "a.rsvd", "offset_time",
        "float_bits(x.x)", "float_bits(x.y)", "float_bits(x.z)",
        "orientation_covariance", "angular_velocity_covariance",
        "linear_acceleration_covariance", "double_bits(a.orientation.x)",
    ):
        assert token in semantic


def test_fast_lio_startup_guards_remain_in_phase5_route():
    phase5 = PHASE5_LAUNCH.read_text()
    fast_lio = FAST_LIO_SOURCE.read_text()
    assert '"startup_freshness_guard_enabled": "true"' in phase5
    assert '"startup_max_lidar_age_sec": "0.5"' in phase5
    assert '"startup_fresh_consecutive_groups": "10"' in phase5
    assert "startup_freshness_guard_" in fast_lio
    assert "observe_ingress" in fast_lio
    assert "startup_sync_drain_batch" in fast_lio
