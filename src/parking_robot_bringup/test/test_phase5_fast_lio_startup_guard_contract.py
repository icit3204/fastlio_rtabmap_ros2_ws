from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE.parent
FAST_LIO_SOURCE = SRC / "FAST_LIO_ROS2" / "src" / "laserMapping.cpp"
MAPPING_LAUNCH = SRC / "FAST_LIO_ROS2" / "launch" / "mapping.launch.py"
PHASE5_LOCALIZATION = SRC / "robot_bringup" / "launch" / "phase5_calibrated_localization.launch.py"
MID360 = SRC / "FAST_LIO_ROS2" / "config" / "mid360.yaml"


def test_guard_defaults_are_opt_in_and_phase5_values_are_explicit():
    source = FAST_LIO_SOURCE.read_text()
    mapping = MAPPING_LAUNCH.read_text()
    phase5 = PHASE5_LOCALIZATION.read_text()
    assert 'declare_parameter<bool>("startup_freshness_guard_enabled", false)' in source
    assert 'declare_parameter<double>("startup_max_lidar_age_sec", 0.5)' in source
    assert 'declare_parameter<int>("startup_fresh_consecutive_groups", 10)' in source
    assert "'startup_freshness_guard_enabled', default_value='false'" in mapping
    assert "'startup_max_lidar_age_sec', default_value='0.5'" in mapping
    assert "'startup_fresh_consecutive_groups', default_value='10'" in mapping
    assert '"startup_freshness_guard_enabled": "true"' in phase5
    assert '"startup_max_lidar_age_sec": "0.5"' in phase5
    assert '"startup_fresh_consecutive_groups": "10"' in phase5


def test_guard_is_before_estimator_and_publication_path():
    source = FAST_LIO_SOURCE.read_text()
    guard = source.index("const auto startup_decision")
    first_scan = source.index("if (flg_first_scan)")
    process = source.index("p_imu->Process(Measures", guard)
    odom = source.index("publish_odometry", process)
    body_cloud = source.index("publish_frame_body", odom)
    assert guard < first_scan < process < odom < body_cloud


def test_startup_ingress_filter_precedes_legacy_livox_preprocessing():
    source = FAST_LIO_SOURCE.read_text()
    wrapper = source.index("void livox_pcl_cbk(livox_ros_driver2::msg::CustomMsg::UniquePtr msg)")
    ingress = source.index("startup_freshness_guard_->observe_ingress", wrapper)
    legacy_call = source.index("livox_pcl_cbk_legacy(msg, loopback)", ingress)
    legacy_body = source.index("void livox_pcl_cbk_legacy(")
    preprocessing = source.index("p_pre->process(msg, ptr);", legacy_body)
    buffer_push = source.index("lidar_buffer.push_back(ptr);", preprocessing)
    assert wrapper < ingress < legacy_call
    assert legacy_body < preprocessing < buffer_push
    assert "startup_freshness_guard_->ingress_stale_dropped()" in source
    assert "startup_freshness_guard_->fresh_group_count()" in source
    assert "std::bind(&LaserMappingNode::livox_pcl_cbk" in source


def test_ingress_reuses_phase5_threshold_and_has_no_new_parameter():
    source = FAST_LIO_SOURCE.read_text()
    assert "observe_ingress(" in source or "observe_ingress(" in (SRC / "FAST_LIO_ROS2" / "include" / "startup_freshness_guard.hpp").read_text()
    assert source.count('declare_parameter<double>("startup_max_lidar_age_sec", 0.5)') == 1
    assert source.count('declare_parameter<int>("startup_fresh_consecutive_groups", 10)') == 1


def test_geometry_and_sensor_config_remain_unchanged_in_shared_mid360_yaml():
    yaml_text = MID360.read_text()
    assert "lid_topic:  \"/livox/lidar\"" in yaml_text
    assert "imu_topic:  \"/livox/imu\"" in yaml_text
    assert "point_filter_num: 3" in yaml_text
    assert "extrinsic_T: [ -0.011, -0.02329, 0.04412 ]" in yaml_text
    assert "extrinsic_R: [ 1., 0., 0.," in yaml_text
