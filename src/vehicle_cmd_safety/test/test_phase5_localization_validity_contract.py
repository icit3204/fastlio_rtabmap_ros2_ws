from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
NODE = ROOT / "vehicle_cmd_safety" / "localization_validity_monitor.py"


def test_public_topic_types_and_reliable_volatile_qos_are_explicit():
    text = NODE.read_text()
    expected = {
        'Bool, "/system/localization_valid"',
        'String, "/system/localization_reason"',
        'Float64, "/system/pose_age"',
        'Float64, "/system/tf_age"',
        'Bool, "/system/pose_jump_detected"',
    }
    assert all(item in text for item in expected)
    assert "ReliabilityPolicy.RELIABLE" in text
    assert "DurabilityPolicy.VOLATILE" in text


def test_monitor_is_tf_consumer_only_and_requires_both_transforms():
    text = NODE.read_text()
    assert "TransformListener" in text and "lookup_transform" in text
    assert "TransformBroadcaster" not in text and "StaticTransformBroadcaster" not in text
    assert "self._map_frame, self._odom_frame" in text
    assert "self._map_frame, self._base_frame" in text


def test_gate_and_mission_manager_consume_identical_bool_permission_contract():
    gate = (ROOT / "vehicle_cmd_safety" / "guarded_vehicle_cmd_gate.py").read_text()
    mission = (ROOT.parent / "parking_robot_mission_manager" / "parking_robot_mission_manager" / "mission_manager_node.py").read_text()
    assert "create_subscription(Bool, self._localization_topic" in gate
    assert 'topic("localization_valid_topic")' in mission
    assert 'localization_valid_topic", "/system/localization_valid"' in gate
    assert 'localization_valid_topic", "/system/localization_valid"' in mission
    assert 'localization_timeout_sec", 0.50' in gate


def test_configuration_has_bounded_phase5_defaults():
    config = yaml.safe_load((ROOT / "config" / "phase5_localization_validity.yaml").read_text())
    params = config["localization_validity_monitor"]["ros__parameters"]
    assert params["odom_topic"] == "/Odometry"
    assert (params["map_frame"], params["odom_frame"], params["base_frame"]) == (
        "map", "odom", "base_footprint"
    )
    assert params["odom_freshness_sec"] == 0.50
    assert params["tf_freshness_sec"] == 0.50
    assert params["stability_min_observations"] >= 2


def test_configuration_matches_rtab_future_tf_contract():
    config = yaml.safe_load((ROOT / "config" / "phase5_localization_validity.yaml").read_text())
    params = config["localization_validity_monitor"]["ros__parameters"]
    assert params["future_stamp_tolerance_sec"] == 0.10
    assert "tf_tolerance=0.10" in (ROOT / "config" / "phase5_localization_validity.yaml").read_text()


def test_package_exports_monitor_and_required_runtime_dependencies():
    setup = (ROOT / "setup.py").read_text()
    package = (ROOT / "package.xml").read_text()
    assert "localization_validity_monitor = vehicle_cmd_safety.localization_validity_monitor:main" in setup
    assert "<exec_depend>nav_msgs</exec_depend>" in package
    assert "<exec_depend>tf2_ros</exec_depend>" in package
