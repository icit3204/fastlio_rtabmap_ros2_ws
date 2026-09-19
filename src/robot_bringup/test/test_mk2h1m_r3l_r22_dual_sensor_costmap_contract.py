"""R22: Nav2 receives bounded T-mini and filtered MID-360 observations."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent
NAV = yaml.safe_load((ROOT / "config/nav2_common.yaml").read_text())
FILTER = yaml.safe_load((ROOT / "config/mkmini_mid360_nav_filter.yaml").read_text())
FILTER_PARAMS = FILTER["mid360_nav_obstacle_filter"]["ros__parameters"]
R3H = (SRC / "parking_robot_bringup/launch/r3h_physical_navigation.launch.py").read_text()
CANONICAL = (ROOT / "launch/bringup.launch.py").read_text()
COLLISION = yaml.safe_load((ROOT / "config/collision_monitor_dual_sensor.yaml").read_text())
MID_VALIDITY = yaml.safe_load(
    (SRC / "vehicle_cmd_safety/config/phase5_dual_mid360_validity.yaml").read_text())
TMINI = yaml.safe_load((ROOT / "config/tmini_mk2c.yaml").read_text())
SCAN_NORMALIZER = (ROOT / "src/tmini_scan_normalizer.cpp").read_text()


def _assert_mid_layer(params):
    layer = params["mid360_obstacle_layer"]
    assert layer["plugin"] == \
        "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    assert layer["voxel_decay"] == 1.0
    assert layer["decay_model"] == 0
    assert layer["observation_sources"] == "mid360_marking"
    marking = layer["mid360_marking"]
    assert marking == {
        "topic": "/cloud_registered_nav2_obstacles",
        "sensor_frame": "body",
        "data_type": "PointCloud2",
        "clearing": False,
        "marking": True,
        "min_obstacle_height": 0.15,
        "max_obstacle_height": 1.60,
        "obstacle_range": 3.00,
        "expected_update_rate": 0.0,
        "observation_persistence": 0.0,
        "filter": "passthrough",
        "voxel_min_points": 0,
        "clear_after_reading": True,
    }


def test_both_costmaps_keep_tmini_and_add_filtered_mid():
    local = NAV["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = NAV["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["plugins"] == [
        "tmini_obstacle_layer", "mid360_obstacle_layer", "inflation_layer"]
    assert global_["plugins"] == [
        "static_layer", "tmini_obstacle_layer", "mid360_obstacle_layer",
        "inflation_layer"]
    for params in (local, global_):
        tmini = params["tmini_obstacle_layer"]["tmini_scan"]
        assert tmini["topic"] == "/scan"
        assert tmini["marking"] is True and tmini["clearing"] is True
        assert tmini["inf_is_valid"] is True
        _assert_mid_layer(params)


def test_tmini_no_return_contract_supports_open_space_clearing():
    driver = TMINI["ydlidar_ros2_driver_node"]["ros__parameters"]
    assert driver["invalid_range_is_inf"] is False
    assert '"/scan_raw"' in SCAN_NORMALIZER
    assert '"/scan"' in SCAN_NORMALIZER
    assert "normalize_tmini_range" in SCAN_NORMALIZER
    for launch in (R3H, CANONICAL):
        assert "tmini_scan_normalizer" in launch
        assert "/scan_raw" in launch


def test_mid_filter_is_downstream_of_self_crop_and_is_frame_bounded():
    assert FILTER_PARAMS == {
        "input_topic": "/cloud_registered_rtabmap",
        "obstacle_topic": "/cloud_registered_nav2_obstacles",
        "clearing_topic": "/cloud_registered_nav2_clearing",
        "expected_input_frame": "body",
        "filter_frame": "base_footprint",
        "tf_timeout_sec": 0.10,
        "obstacle_min_height": 0.15,
        "obstacle_max_height": 1.60,
        "obstacle_min_range": 0.10,
        "obstacle_max_range": 3.00,
        "clearing_min_height": -0.20,
        "clearing_max_height": 2.00,
        "clearing_min_range": 0.10,
        "clearing_max_range": 3.50,
    }
    assert 'executable="mid360_nav_obstacle_filter"' in R3H
    assert '"mkmini_mid360_nav_filter.yaml"' in R3H
    assert "executable='mid360_nav_obstacle_filter'" in CANONICAL
    assert "'mkmini_mid360_nav_filter.yaml'" in CANONICAL


def test_collision_monitor_ownership_and_planner_geometry_are_unchanged():
    collision = COLLISION["collision_monitor"]["ros__parameters"]
    assert collision["observation_sources"] == ["mid360_body_cloud", "tmini_scan"]
    assert collision["mid360_body_cloud"]["topic"] == "/cloud_registered_nav2_obstacles"
    validity = MID_VALIDITY["/**"]["ros__parameters"]
    assert validity["source_topic"] == "/cloud_registered_nav2_obstacles"
    assert validity["expected_observation_source_topic"] == "/cloud_registered_nav2_obstacles"
    planner = NAV["planner_server"]["ros__parameters"]["GridBased"]
    controller = NAV["controller_server"]["ros__parameters"]["FollowPath"]
    assert planner["motion_model_for_search"] == "DUBIN"
    assert planner["minimum_turning_radius"] == 1.75
    assert controller["AckermannConstraints"]["min_turning_r"] == 1.75
