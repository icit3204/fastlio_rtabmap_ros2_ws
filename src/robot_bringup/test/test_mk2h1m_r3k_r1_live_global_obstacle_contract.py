"""R3K-R1: planner sees temporary T-mini obstacles without changing CM."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_global_live_obstacle_overlay_is_bounded_and_clearing_capable():
    config = yaml.safe_load((ROOT / "config/nav2_common.yaml").read_text())
    params = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert params["global_frame"] == "map"
    assert params["robot_base_frame"] == "base_footprint"
    assert params["track_unknown_space"] is True
    assert params["plugins"] == ["static_layer", "tmini_obstacle_layer", "inflation_layer"]
    source = params["tmini_obstacle_layer"]["tmini_scan"]
    assert source == {
        "topic": "/scan", "data_type": "LaserScan", "clearing": True,
        "marking": True, "min_obstacle_height": 0.0,
        "max_obstacle_height": 1.6, "raytrace_min_range": 0.0,
        "raytrace_max_range": 3.5, "obstacle_min_range": 0.0,
        "obstacle_max_range": 3.0,
    }


def test_local_dynamic_layer_uses_clearable_tmini_not_registered_map_cloud():
    config = yaml.safe_load((ROOT / "config/nav2_common.yaml").read_text())
    params = config["local_costmap"]["local_costmap"]["ros__parameters"]
    assert params["plugins"] == ["tmini_obstacle_layer", "inflation_layer"]
    obstacle = params["tmini_obstacle_layer"]
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle["observation_persistence"] == 0.0
    assert obstacle["observation_sources"] == "tmini_scan"
    assert "lidar_cloud" not in obstacle
    source = obstacle["tmini_scan"]
    assert source["topic"] == "/scan"
    assert source["marking"] is True and source["clearing"] is True
    assert source["raytrace_max_range"] > source["obstacle_max_range"]


def test_collision_monitor_keeps_its_independent_dual_sensor_contract():
    config = yaml.safe_load((ROOT / "config/collision_monitor_dual_sensor.yaml").read_text())
    params = config["collision_monitor"]["ros__parameters"]
    assert params["observation_sources"] == ["mid360_body_cloud", "tmini_scan"]
    assert params["tmini_scan"]["topic"] == "/scan"
    assert params["mid360_body_cloud"]["topic"] == "/cloud_registered_body"
