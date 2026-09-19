from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "nav2_common.yaml"
LAUNCH = ROOT / "launch" / "bringup.launch.py"


def test_global_and_local_costmaps_use_tmini_live_obstacle_authority():
    config = yaml.safe_load(CONFIG.read_text())
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    assert global_costmap["plugins"] == [
        "static_layer", "tmini_obstacle_layer", "mid360_obstacle_layer",
        "inflation_layer"]
    assert global_costmap["static_layer"]["map_topic"] == "/map"
    live = global_costmap["tmini_obstacle_layer"]
    assert live["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert live["observation_persistence"] == 0.0
    assert live["observation_sources"] == "tmini_scan"
    source = live["tmini_scan"]
    assert source["topic"] == "/scan"
    assert source["data_type"] == "LaserScan"
    assert source["marking"] is True and source["clearing"] is True
    assert source["raytrace_max_range"] > source["obstacle_max_range"]
    assert local_costmap["plugins"] == [
        "tmini_obstacle_layer", "mid360_obstacle_layer", "inflation_layer"]
    assert local_costmap["tmini_obstacle_layer"]["observation_sources"] == \
        "tmini_scan"
    assert "lidar_cloud" not in local_costmap["tmini_obstacle_layer"]


def test_localization_uses_hash_verified_runtime_copy_and_mapping_is_excluded():
    source = LAUNCH.read_text()
    for required in (
        "rtabmap_use_working_copy",
        "rtabmap_runtime_dir",
        "prepare_rtabmap_working_copy",
        "shutil.copy2(reference, runtime_db)",
        "runtime_hash != reference_hash",
        "context.launch_configurations['database_path'] = str(runtime_db)",
        "mode.perform(context) == 'mapping'",
    ):
        assert required in source
