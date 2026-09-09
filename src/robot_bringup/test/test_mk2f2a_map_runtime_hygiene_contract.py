from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "nav2_common.yaml"
LAUNCH = ROOT / "launch" / "bringup.launch.py"


def test_global_costmap_is_saved_map_only_and_local_costmap_keeps_dual_sensor_authority():
    config = yaml.safe_load(CONFIG.read_text())
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    assert global_costmap["plugins"] == ["static_layer", "inflation_layer"]
    assert "voxel_layer" not in global_costmap
    assert global_costmap["static_layer"]["map_topic"] == "/map"
    assert local_costmap["voxel_layer"]["observation_sources"] == "lidar_cloud tmini_scan"


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
