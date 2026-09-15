from pathlib import Path
import yaml


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = (ROOT / "launch/rtabmap_bridge.launch.py").read_text()
BRINGUP = (ROOT / "launch/bringup.launch.py").read_text()
CALIBRATED_LOCALIZATION = (ROOT / "launch/phase5_calibrated_localization.launch.py").read_text()
NODE = (ROOT / "src/rtabmap_self_body_filter.cpp").read_text()
CONFIG = yaml.safe_load((ROOT / "config/mkmini_rtabmap_self_body_crop.yaml").read_text())
PARAMS = CONFIG["rtabmap_self_body_filter"]["ros__parameters"]
NAV2 = (ROOT / "config/nav2_common.yaml").read_text()
COLLISION = (ROOT / "config/collision_monitor_dual_sensor.yaml").read_text()


def test_temporary_mkmini_bounds_and_frame_are_explicit():
    assert PARAMS["input_topic"] == "/cloud_registered_body"
    assert PARAMS["output_topic"] == "/cloud_registered_rtabmap"
    assert PARAMS["expected_input_frame"] == "body"
    assert PARAMS["crop_frame"] == "base_footprint"
    assert [PARAMS[k] for k in ("x_min", "x_max")] == [0.540, 0.670]
    assert [PARAMS[k] for k in ("y_min", "y_max")] == [-0.140, 0.150]
    assert [PARAMS[k] for k in ("z_min", "z_max")] == [0.530, 0.580]


def test_rtab_only_topic_branch_and_disabled_generic_default():
    assert "DeclareLaunchArgument('enable_rtabmap_self_filter', default_value='false')" in BRIDGE
    assert "'/cloud_registered_rtabmap'" in BRIDGE
    assert "'enable_rtabmap_self_filter': 'true'" in BRINGUP
    assert '"enable_rtabmap_self_filter": "true"' in CALIBRATED_LOCALIZATION
    assert "'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic')" not in BRIDGE
    assert "topic: /cloud_registered_body" in NAV2
    assert 'topic: "/cloud_registered_body"' in COLLISION


def test_filter_fails_closed_without_touching_raw_topic():
    assert "Dropping RTAB cloud" in NODE
    assert "lookupTransform" in NODE
    assert "cloud->header.frame_id != expected_input_frame_" in NODE
    assert "publisher_->publish(output)" in NODE
    assert "create_publisher<sensor_msgs::msg::PointCloud2>(input_topic_" not in NODE


def test_r3a_and_r3g_contract_remain_present():
    assert "default_value='odom_chassis'" in BRIDGE
    assert "default_value='base_footprint'" in BRIDGE
    assert "default_value='/unused_imu'" in BRIDGE
    for setting in (
        "--Grid/3D true",
        "--Grid/NormalsSegmentation false",
        "--Grid/MinGroundHeight -0.20",
        "--Grid/MaxGroundHeight 0.15",
        "--Grid/MaxObstacleHeight 2.0",
        "--Grid/RayTracing true",
    ):
        assert setting in BRIDGE
