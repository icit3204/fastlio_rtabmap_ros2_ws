from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_minimal_launch_uses_real_local_costmap_only():
    text = (ROOT / "launch" / "mk2c_local_costmap_observation.launch.py").read_text()
    assert 'package="nav2_costmap_2d"' in text
    assert 'executable="nav2_costmap_2d"' in text
    assert '"nav2_common.yaml"' in text
    assert 'node_names": ["costmap/costmap"]' in text
    assert 'source["local_costmap"]["local_costmap"]["ros__parameters"]' in text
    assert '"/**": {"ros__parameters": params}' in text
    assert 'DeclareLaunchArgument("start_stack", default_value="false")' in text
    for prohibited in ("controller_server", "planner_server", "bt_navigator", "collision_monitor", "cmd_vel", "mkmini_cmd_adapter"):
        assert prohibited not in text.lower()


def test_production_local_costmap_cloud_contract_is_explicit():
    config = yaml.safe_load((ROOT / "config" / "nav2_common.yaml").read_text())
    params = config["local_costmap"]["local_costmap"]["ros__parameters"]
    voxel = params["voxel_layer"]
    source = voxel["lidar_cloud"]
    assert params["global_frame"] == "odom_chassis"
    assert params["robot_base_frame"] == "base_footprint"
    assert params["footprint"] == "[[ 0.755, 0.300 ],[ 0.755, -0.300 ],[-0.145, -0.300 ],[-0.145, 0.300 ]]"
    assert voxel["plugin"] == "nav2_costmap_2d::VoxelLayer"
    assert (voxel["origin_z"], voxel["z_resolution"], voxel["z_voxels"]) == (0.0, 0.1, 16)
    assert source["topic"] == "/cloud_registered_body"
    assert source["data_type"] == "PointCloud2"
    assert (source["min_obstacle_height"], source["max_obstacle_height"]) == (0.1, 1.6)
    assert (source["obstacle_min_range"], source["obstacle_max_range"]) == (0.0, 3.0)
    assert (source["raytrace_min_range"], source["raytrace_max_range"]) == (0.0, 3.5)


def test_production_costmaps_never_use_sensor_tilted_body_as_global_frame():
    text = (ROOT / "config" / "nav2_common.yaml").read_text()
    assert "global_frame: body" not in text


def test_production_local_costmap_has_native_tmini_scan_source():
    config = yaml.safe_load((ROOT / "config" / "nav2_common.yaml").read_text())
    params = config["local_costmap"]["local_costmap"]["ros__parameters"]
    voxel = params["voxel_layer"]
    assert voxel["observation_sources"] == "lidar_cloud tmini_scan"
    scan = voxel["tmini_scan"]
    assert scan["topic"] == "/scan"
    assert scan["data_type"] == "LaserScan"
    assert scan["marking"] is True and scan["clearing"] is True
    assert (scan["min_obstacle_height"], scan["max_obstacle_height"]) == (0.0, 1.6)
    assert (scan["obstacle_min_range"], scan["obstacle_max_range"]) == (0.0, 3.0)
    assert (scan["raytrace_min_range"], scan["raytrace_max_range"]) == (0.0, 3.5)


def test_tmini_driver_and_tf_use_frozen_authority():
    params = yaml.safe_load((ROOT / "config" / "tmini_mk2c.yaml").read_text())
    p = params["ydlidar_ros2_driver_node"]["ros__parameters"]
    assert p["port"] == "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    assert p["frame_id"] == "laser_frame"
    assert p["reversion"] is False and p["inverted"] is True
    launch = (ROOT / "launch" / "mk2c_local_costmap_observation.launch.py").read_text()
    assert '"--x", "0.703"' in launch
    assert '"--z", "0.1923"' in launch
    assert '"--frame-id", "base_link", "--child-frame-id", "laser_frame"' in launch
    assert "mk2c_tmini_odom_chassis_to_base_footprint" in launch
    assert "start_fast_lio" in launch


def test_calibrated_launch_anchors_horizontal_chassis_odom_frame():
    text = (ROOT / "launch" / "mkmini_calibrated_localization.launch.py").read_text()
    assert '_conditional_static_node("mkmini_odom_chassis_to_odom", "odom_chassis", "odom", base_to_body, start_fast_lio)' in text
    assert 'load_current_frame_authority(mkmini_share)' in text
    assert 'body_to_base = authority["body_to_base_footprint"]' in text
    assert 'base_to_body = authority["base_footprint_to_body"]' in text
    assert "BODY_TO_BASE_FOOTPRINT" not in text


def test_reconciliation_observer_transforms_before_height_filtering():
    text = (ROOT / "scripts" / "mk2c1r_reconciliation_observer.py").read_text()
    assert "points_costmap = transform_points(points_source, cloud_tf)" in text
    assert "accepted = points_costmap[" in text
    assert '"points_in_same_xy_accepted_after_height_filter"' in text
    assert '"points_in_box_xyz_accepted_after_height_filter"' in text
    assert '"base_frame_forward_profile"' in text
    assert "MultiThreadedExecutor(num_threads=3)" in text
    assert "self.create_publisher" not in text
    assert 'self.declare_parameter("costmap_frame", "odom_chassis")' in text
    assert 'self.declare_parameter("min_obstacle_height_m", 0.10)' in text
    assert 'self.declare_parameter("max_obstacle_height_m", 1.60)' in text


def test_canonical_main_bringup_uses_qualified_dual_sensor_authorities():
    text = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert 'mkmini_calibrated_localization.launch.py' in text
    assert "'start_fast_lio': use_fast_lio" in text
    assert "default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'tmini_mk2c.yaml'])" in text
    assert "DeclareLaunchArgument('start_ydlidar', default_value='true'" in text
    assert "'--x', '0.703'" in text
    assert "'--z', '0.1923'" in text
    assert "main_base_link_to_laser_frame" in text
    assert "base_link_to_laser_frame',\n        arguments=['--x', '0'" not in text
    assert "base_link_to_livox_frame" not in text
    assert "scan_to_pointcloud.py" not in text


def test_canonical_main_stationary_gate_uses_production_local_costmap_only():
    text = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert "DeclareLaunchArgument('stationary_integration_gate', default_value='false'" in text
    assert "source['local_costmap']['local_costmap']['ros__parameters']" in text
    assert "main_pipeline_costmap_host" in text
    assert "node_names': ['costmap/costmap']" in text
    mppi_guarded = text.count("stationary_mppi_mock_gate, \"' != 'true' and '\"")
    planner_guarded = text.count("stationary_planner_mock_gate, \"' != 'true' and '\"")
    bt_guarded = text.count("stationary_bt_navigator_mock_gate, \"' != 'true' and '\"")
    mission_guarded = text.count("stationary_mission_manager_mock_gate, \"' != 'true'\"")
    assert mppi_guarded == 4  # Nav2, production CM, its lifecycle manager, preview publisher.
    assert planner_guarded == 4
    assert bt_guarded == 4
    assert mission_guarded == 4
    assert "OpaqueFunction(function=stationary_local_costmap_actions)" in text


def test_canonical_collision_monitor_uses_dual_real_sources_and_physical_zones():
    config = yaml.safe_load((ROOT / "config" / "collision_monitor_dual_sensor.yaml").read_text())
    p = config["collision_monitor"]["ros__parameters"]
    assert p["base_frame_id"] == "base_footprint"
    assert p["odom_frame_id"] == "odom"
    assert p["cmd_vel_in_topic"] == "/cmd_vel_nav"
    assert p["cmd_vel_out_topic"] == "/cmd_vel"
    assert p["source_timeout"] == 0.5
    assert p["polygons"] == ["Phase5Stop", "Phase5Slow"]
    assert p["Phase5Stop"]["points"] == [0.785, 0.53, 1.05, 0.53, 1.05, -0.53, 0.785, -0.53]
    assert p["Phase5Slow"]["points"] == [0.785, 0.53, 1.35, 0.53, 1.35, -0.53, 0.785, -0.53]
    assert p["Phase5Stop"]["action_type"] == "stop"
    assert p["Phase5Slow"]["action_type"] == "slowdown"
    assert p["Phase5Slow"]["slowdown_ratio"] == 0.30
    assert p["Phase5Stop"]["max_points"] == p["Phase5Slow"]["max_points"] == 3
    assert p["observation_sources"] == ["mid360_body_cloud", "tmini_scan"]
    assert p["mid360_body_cloud"] == {
        "type": "pointcloud", "topic": "/cloud_registered_body",
        "min_height": 0.10, "max_height": 1.60, "enabled": True,
    }
    assert p["tmini_scan"] == {"type": "scan", "topic": "/scan", "enabled": True}


def test_stationary_collision_monitor_gate_is_hard_topic_isolated():
    text = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert "DeclareLaunchArgument('stationary_collision_monitor_gate', default_value='false'" in text
    assert "params['cmd_vel_in_topic'] = '/phase5/cm_test/cmd_vel_in'" in text
    assert "params['cmd_vel_out_topic'] = '/phase5/cm_test/cmd_vel_out'" in text
    assert "lifecycle_manager_stationary_collision_monitor" in text
    assert "source['collision_monitor']['ros__parameters']" in text
    assert "p5a_mk2d1_collision_monitor_isolated.yaml" in text
    assert "OpaqueFunction(function=stationary_collision_monitor_actions)" in text
    # Only the explicitly requested stationary dry-run mode owns the mock-only
    # labmate-derived backend; the temporary MK-mini adapter is absent.
    assert "stationary_command_chain_dry_run" in text
    assert "executable='gate_to_labmate_bridge'" in text
    assert "executable='wheelchair_controller_node'" in text
    assert "'output_transport': 'mock'" in text
    assert "mkmini_command_chain_dry_run_node" not in text
    assert "SocketCAN" not in text
    assert "output_topic: /phase5/gate_test/cmd_vel_safe" in (ROOT.parent / "vehicle_cmd_safety" / "config" / "phase5_dual_fail_close_gate.yaml").read_text()


def test_stationary_fail_close_gate_requires_both_real_sources_and_is_isolated():
    text = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert "DeclareLaunchArgument('stationary_fail_close_gate', default_value='false'" in text
    assert text.count("executable='collision_monitor_validity_monitor'") == 2
    assert "executable='required_perception_validity'" in text
    assert "executable='guarded_vehicle_cmd_gate'" in text
    safety = ROOT.parent / "vehicle_cmd_safety" / "config"
    assert "/cloud_registered_body" in (safety / "phase5_dual_mid360_validity.yaml").read_text()
    assert "/scan" in (safety / "phase5_dual_tmini_validity.yaml").read_text()
    gate = (safety / "phase5_dual_fail_close_gate.yaml").read_text()
    assert "safe_input_topic: /phase5/cm_test/cmd_vel_out" in gate
    assert "output_topic: /phase5/gate_test/cmd_vel_safe" in gate
