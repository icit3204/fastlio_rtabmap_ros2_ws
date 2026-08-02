from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch" / "phase2_core_nav2.launch.py"
PARAMS = PKG / "config" / "phase2_nav2_params.yaml"
PACKAGE_XML = PKG / "package.xml"

EXCLUDED = [
    "semantic_grid_tools",
    "/ground_cloud",
    "/obstacle_cloud",
    "/semantic_grid",
    "Collision Monitor",
    "collision_monitor",
    "Generic Command Safety Gate",
    "/vehicle_cmd_safe",
    "Wheelchair Command Adapter",
    "/wheelchair_control_command",
    "wheelchair_controller_node",
    "can0",
    "SocketCAN",
    "plan_nav",
    "Pure Pursuit",
    "pure_pursuit",
    "plan_nav_laser_avoidance",
    "laser_command_safety_filter",
    "Livox",
    "MID-360",
    "FAST-LIO",
    "fast_lio",
    "RTAB-Map live",
    "rtabmap",
    "camera",
    "YDLIDAR",
    "ydlidar",
    "ultrasonic",
    "keepout",
    "speed_filter",
    "Gazebo",
    "Mission Manager",
]


def test_package_xml_parses():
    ET.parse(PACKAGE_XML)


def test_no_excluded_scope_terms_in_launch_or_params():
    text = LAUNCH.read_text() + "\n" + PARAMS.read_text()
    for term in EXCLUDED:
        assert term not in text


def test_only_mock_velocity_topic_is_used():
    runner = PKG / "parking_robot_bringup" / "phase2_sequence_cancel_test_runner.py"
    text = LAUNCH.read_text() + "\n" + PARAMS.read_text() + "\n" + runner.read_text()
    assert "/cmd_vel_phase2_mock" in text
    for forbidden in ["/wheelchair_control_command", "/vehicle_cmd_safe", "/cmd_vel_nav"]:
        assert forbidden not in text


def test_costmaps_have_static_and_inflation_without_observation_sources():
    params = yaml.safe_load(PARAMS.read_text())
    for key in ("global_costmap", "local_costmap"):
        ros_params = params[key][key]["ros__parameters"]
        assert "static_layer" in ros_params["plugins"]
        assert "inflation_layer" in ros_params["plugins"]
        assert "obstacle_layer" not in ros_params["plugins"]
        assert "voxel_layer" not in ros_params["plugins"]
        assert "observation_sources" not in yaml.dump(ros_params)


def test_use_sim_time_defaults_false_and_package_share_defaults():
    params = yaml.safe_load(PARAMS.read_text())
    assert params["map_server"]["ros__parameters"]["use_sim_time"] is False
    assert params["planner_server"]["ros__parameters"]["use_sim_time"] is False
    assert params["controller_server"]["ros__parameters"]["use_sim_time"] is False
    launch_text = LAUNCH.read_text()
    assert 'DeclareLaunchArgument("use_sim_time", default_value="false")' in launch_text
    assert 'FindPackageShare("parking_robot_bringup")' in launch_text
    assert "PathJoinSubstitution" in launch_text
    assert 'DeclareLaunchArgument("start_fake_base", default_value="true")' in launch_text


def test_plugin_identifiers_are_expected_phase2_set():
    params = yaml.safe_load(PARAMS.read_text())
    dumped = yaml.dump(params)
    expected = [
        "nav2_navfn_planner/NavfnPlanner",
        "nav2_mppi_controller::MPPIController",
        "nav2_costmap_2d::StaticLayer",
        "nav2_costmap_2d::InflationLayer",
        "nav2_behaviors/Spin",
        "nav2_behaviors/BackUp",
        "nav2_behaviors/Wait",
    ]
    for plugin in expected:
        assert plugin in dumped
    forbidden_plugins = ["ObstacleLayer", "VoxelLayer", "KeepoutFilter", "SpeedFilter"]
    for plugin in forbidden_plugins:
        assert plugin not in dumped


def test_candidate_c_mppi_profile_is_tracked_exactly():
    params = yaml.safe_load(PARAMS.read_text())
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    critics = follow_path["critics"]
    assert critics.count("PathAngleCritic") == 1
    assert "PreferForwardCritic" not in critics
    assert "ConstraintCritic" not in critics

    assert follow_path["regenerate_noises"] is True
    assert follow_path["time_steps"] == 40
    assert follow_path["model_dt"] == 0.05
    assert follow_path["batch_size"] == 500
    assert follow_path["iteration_count"] == 1
    assert follow_path["vx_std"] == 0.10
    assert follow_path["vy_std"] == 0.0
    assert follow_path["wz_std"] == 0.20
    assert follow_path["temperature"] == 0.40
    assert follow_path["gamma"] == 0.02

    assert "GoalAngleCritic" not in follow_path
    assert follow_path["PathAngleCritic"] == {
        "cost_power": 1,
        "cost_weight": 2.2,
        "threshold_to_consider": 0.5,
        "offset_from_furthest": 4,
        "max_angle_to_furthest": 1.0,
        "forward_preference": True,
    }


def test_candidate_c_does_not_change_goal_or_progress_checker():
    params = yaml.safe_load(PARAMS.read_text())
    controller = params["controller_server"]["ros__parameters"]
    assert controller["progress_checker"] == {
        "plugin": "nav2_controller::SimpleProgressChecker",
        "required_movement_radius": 0.10,
        "movement_time_allowance": 20.0,
    }
    assert controller["general_goal_checker"] == {
        "plugin": "nav2_controller::SimpleGoalChecker",
        "stateful": True,
        "xy_goal_tolerance": 0.25,
        "yaw_goal_tolerance": 0.50,
    }


def test_p2e_runner_console_entry_is_installed_once():
    setup_text = (PKG / "setup.py").read_text()
    entry = "phase2_sequence_cancel_test_runner = parking_robot_bringup.phase2_sequence_cancel_test_runner:main"
    assert setup_text.count(entry) == 1


def test_p2f_runner_console_entry_is_installed_once():
    setup_text = (PKG / "setup.py").read_text()
    entry = "phase2_failure_test_runner = parking_robot_bringup.phase2_failure_test_runner:main"
    assert setup_text.count(entry) == 1


def test_bt_navigator_has_required_humble_default_tree_plugins():
    params = yaml.safe_load(PARAMS.read_text())
    bt_params = params["bt_navigator"]["ros__parameters"]

    assert (
        bt_params["default_nav_to_pose_bt_xml"]
        == "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    )
    assert (
        bt_params["default_nav_through_poses_bt_xml"]
        == "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"
    )

    libs = set(bt_params["plugin_lib_names"])
    required_humble_default_bt_libs = {
        "nav2_compute_path_to_pose_action_bt_node",
        "nav2_navigate_to_pose_action_bt_node",
        "nav2_compute_path_through_poses_action_bt_node",
        "nav2_navigate_through_poses_action_bt_node",
        "nav2_remove_passed_goals_action_bt_node",
        "nav2_follow_path_action_bt_node",
        "nav2_clear_costmap_service_bt_node",
        "nav2_spin_action_bt_node",
        "nav2_wait_action_bt_node",
        "nav2_back_up_action_bt_node",
        "nav2_goal_updated_condition_bt_node",
        "nav2_rate_controller_bt_node",
        "nav2_recovery_node_bt_node",
        "nav2_pipeline_sequence_bt_node",
        "nav2_round_robin_node_bt_node",
    }
    assert required_humble_default_bt_libs <= libs


def test_launch_node_packages_are_explicit_phase2_set():
    text = LAUNCH.read_text()
    packages = set(re.findall(r'package="([^"]+)"', text))
    assert packages == {
        "nav2_map_server",
        "tf2_ros",
        "parking_robot_bringup",
        "nav2_planner",
        "nav2_controller",
        "nav2_behaviors",
        "nav2_bt_navigator",
        "nav2_lifecycle_manager",
    }


def test_start_fake_base_false_omits_only_fake_base():
    text = LAUNCH.read_text()
    assert "IfCondition(start_fake_base)" in text
    assert text.count("phase2_fake_base") == 2
    assert text.count("condition=IfCondition(start_fake_base)") == 1
    assert '"odom_topic": "/Odometry"' in text
    assert '"odom_frame": "odom"' in text
    assert '"base_frame": "base_footprint"' in text
    assert 'name="phase2_map_to_odom_static_tf"' in text
    assert 'arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]' in text
