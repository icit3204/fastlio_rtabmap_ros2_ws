from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "nav2_common.yaml"
LAUNCH = ROOT / "launch" / "bringup.launch.py"
POSE_BT = ROOT / "config" / "ackermann_navigate_to_pose.xml"
THROUGH_BT = ROOT / "config" / "ackermann_navigate_through_poses.xml"
RUNNER = ROOT / "scripts" / "mk2f3_navigate_to_pose_stationary_runner.py"


def test_ackermann_navigate_to_pose_tree_has_no_incompatible_motion_recovery():
    root = ET.parse(POSE_BT).getroot()
    tags = {element.tag for element in root.iter()}
    assert {"ComputePathToPose", "FollowPath", "ClearEntireCostmap"} <= tags
    assert tags.isdisjoint({"Spin", "BackUp", "DriveOnHeading", "AssistedTeleop", "Wait"})
    follow = next(root.iter("FollowPath"))
    assert follow.attrib["controller_id"] == "FollowPath"
    assert follow.attrib["goal_checker_id"] == "general_goal_checker"
    planner = next(root.iter("ComputePathToPose"))
    assert planner.attrib["planner_id"] == "GridBased"


def test_bt_defaults_and_plugins_fail_safe_for_both_humble_navigators():
    ET.parse(THROUGH_BT)
    params = yaml.safe_load(CONFIG.read_text())["bt_navigator"]["ros__parameters"]
    assert params["default_nav_to_pose_bt_xml"].endswith("/ackermann_navigate_to_pose.xml")
    assert params["default_nav_through_poses_bt_xml"].endswith("/ackermann_navigate_through_poses.xml")
    plugins = set(params["plugin_lib_names"])
    assert {
        "nav2_compute_path_to_pose_action_bt_node",
        "nav2_compute_path_through_poses_action_bt_node",
        "nav2_follow_path_action_bt_node",
        "nav2_clear_costmap_service_bt_node",
        "nav2_goal_updated_condition_bt_node",
        "nav2_rate_controller_bt_node",
        "nav2_recovery_node_bt_node",
        "nav2_pipeline_sequence_bt_node",
    } == plugins
    assert not any(token in plugin for plugin in plugins for token in ("spin", "back_up", "drive_on_heading", "assisted_teleop"))
    behavior = yaml.safe_load(CONFIG.read_text())["behavior_server"]["ros__parameters"]
    assert behavior["behavior_plugins"] == ["wait"]


def test_stationary_bt_mode_owns_real_bt_without_behavior_server_or_can():
    source = LAUNCH.read_text()
    assert "DeclareLaunchArgument('stationary_bt_navigator_mock_gate'" in source
    section = source.split("def stationary_bt_navigator_actions", 1)[1].split("# ── 7.", 1)[0]
    assert "package='nav2_bt_navigator', executable='bt_navigator'" in section
    assert "'node_names': ['bt_navigator']" in section
    assert "ackermann_navigate_to_pose.xml" in section
    assert "behavior_server" in section  # documented as intentionally absent
    assert "package='nav2_behaviors'" not in section
    command = source.split("def stationary_command_chain_actions", 1)[1].split("def stationary_mppi_controller_actions", 1)[0]
    assert "'output_transport': 'mock'" in command
    assert "CANONICAL_MOCK_FORBIDDEN" in command


def test_runner_uses_only_real_navigate_to_pose_orchestration_action():
    source = RUNNER.read_text()
    assert 'ActionClient(self, NavigateToPose, "/navigate_to_pose")' in source
    assert "ComputePathToPose" not in source
    assert "FollowPath.Goal" not in source
    assert 'request.behavior_tree = ""' in source
    assert 'self.create_subscription(Twist, "/cmd_vel_nav"' in source
    assert "create_publisher" not in source
    assert "cancel_goal_async" in source
    assert "SocketCAN" not in source
    assert "can0" not in source
