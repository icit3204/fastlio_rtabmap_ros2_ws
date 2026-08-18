from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "config" / "phase4_p4e_chassis_nav2_params.yaml"
BT_DIR = ROOT / "behavior_trees"
MISSION = ROOT.parent / "parking_robot_mission_manager" / "parking_robot_mission_manager"
FORBIDDEN = {"Spin", "BackUp", "DriveOnHeading"}


def profile():
    return yaml.safe_load(CONFIG.read_text())


def tags(path):
    return {node.tag for node in ET.parse(path).getroot().iter()}


def test_humble_profile_closes_both_hardcoded_navigators():
    params = profile()["bt_navigator"]["ros__parameters"]
    assert params["default_nav_to_pose_bt_xml"] == "phase4_p4e_chassis_navigate_to_pose.xml"
    assert params["default_nav_through_poses_bt_xml"] == "phase4_p4e_chassis_navigate_through_poses.xml"
    for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
        assert (BT_DIR / params[key]).is_file()


def test_every_default_bt_has_satisfied_wait_only_behavior_dependencies():
    params = profile()["bt_navigator"]["ros__parameters"]
    behavior = profile()["behavior_server"]["ros__parameters"]
    assert behavior["behavior_plugins"] == ["wait"]
    for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
        tree_tags = tags(BT_DIR / params[key])
        assert not (tree_tags & FORBIDDEN)
        assert tree_tags & {"Wait"}
        assert tree_tags <= {"root", "BehaviorTree", "RecoveryNode", "PipelineSequence",
                             "RateController", "ComputePathToPose", "ComputePathThroughPoses",
                             "ReactiveSequence", "RemovePassedGoals", "FollowPath",
                             "ClearEntireCostmap", "ReactiveFallback", "GoalUpdated",
                             "RoundRobin", "Sequence", "Wait"}


def test_through_poses_tree_preserves_installed_nonvelocity_structure_and_budget():
    root = ET.parse(BT_DIR / "phase4_p4e_chassis_navigate_through_poses.xml").getroot()
    tree_tags = {n.tag for n in root.iter()}
    assert {"ComputePathThroughPoses", "RemovePassedGoals", "FollowPath",
            "ClearEntireCostmap", "Wait", "GoalUpdated"} <= tree_tags
    outer = next(n for n in root.iter("RecoveryNode") if n.attrib.get("name") == "NavigateRecovery")
    assert outer.attrib["number_of_retries"] == "6"


def test_mission_manager_remains_navigate_to_pose_only():
    source = (MISSION / "mission_manager_node.py").read_text()
    transport = (MISSION / "nav2_goal_executor.py").read_text()
    combined = source + transport
    assert "NavigateToPose" in combined
    assert "NavigateThroughPoses" not in combined


def test_ackermann_and_wait_only_contract_unchanged():
    p = profile()
    follow = p["controller_server"]["ros__parameters"]["FollowPath"]
    assert (follow["motion_model"], follow["vx_min"], follow["vx_max"],
            follow["vy_max"], follow["wz_max"],
            follow["AckermannConstraints"]["min_turning_r"]) == ("Ackermann", 0.0, .2, 0.0, .5, 1.0)
    assert p["behavior_server"]["ros__parameters"]["behavior_plugins"] == ["wait"]
