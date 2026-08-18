"""Static contract for the fake-only P4-E 30-second STOP profile.

The policy constants below are copied explicitly from the accepted P4-E.0B
contract; they are not loaded from an unrelated runtime YAML.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

import yaml


PKG = Path(__file__).parents[1]
SRC = PKG.parent
P4B = PKG / "config" / "phase4_p4b_collision_monitor_scan.yaml"
P4E = PKG / "config" / "phase4_p4e_collision_monitor_scan.yaml"
P4B_SHA256 = "7df5102534885fbdd37d3ba0707e29da29d21d65793337405ddc6280d6ab52d0"
PERSISTENT_BLOCK_SEC = 20.0
CANCEL_RESPONSE_SEC = 2.0
CANCEL_RESULT_SEC = 5.0


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def document(path): return yaml.safe_load(path.read_text(encoding="utf-8"))
def params(path): return document(path)["collision_monitor"]["ros__parameters"]


def differing_paths(left, right, prefix=()):
    if isinstance(left, dict) and isinstance(right, dict):
        result=[]
        for key in sorted(set(left)|set(right)):
            if key not in left or key not in right: result.append(prefix+(key,))
            else: result.extend(differing_paths(left[key],right[key],prefix+(key,)))
        return result
    if isinstance(left, list) and isinstance(right, list):
        if len(left)!=len(right): return [prefix]
        result=[]
        for index,(a,b) in enumerate(zip(left,right)): result.extend(differing_paths(a,b,prefix+(index,)))
        return result
    return [] if left==right else [prefix]


def test_p4b_profile_exists(): assert P4B.is_file()
def test_p4b_profile_sha_is_frozen(): assert digest(P4B)==P4B_SHA256
def test_p4b_timeout_remains_two_seconds(): assert params(P4B)["stop_pub_timeout"]==2.0
def test_p4e_profile_exists(): assert P4E.is_file()
def test_p4e_timeout_is_thirty_seconds(): assert params(P4E)["stop_pub_timeout"]==30.0
def test_both_timeouts_are_finite_positive(): assert all(math.isfinite(v) and v>0 for v in (params(P4B)["stop_pub_timeout"],params(P4E)["stop_pub_timeout"]))
def test_normalized_profiles_differ_at_exactly_one_path():
    assert differing_paths(document(P4B),document(P4E))==[("collision_monitor","ros__parameters","stop_pub_timeout")]
def test_topics_frames_sources_polygons_ratios_and_thresholds_match():
    left=params(P4B).copy(); right=params(P4E).copy(); left.pop("stop_pub_timeout"); right.pop("stop_pub_timeout"); assert left==right
def test_p4b_launch_selects_only_p4b_profile():
    text=(PKG/"launch"/"phase4_p4b_collision_monitor.launch.py").read_text(); assert "phase4_p4b_collision_monitor_scan.yaml" in text and "phase4_p4e_collision_monitor_scan.yaml" not in text
def test_p4e_launch_selects_only_p4e_profile():
    text=(PKG/"launch"/"phase4_p4e1b_clear_full_chain.launch.py").read_text(); assert "phase4_p4e_collision_monitor_scan.yaml" in text and "phase4_p4b_collision_monitor_scan.yaml" not in text
def test_p4e_launch_has_no_timeout_dictionary_override(): assert "stop_pub_timeout" not in (PKG/"launch"/"phase4_p4e1b_clear_full_chain.launch.py").read_text()
def test_setup_installs_all_config_yaml(): assert 'glob("config/*.yaml")' in (PKG/"setup.py").read_text()
def test_source_p4e_profile_parses(): assert isinstance(document(P4E),dict)
def test_installed_contract_expected_value_is_explicit(): assert params(P4E)["stop_pub_timeout"]==30.0
def test_source_profile_digest_is_stable_and_nonempty(): assert len(digest(P4E))==64 and P4E.stat().st_size>0
def test_validity_monitor_expectations_remain_compatible():
    valid=yaml.safe_load((SRC/"vehicle_cmd_safety"/"config"/"phase4_p4c_collision_validity_scan.yaml").read_text())["collision_monitor_validity_monitor"]["ros__parameters"]
    p=params(P4E); assert valid["collision_monitor_node_name"]=="collision_monitor"; assert valid["expected_frame"]==p["base_frame_id"]; assert valid["expected_observation_source_name"] in p["observation_sources"]; assert valid["expected_observation_source_type"]==p["scan"]["type"]; assert valid["expected_observation_source_topic"]==p["scan"]["topic"]; assert valid["source_freshness_sec"]==p["source_timeout"]
def test_no_second_safe_publisher_is_introduced():
    text=(PKG/"launch"/"phase4_p4e1b_clear_full_chain.launch.py").read_text(); assert text.count('package="nav2_collision_monitor"')==1 and "/cmd_vel_nav_safe" not in text
def test_no_synthetic_command_zero_publisher_is_introduced():
    fixture=(PKG/"parking_robot_bringup"/"phase4_p4b_synthetic_obstacles.py").read_text(); assert 'create_publisher(Twist' not in fixture and "/cmd_vel_nav_safe" not in fixture
def test_generic_gate_timeout_remains_quarter_second():
    gate=yaml.safe_load((SRC/"vehicle_cmd_safety"/"config"/"phase4_p4c_gate_mock.yaml").read_text())["guarded_vehicle_cmd_gate"]["ros__parameters"]; assert gate["safe_twist_timeout_sec"]==0.25
def test_shared_p4b_tests_still_freeze_two_seconds(): assert 'assert params["stop_pub_timeout"] == 2.0' in (PKG/"test"/"test_phase4_p4b_collision_monitor_contract.py").read_text()
def test_thirty_second_budget_covers_frozen_contract(): assert 30.0 >= PERSISTENT_BLOCK_SEC+CANCEL_RESPONSE_SEC+CANCEL_RESULT_SEC
def test_thirty_second_budget_has_exact_three_second_margin(): assert 30.0-(PERSISTENT_BLOCK_SEC+CANCEL_RESPONSE_SEC+CANCEL_RESULT_SEC)==3.0
