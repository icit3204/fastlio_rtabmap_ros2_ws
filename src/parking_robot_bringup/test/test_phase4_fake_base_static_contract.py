from pathlib import Path
import yaml

ROOT = Path(__file__).parents[1]
NODE = ROOT / "parking_robot_bringup" / "phase4_vehicle_cmd_fake_base.py"
FORBIDDEN = ["/cmd_vel_nav_raw", "/cmd_vel_nav_safe", "/cmd_vel", "/cmd_vel_phase2_mock",
             "/cmd_vel_nav", "/wheelchair_control_command_mock",
             "/wheelchair_control_command", "/wheelchair_control_command_raw"]

def test_production_has_no_forbidden_command_topic_or_command_publisher():
    text = NODE.read_text()
    assert all(topic not in text for topic in FORBIDDEN)
    assert "create_publisher(Twist" not in text
    assert text.count("create_subscription(TwistStamped") == 1

def test_retained_explicit_steady_timers_and_no_header_deadman():
    text = NODE.read_text()
    assert text.count("Clock(clock_type=ClockType.STEADY_TIME)") == 3
    assert "clock=self._integration_clock" in text
    assert "clock=self._authority_clock" in text
    assert "msg.header.stamp" not in text

def test_frames_outputs_and_launch_is_component_only():
    text = NODE.read_text()
    assert '"odom"' in text and '"base_footprint"' in text
    launch = (ROOT / "launch" / "phase4_p4e1a_fake_base.launch.py").read_text()
    assert launch.count("Node(") == 1
    assert all(word not in launch.lower() for word in ["nav2", "collision_monitor", "mission", "adapter"])

def test_reviewed_yaml_contract():
    params = yaml.safe_load((ROOT / "config" / "phase4_p4e1a_fake_base.yaml").read_text())["phase4_vehicle_cmd_fake_base"]["ros__parameters"]
    assert params["input_topic"] == "/vehicle_cmd_safe"
    assert params["publish_rate_hz"] == 50.0
    assert params["command_timeout_sec"] == .5
    assert params["max_integration_dt_sec"] == .1

def test_phase2_files_are_git_baseline_unchanged():
    import subprocess
    for rel in ["parking_robot_bringup/phase2_fake_base.py", "parking_robot_bringup/phase2_fake_base_math.py"]:
        current = (ROOT / rel).read_bytes()
        baseline = subprocess.check_output(["git", "show", f"HEAD:src/parking_robot_bringup/{rel}"], cwd=ROOT)
        assert current == baseline
