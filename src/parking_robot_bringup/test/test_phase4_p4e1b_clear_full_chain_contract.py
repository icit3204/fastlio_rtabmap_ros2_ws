from pathlib import Path
import ast
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
from parking_robot_bringup.phase4_p4e1b_clear_runner import Runner

PKG = Path(__file__).parents[1]
LAUNCH = PKG / "launch" / "phase4_p4e1b_clear_full_chain.launch.py"


def source():
    return LAUNCH.read_text(encoding="utf-8")


def test_launch_parses_and_has_allowlisted_nodes():
    ast.parse(source())
    text = source()
    for executable in ["map_server", "planner_server", "controller_server", "behavior_server",
                       "bt_navigator", "collision_monitor", "collision_monitor_validity_monitor",
                       "guarded_vehicle_cmd_gate", "mock_wheelchair_cmd_adapter",
                       "phase4_vehicle_cmd_fake_base", "phase4_p4b_synthetic_obstacles"]:
        assert f'executable="{executable}"' in text


def test_exact_tf_ownership_is_encoded():
    text = source()
    assert text.count('name="phase2_map_to_odom_static_tf"') == 1
    assert '["0", "0", "0", "0", "0", "0", "map", "odom"]' in text
    assert "phase2_fake_base" not in text
    assert text.count('executable="phase4_vehicle_cmd_fake_base"') == 1


def test_phase2_assets_are_selected():
    text = source()
    assert "phase2_nav2_params.yaml" in text
    assert "phase2_clean_map.yaml" in text


def test_raw_authority_is_only_official_nav2():
    text = source()
    assert text.count('remappings=[("/cmd_vel", "/cmd_vel_nav_raw")]') == 2
    assert 'executable="controller_server"' in text
    assert 'executable="behavior_server"' in text


def test_fake_base_has_no_bypass_subscription():
    fake = (PKG / "parking_robot_bringup" / "phase4_vehicle_cmd_fake_base.py").read_text()
    config = yaml.safe_load((PKG / "config" / "phase4_p4e1a_fake_base.yaml").read_text())
    assert config["phase4_vehicle_cmd_fake_base"]["ros__parameters"]["input_topic"] == "/vehicle_cmd_safe"
    for forbidden in ["/cmd_vel_nav_raw", "/cmd_vel_nav_safe", '"/cmd_vel"',
                      "/cmd_vel_phase2_mock", "/cmd_vel_nav\""]:
        assert forbidden not in fake


def test_clear_scan_contract_is_frozen():
    text = source()
    assert '"mode": "CLEAR"' in text
    assert '"frame_id": "base_footprint"' in text
    assert '"publish_scan": True' in text
    params = yaml.safe_load((PKG / "config" / "phase4_p4b_collision_monitor_scan.yaml").read_text())
    p = params["collision_monitor"]["ros__parameters"]
    assert p["cmd_vel_in_topic"] == "/cmd_vel_nav_raw"
    assert p["cmd_vel_out_topic"] == "/cmd_vel_nav_safe"
    assert p["scan"]["topic"] == "/phase4/synthetic_scan"


def test_forbidden_production_paths_are_absent():
    text = source().lower()
    for token in ["mission_manager", "plan_nav", "pure_pursuit", "rtab", "fast_lio",
                  "wheelchair_controller", "socketcan", "vcan", "udp"]:
        assert token not in text


def test_humble_subscription_callbacks_are_one_argument():
    tree = ast.parse((PKG / "parking_robot_bringup" / "phase4_p4e1b_clear_runner.py").read_text())
    methods = {n.name: len(n.args.args) - 1 for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.args.args and n.args.args[0].arg == "self"}
    callbacks = []
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute) and n.func.attr == "create_subscription"):
        cb = call.args[2]
        callbacks.append((len(cb.args.args) - len(cb.args.defaults))
                         if isinstance(cb, ast.Lambda) else methods[cb.attr])
    assert len(callbacks) == 12  # Two loop sites expand to five runtime subscriptions.
    assert callbacks == [1] * 12


def test_evidence_callbacks_execute_without_message_info(tmp_path):
    runner = object.__new__(Runner)
    names = ["cmd_vel_nav_raw_timeline.tsv", "vehicle_cmd_safe_timeline.tsv",
             "mock_wheelchair_output_timeline.tsv", "odometry_timeline.tsv",
             "tf_timeline.tsv", "tf_owner_timeline.tsv", "gate_diagnostics.jsonl",
             "gate_state_timeline.tsv", "adapter_diagnostics.jsonl", "fake_base_diagnostics.jsonl"]
    runner.files = {name: (tmp_path / name).open("w", encoding="utf-8") for name in names}
    runner.samples = {topic: [] for topic in ["/cmd_vel_nav_raw", "/vehicle_cmd_safe",
                                              "/wheelchair_control_command_mock", "/Odometry"]}
    runner.tf_samples = {"map->odom": [], "odom->base_footprint": []}
    runner.diagnostic_counts = {"gate": 0, "adapter": 0, "fake_base": 0}
    runner.diagnostic_history = {"gate": [], "adapter": [], "fake_base": []}
    runner.diag_conditions = {}; runner.initial_odom = None; runner.latest_odom = None
    runner.now = lambda: (100, 200); runner.ident = lambda topic: ("gid", "/publisher")
    runner.cmd("/cmd_vel_nav_raw", Twist())
    runner.vehicle(TwistStamped()); runner.mock(Float32MultiArray(data=[0.0, 0.0, 0.0]))
    runner.odom(Odometry())
    transform = TransformStamped(); transform.header.frame_id = "odom"; transform.child_frame_id = "base_footprint"
    runner.tf(TFMessage(transforms=[transform]), False)
    for topic, name in [("/diagnostics", "vehicle_cmd_safety/guarded_vehicle_cmd_gate"),
                        ("/wheelchair_cmd_adapter/diagnostics", "mock_wheelchair_cmd_adapter"),
                        ("/phase4_fake_base/diagnostics", "phase4_vehicle_cmd_fake_base")]:
        status = DiagnosticStatus(name=name, message="VALID")
        runner.diagnostics(topic, DiagnosticArray(status=[status]))
    for handle in runner.files.values(): handle.close()
    assert runner.diagnostic_counts == {"gate": 1, "adapter": 1, "fake_base": 1}
    assert len(runner.samples["/Odometry"]) == 1
    assert len(runner.tf_samples["odom->base_footprint"]) == 1
    assert len(runner.samples["/cmd_vel_nav_raw"]) == 1


def test_goal_precedes_arm_and_arm_requires_fresh_safe_evidence():
    text = (PKG / "parking_robot_bringup" / "phase4_p4e1b_clear_runner.py").read_text()
    ordering = "node.start_goal(); interlock=node.wait_fresh_safe_disarmed(); safe_fresh_ns=time.monotonic_ns(); arm_response=node.arm_gate()"
    assert ordering in text
    start = text.index("def wait_fresh_safe_disarmed")
    end = text.index("def wait_goal_done", start)
    precondition = text[start:end]
    assert "len(raw)<3 or len(safe)<3" in precondition
    assert "time.monotonic_ns()-safe[-1][0]>250_000_000" in precondition
    assert 'fields.get("state")!="DISARMED"' in precondition
    assert 'fields.get("fault_latched")!="false"' in precondition
    assert "gate bypass while disarmed" in precondition


def test_action_and_arm_events_are_fsynced_immediately():
    text = (PKG / "parking_robot_bringup" / "phase4_p4e1b_clear_runner.py").read_text()
    assert "os.fsync(h.fileno())" in text
    assert "self.persist_event(request)" in text
    assert "self.persist_event(response)" in text
    assert "self.persist_event(event,arm=True)" in text
    assert "arm retry forbidden" in text
    assert "goal resend forbidden" in text


def test_readiness_rate_window_is_at_least_four_seconds():
    text = (PKG / "parking_robot_bringup" / "phase4_p4e1b_clear_runner.py").read_text()
    assert "window_start=time.monotonic_ns(); start_pose=node.latest_odom; node.spin(4.0)" in text
