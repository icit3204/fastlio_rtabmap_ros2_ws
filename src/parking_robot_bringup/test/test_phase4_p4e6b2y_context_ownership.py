from pathlib import Path

WRAPPER = Path(__file__).with_name("phase4_p4e6b2y_h02_exact_wrapper.py")
RUNNER = Path(__file__).parents[1] / "parking_robot_bringup" / "phase4_p4e6b_health_failure_runner.py"

def test_exact_wrapper_does_not_initialize_default_context():
    text = WRAPPER.read_text()
    assert "rclpy.init" not in text

def test_exact_wrapper_delegates_context_ownership_to_execute_campaign_child():
    text = WRAPPER.read_text()
    assert "execute_campaign" in text
    assert "subprocess.Popen" in text

def test_frozen_runner_owns_context_initialization():
    text = RUNNER.read_text()
    assert "rclpy.init(args=ros_args)" in text

def test_wrapper_has_no_h02_attempt_consumption_marker():
    text = WRAPPER.read_text()
    assert "H02_ATTEMPT2_RUNTIME_STARTED" not in text
    assert "B2Y_MATRIX_STARTED" in text

def test_wrapper_has_preinjection_stop_and_zero_injection_guard():
    text = WRAPPER.read_text()
    assert "B2Y_RUNNER_RUNTIME_STATE_MACHINE_ENTERED" in text
    assert 'event") == "INJECTED"' in text
