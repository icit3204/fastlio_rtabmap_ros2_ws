from pathlib import Path
def test_control_is_default_off_and_gated():
 text=(Path(__file__).parents[1]/"wheelchair_cmd_adapter/mock_wheelchair_cmd_adapter.py").read_text()
 assert 'declare_parameter("qualification_health_control_enabled", False)' in text
 assert 'declare_parameter("qualification_force_invalid", False)' in text
 assert "qualification control is disabled" in text
 assert "(0.0, 0.0, 0.0)" in text and "P4E6B_QUALIFICATION_INVALID" in text
