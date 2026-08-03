from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def test_mission_controls_require_explicit_mission_mode_and_received_state():
    sidebar_source = (REPO / "plan_nav" / "ui" / "sidebar.py").read_text(encoding="utf-8")
    main_source = (REPO / "plan_nav" / "ui" / "main_window.py").read_text(encoding="utf-8")
    assert "addItem('legacy', 'legacy')" in sidebar_source
    assert "addItem('mission_nav2', 'mission_nav2')" in sidebar_source
    assert "mission_mode = self.current_authority_mode() == 'mission_nav2'" in sidebar_source
    assert "self._btn_mission_start.setEnabled(mission_mode and start_ready)" in sidebar_source
    assert "int(msg.state) == 1" in main_source
    assert "_on_publish_mission_requested" in main_source
    assert "_on_start_mission_requested" in main_source


def test_mission_bridge_uses_qt_signals_not_widgets_for_ros_callbacks():
    source = (REPO / "plan_nav" / "core" / "mission_bridge.py").read_text(encoding="utf-8")
    assert "pyqtSignal" in source
    assert "QWidget" not in source
    assert ".setText(" not in source
