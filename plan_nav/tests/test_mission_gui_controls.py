from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def _source(relative):
    return (REPO / relative).read_text(encoding='utf-8')


def test_normal_sidebar_exposes_route_intent_not_runtime_supervision():
    source = _source('plan_nav/ui/sidebar.py')
    assert 'Publish RouteMission' in source
    assert 'mission_publish_requested' in source
    for forbidden in ('Start Mission', 'Pause', 'Resume', 'Cancel', 'UDP', 'legacy', 'mission_nav2', 'authority_mode_changed'):
        assert forbidden not in source


def test_main_window_route_selection_has_no_active_runtime_transport_binding():
    source = _source('plan_nav/ui/main_window.py')
    setup = source[source.index('    def _setup_ui'):source.index('    def _setup_shortcuts')]
    route_click = source[source.index("        elif tool == 'plan':"):source.index("        elif tool == 'delete_edge':")]
    assert 'mission_publish_requested.connect' in setup
    for forbidden in ('mode_changed.connect', 'mission_start_requested', 'mission_pause_requested', 'mission_resume_requested', 'mission_cancel_requested'):
        assert forbidden not in setup
    assert '_prepare_mission_route(path_ids)' in route_click
    for forbidden in ('_start_udp_send', '_plan_publisher', '_start_mission_requested', '/plan_nav'):
        assert forbidden not in route_click


def test_route_bridge_has_no_runtime_service_or_state_authority():
    source = _source('plan_nav/core/mission_bridge.py')
    assert 'pyqtSignal' in source
    assert '"/mission/route"' in source
    for forbidden in ('/mission/start', '/mission/pause', '/mission/cancel', '/mission/state', 'create_client', 'ActionClient'):
        assert forbidden not in source


def test_no_default_motion_or_network_authority_in_gui_modules():
    combined = '\n'.join(_source(path) for path in (
        'plan_nav/ui/sidebar.py', 'plan_nav/ui/log_panel.py', 'plan_nav/core/mission_bridge.py'
    ))
    for forbidden in ('UdpSender', 'socket.socket', '/cmd_vel_nav', '/cmd_vel', '/vehicle_cmd_safe', '/wheelchair_control_command', 'SocketCAN', 'can0'):
        assert forbidden not in combined


def test_legacy_modules_are_retained_but_not_imported_by_normal_gui():
    assert (REPO / 'plan_nav/core/udp_sender.py').exists()
    assert (REPO / 'plan_nav/core/nav_publisher.py').exists()
    main_source = _source('plan_nav/ui/main_window.py')
    assert 'from core.udp_sender import UdpSender' not in main_source
    assert 'from core.nav_publisher import PlanPublisher' not in main_source
    assert "self.log('历史 /plan_nav 运行时发布已隔离'" in main_source


def test_detection_settings_remain_authoring_only():
    panel = _source('plan_nav/ui/log_panel.py')
    assert 'detector_config_changed' in panel
    assert 'yes_conf_input' in panel
    assert 'axis_tol_input' in panel
    assert 'pursuit_' not in panel
    assert 'UDP' not in panel
