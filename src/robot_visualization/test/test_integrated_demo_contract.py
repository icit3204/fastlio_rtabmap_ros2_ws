from pathlib import Path
from types import SimpleNamespace

from robot_visualization.integrated_demo_fixture import IntegratedDemoFixture


def _fixture_state(name='RECEIVED'):
    return SimpleNamespace(
        _state_name=name,
        _normal_gate=('ARMED_COMMAND', 'NONE'),
        _gate_state='ARMED_COMMAND',
        _gate_reason='NONE',
    )


def test_demo_service_state_sequence_is_authoritative_and_bounded():
    fixture = _fixture_state()
    response = SimpleNamespace(success=False, message='')
    assert IntegratedDemoFixture._start(fixture, None, response).success
    assert fixture._state_name == 'NAVIGATING'

    response = SimpleNamespace(success=False, message='')
    assert IntegratedDemoFixture._pause(fixture, SimpleNamespace(data=True), response).success
    assert fixture._state_name == 'PAUSED'
    assert fixture._gate_state == 'DISARMED_ZERO'

    response = SimpleNamespace(success=False, message='')
    assert IntegratedDemoFixture._pause(fixture, SimpleNamespace(data=False), response).success
    assert fixture._state_name == 'NAVIGATING'

    response = SimpleNamespace(success=False, message='')
    assert IntegratedDemoFixture._cancel(fixture, None, response).success
    assert fixture._state_name == 'CANCELLED'
    assert fixture._gate_reason == 'MISSION_CANCELLED_DEMO'


def test_demo_rejects_invalid_service_transitions():
    fixture = _fixture_state('PAUSED')
    response = SimpleNamespace(success=True, message='')
    assert not IntegratedDemoFixture._start(fixture, None, response).success
    response = SimpleNamespace(success=True, message='')
    assert not IntegratedDemoFixture._pause(fixture, SimpleNamespace(data=True), response).success


def test_integrated_fixture_has_no_motion_or_navigation_authority():
    source = (Path(__file__).parents[1] / 'robot_visualization' / 'integrated_demo_fixture.py').read_text()
    for forbidden in ('/cmd_vel_nav', "'/cmd_vel'", '/wheelchair_control_command', 'ActionClient', 'socket('):
        assert forbidden not in source
    assert '/vehicle_cmd_safe' in source  # Explicit DEMO-only status input.
    for service in ('/mission/start', '/mission/pause', '/mission/cancel'):
        assert service in source


def test_canonical_rviz_keeps_candidates_hidden_while_fixture_publishes_them():
    rviz = (Path(__file__).parents[1] / 'config' / 'canonical_navigation.rviz').read_text()
    fixture = (Path(__file__).parents[1] / 'robot_visualization' / 'integrated_demo_fixture.py').read_text()
    assert 'Optimal Trajectory: true' in rviz
    assert 'Candidate Trajectories: false' in rviz
    assert "'Candidate Trajectories'" in fixture
