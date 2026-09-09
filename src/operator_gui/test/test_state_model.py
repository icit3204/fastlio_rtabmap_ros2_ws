from types import SimpleNamespace

from operator_gui.state_model import GuiStateModel, button_enabled, goal_status_label


def pose(x, y=0.0):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))


def route(mission_id='m1'):
    return SimpleNamespace(
        mission_id=mission_id, route_id='r1', node_ids=['A', 'B', 'C'],
        edge_ids=['A-B', 'B-C'], poses=[pose(0), pose(1), pose(2)])


def mission(state=1, index=0, mission_id='m1'):
    return SimpleNamespace(
        mission_id=mission_id, route_id='r1', state=state,
        current_waypoint_index=index, completed_waypoint_count=index,
        total_waypoint_count=3, progress=index / 3.0, reason_code='')


def model_at(state=1, index=0):
    model = GuiStateModel()
    model.update_route(route(), 0.0)
    model.update_mission(mission(state, index), 0.0)
    model.update_bool('controller_valid', True, 0.0)
    model.update_bool('perception_valid', True, 0.0)
    model.update_bool('localization_valid', True, 0.0)
    model.update_safe_speed(0.25, 0.0)
    return model


def test_mission_join_and_progress():
    view = model_at(index=1).view(0.1)
    assert view.mission_id == 'm1'
    assert view.start_node == 'A'
    assert view.goal_node == 'C'
    assert view.current_edge == 'B-C'
    assert view.current_goal.startswith('B (')
    assert view.progress == '1 / 3 (33%)'


def test_route_mismatch_and_invalid_index_are_safe():
    model = model_at(index=9)
    view = model.view(0.1)
    assert view.current_goal == 'UNKNOWN'
    assert view.current_edge == '—'
    model.update_route(route('other'), 0.2)
    assert model.view(0.3).route_ready is False


def test_stale_status_is_not_presented_as_current():
    view = model_at().view(3.0)
    assert view.mission_state == 'STALE'
    assert view.controller_valid == 'STALE'
    assert view.safe_speed == 'STALE'


def test_gate_reason_and_fault_are_separate():
    model = model_at(state=4)
    model.update_gate(SimpleNamespace(message='SAFE_TWIST_STALE', values=[SimpleNamespace(key='state', value='FAULT')]), 0.0)
    view = model.view(0.1)
    assert view.gate_state == 'FAULT'
    assert view.gate_reason == 'SAFE_TWIST_STALE'
    assert view.fault_reason == 'SAFE_TWIST_STALE'


def test_gate_and_action_status_become_unknown_when_stale():
    model = model_at(state=4)
    model.update_gate(SimpleNamespace(message='ARMED_COMMAND', values=[]), 0.0)
    model.update_nav_status(2, 0.0)
    view = model.view(2.0)
    assert view.gate_state == 'STALE'
    assert view.gate_reason == 'STALE'
    assert view.nav2_state == 'ACTIVE'


def test_buttons_follow_mission_contract_and_unknown_disables():
    view = model_at(state=4).view(0.1)
    assert button_enabled(view, 'pause')
    assert button_enabled(view, 'cancel')
    assert not button_enabled(view, 'start')
    paused = model_at(state=5).view(0.1)
    assert button_enabled(paused, 'resume')
    unknown = GuiStateModel().view(0.1)
    assert not any(button_enabled(unknown, name) for name in ('start', 'pause', 'resume', 'cancel'))


def test_goal_status_labels():
    assert goal_status_label(2) == 'ACTIVE'
    assert goal_status_label(5) == 'CANCELED'
    assert goal_status_label(None) == 'UNKNOWN'


def test_no_reset_control_is_defined():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'operator_gui' / 'main_window.py').read_text()
    assert 'RESET' not in source


def test_gui_has_no_motion_authority_or_navigation_action_client():
    from pathlib import Path
    root = Path(__file__).parents[1] / 'operator_gui'
    # The demo fixture is deliberately allowed to publish observational test
    # data.  Audit only the GUI/runtime modules for motion authority.
    source = '\n'.join(
        p.read_text() for p in root.glob('*.py') if p.name != 'mock_fixture.py'
    )
    assert 'ActionClient' not in source
    assert 'create_publisher' not in source
    assert 'socket(' not in source
