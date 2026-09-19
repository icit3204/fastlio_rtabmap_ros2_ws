from lifecycle_msgs.msg import State

from vehicle_cmd_safety.nav2_controller_validity_monitor import ControllerLifecycleHealth


def test_only_fresh_active_controller_state_is_valid():
    health = ControllerLifecycleHealth(timeout_sec=0.5)
    health.observe(state_id=State.PRIMARY_STATE_ACTIVE, state_label="active", now=1.0)
    assert health.valid(1.49)
    assert not health.valid(1.501)


def test_inactive_or_failed_lifecycle_query_fails_closed():
    health = ControllerLifecycleHealth(timeout_sec=0.5)
    health.observe(state_id=State.PRIMARY_STATE_INACTIVE, state_label="inactive", now=1.0)
    assert not health.valid(1.01)
    health.observe(state_id=State.PRIMARY_STATE_ACTIVE, state_label="active", now=2.0)
    assert health.valid(2.1)
    health.query_failed()
    assert not health.valid(2.1)


def test_state_id_and_label_must_both_report_active():
    health = ControllerLifecycleHealth(timeout_sec=0.5)
    health.observe(state_id=State.PRIMARY_STATE_INACTIVE, state_label="active", now=1.0)
    assert not health.valid(1.01)
    health.observe(state_id=State.PRIMARY_STATE_ACTIVE, state_label="inactive", now=2.0)
    assert not health.valid(2.01)


def test_future_or_invalid_response_time_is_not_valid():
    health = ControllerLifecycleHealth(timeout_sec=0.5)
    health.observe(state_id=State.PRIMARY_STATE_ACTIVE, state_label="active", now=3.0)
    assert not health.valid(2.99)
    health.observe(state_id=State.PRIMARY_STATE_ACTIVE, state_label="", now=4.0)
    assert not health.valid(4.0)


def test_query_failure_after_an_unanswered_request_revokes_authority():
    """The monitor must fail closed, then allow a later fresh query to recover."""
    health = ControllerLifecycleHealth(timeout_sec=0.5)
    health.observe(state_id=State.PRIMARY_STATE_ACTIVE, state_label="active", now=1.0)
    assert health.valid(1.1)
    health.query_failed()
    assert not health.valid(1.2)
    health.observe(state_id=State.PRIMARY_STATE_ACTIVE, state_label="active", now=1.3)
    assert health.valid(1.4)
