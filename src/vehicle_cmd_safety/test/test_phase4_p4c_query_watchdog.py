from vehicle_cmd_safety.collision_monitor_validity_monitor import (
    ServiceQueryTracker,
    parse_parameters_response,
    parse_state_response,
)


class FakeFuture:
    def __init__(self, done=False, result=None):
        self._done = done
        self._result = result
        self.cancelled = False

    def done(self):
        return self._done

    def result(self):
        return self._result

    def cancel(self):
        self.cancelled = True
        return True


def test_never_completing_query_expires_and_replacement_is_non_overlapping():
    tracker = ServiceQueryTracker("state", watchdog_sec=0.25, retry_interval_sec=0.10)
    first = FakeFuture()
    assert tracker.can_issue(1.0)
    generation_one = tracker.begin(first, 1.0)
    assert generation_one == 1
    assert not tracker.can_issue(1.20)
    assert tracker.expire(1.26) is first
    assert first.cancelled
    assert tracker.status == "TIMED_OUT"
    assert not tracker.can_issue(1.35)
    second = FakeFuture()
    assert tracker.can_issue(1.36)
    generation_two = tracker.begin(second, 1.36)
    assert generation_two > generation_one
    assert tracker.future is second


def test_late_old_generation_cannot_replace_new_authoritative_request():
    tracker = ServiceQueryTracker("parameters", watchdog_sec=0.25, retry_interval_sec=0.10)
    old = FakeFuture()
    tracker.begin(old, 1.0)
    tracker.expire(1.30)
    current = FakeFuture()
    current_generation = tracker.begin(current, 1.41)
    old._done = True
    old._result = "stale"
    assert tracker.future is current
    assert tracker.generation == current_generation
    assert tracker.future is not old


def test_state_and_parameter_queries_have_independent_watchdogs_and_retries():
    state = ServiceQueryTracker("state", watchdog_sec=0.25, retry_interval_sec=0.10)
    params = ServiceQueryTracker("parameters", watchdog_sec=0.25, retry_interval_sec=0.10)
    state.begin(FakeFuture(), 1.0)
    params.begin(FakeFuture(), 1.0)
    state.expire(1.30)
    params.expire(1.30)
    assert state.status == "TIMED_OUT"
    assert params.status == "TIMED_OUT"
    assert state.future is None
    assert params.future is None
    assert state.can_issue(1.41)
    assert params.can_issue(1.41)


def test_completed_query_waits_for_retry_interval_before_refresh():
    tracker = ServiceQueryTracker("state", watchdog_sec=0.25, retry_interval_sec=0.10)
    tracker.begin(FakeFuture(done=True), 1.0)
    tracker.complete(1.01)
    assert not tracker.can_issue(1.10)
    assert tracker.can_issue(1.11)


def test_service_reappearance_can_issue_after_waiting_state():
    tracker = ServiceQueryTracker("state", watchdog_sec=0.25, retry_interval_sec=0.10)
    tracker.mark_waiting(1.0)
    assert tracker.can_issue(1.0)
    tracker.begin(FakeFuture(done=True), 1.0)
    tracker.complete(1.01)
    tracker.mark_waiting(1.20)
    assert tracker.can_issue(1.20)


def test_malformed_state_and_parameter_responses_fail_closed():
    try:
        parse_state_response(None)
        assert False
    except ValueError:
        pass
    try:
        parse_parameters_response(None, "scan", "scan", "/phase4/synthetic_scan")
        assert False
    except ValueError:
        pass
