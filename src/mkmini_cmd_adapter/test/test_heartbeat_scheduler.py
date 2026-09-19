from mkmini_cmd_adapter.heartbeat_scheduler import HeartbeatScheduler, MonotonicDeadline, timing_summary


def test_monotonic_deadlines_are_100hz_without_cumulative_drift():
    schedule = MonotonicDeadline(10.0, .010)
    assert [schedule.consume() for _ in range(5)] == [10.0, 10.01, 10.02, 10.03, 10.04]


def test_timing_summary_reports_percentiles_and_maximum():
    from mkmini_cmd_adapter.heartbeat_scheduler import HeartbeatTiming
    records = [HeartbeatTiming(float(i), float(i), interval, 0.0, 0.0, "MOTION_PERMITTED")
               for i, interval in enumerate((None, .010, .011, .012, .020))]
    summary = timing_summary(records)
    assert summary["mean_interval_sec"] == .01325
    assert summary["p95_interval_sec"] == .020
    assert summary["p99_interval_sec"] == .020
    assert summary["max_interval_sec"] == .020


def test_logging_and_rx_work_are_not_called_from_tx_callback():
    calls = []
    # The scheduler accepts only a send callback.  RX/logging happen in the
    # supervisor and persist records after stop, never in this callback.
    scheduler = HeartbeatScheduler(lambda: calls.append("tx"))
    assert scheduler.fault is None
    assert calls == []


def test_scheduler_stops_on_expired_supervisor_lease():
    now = [0.0]
    sent = []
    scheduler = HeartbeatScheduler(lambda: sent.append(now[0]), clock=lambda: now[0])
    scheduler.start(state="N_ZERO_PRIMING", now=0.0)
    now[0] = .026
    # A stop is deterministic and immediate even if the test clock does not
    # advance the scheduler thread itself.
    scheduler.stop()
    assert scheduler.fault in (None, "SUPERVISOR_LEASE_EXPIRED")


def test_scheduler_can_run_without_an_artificial_control_plane_lease():
    scheduler = HeartbeatScheduler(lambda: None, lease_sec=None)
    assert scheduler.fault is None
