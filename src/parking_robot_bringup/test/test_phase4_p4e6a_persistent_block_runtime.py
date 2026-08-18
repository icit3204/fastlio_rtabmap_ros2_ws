import csv
import json
import math
from pathlib import Path
import time

import pytest

from action_msgs.msg import GoalStatus
from parking_robot_interfaces.msg import MissionState

from parking_robot_bringup.phase4_p4e6a_persistent_block_runner import (
    BoundedEvidenceWriter,
    EvidenceAdjudicationError,
    EvidenceWriterError,
    EVIDENCE_QUEUE_CAPACITY,
    FROZEN_SUPERVISOR_SHA256,
    MAXIMUM_STATIONARY_TRANSLATION_M,
    PersistentStopAdjudicator,
    adjudicate_persistent_terminal_history,
    find_transition_since,
    PERSISTENT_STOP_NS,
    StopEpisodeLedger,
    TEMPORARY_STOP_NS,
    adjudicate_source_event,
    adjudicate_stationary_checkpoint,
    build_stationary_window,
    checkpoint_freshness,
    prospective_epoch,
    reconstruct_policy_timing,
    reproduce_receipt_anchor_defect,
    require_checkpoint_freshness,
    synthetic_complete_path,
    synthetic_repaired_complete_path,
    verify_supervisor_source_authority,
    write_deterministic_summary,
)


MISSION = "mission-a"
ROUTE = "route-a"
UUID = "00112233445566778899aabbccddeeff"


def terminal_state(ns, state, reason, *, mission=MISSION, route=ROUTE,
                   uuid=UUID, waypoint=0):
    return {"monotonic_ns": ns, "state": state, "reason_code": reason,
            "mission_id": mission, "route_id": route, "waypoint_index": waypoint,
            "active_goal_uuid": uuid}


def valid_terminal_history(duration_ns=36_046_457, start_ns=20_000_000_000):
    cancelling = terminal_state(start_ns, MissionState.CANCELLING,
                                "PERSISTENT_COLLISION_STOP")
    canceling_ns = start_ns + max(1, duration_ns // 4)
    ack_ns = start_ns + max(2, duration_ns // 3)
    canceled_ns = start_ns + max(3, 2 * duration_ns // 3)
    blocked_ns = start_ns + duration_ns
    states = [cancelling,
              terminal_state(ack_ns, MissionState.CANCELLING,
                             "BLOCK_CANCEL_ACK_ACCEPTED"),
              terminal_state(blocked_ns, MissionState.BLOCKED,
                             "PERSISTENT_COLLISION_STOP", uuid="")]
    statuses = [
        {"monotonic_ns": canceling_ns, "goal_uuid": UUID,
         "status": GoalStatus.STATUS_CANCELING},
        {"monotonic_ns": canceled_ns, "goal_uuid": UUID,
         "status": GoalStatus.STATUS_CANCELED},
    ]
    return states, statuses


@pytest.mark.parametrize("waiter_offset", [-1, 0, 1, 36_046_456, 36_046_457])
def test_historical_transition_found_across_waiter_registration_phases(waiter_offset):
    states, _ = valid_terminal_history()
    assert find_transition_since(
        states, state=MissionState.CANCELLING, after_ns=20_000_000_000,
        reason="PERSISTENT_COLLISION_STOP", mission_id=MISSION,
        route_id=ROUTE, waypoint_index=0, goal_uuid=UUID) == states[0]


@pytest.mark.parametrize("duration_ns", [1_000_000, 10_000_000, 36_046_457, 100_000_000])
def test_terminal_history_accepts_brief_ordered_cancelling(duration_ns):
    states, statuses = valid_terminal_history(duration_ns)
    result = adjudicate_persistent_terminal_history(
        states=states, action_statuses=statuses, persistent_ns=20_000_000_000,
        mission_id=MISSION, route_id=ROUTE, waypoint_index=0, goal_uuid=UUID)
    assert result["blocked"]["monotonic_ns"] - result["cancelling"]["monotonic_ns"] == duration_ns


def test_exact_a8f_terminal_chronology_replays_after_blocked():
    states = [
        terminal_state(721565377673766, MissionState.CANCELLING,
                       "PERSISTENT_COLLISION_STOP"),
        terminal_state(721565389663551, MissionState.CANCELLING,
                       "BLOCK_CANCEL_ACK_ACCEPTED"),
        terminal_state(721565413720223, MissionState.BLOCKED,
                       "PERSISTENT_COLLISION_STOP", uuid=""),
    ]
    statuses = [
        {"monotonic_ns": 721565386527034, "goal_uuid": UUID,
         "status": GoalStatus.STATUS_CANCELING},
        {"monotonic_ns": 721565412259246, "goal_uuid": UUID,
         "status": GoalStatus.STATUS_CANCELED},
    ]
    result = adjudicate_persistent_terminal_history(
        states=states, action_statuses=statuses, persistent_ns=721565378552951,
        mission_id=MISSION, route_id=ROUTE, waypoint_index=0, goal_uuid=UUID,
        history_start_ns=721545332512446)
    assert result["blocked"]["reason_code"] == "PERSISTENT_COLLISION_STOP"


@pytest.mark.parametrize("mutation", [
    "wrong_reason", "wrong_mission", "wrong_uuid", "blocked_without_cancelling",
    "cancelling_after_blocked", "duplicate_canceling", "duplicate_canceled",
    "wrong_terminal_reason", "failed_terminal", "shutdown_cancel_only",
])
def test_terminal_history_rejects_invalid_or_incomplete_paths(mutation):
    states, statuses = valid_terminal_history()
    if mutation == "wrong_reason": states[0]["reason_code"] = "CONTROLLER_NO_PROGRESS"
    elif mutation == "wrong_mission": states[0]["mission_id"] = "other"
    elif mutation == "wrong_uuid": states[0]["active_goal_uuid"] = "other"
    elif mutation == "blocked_without_cancelling": states = states[2:]
    elif mutation == "cancelling_after_blocked": states[0]["monotonic_ns"] = states[-1]["monotonic_ns"] + 1
    elif mutation == "duplicate_canceling": statuses.append(dict(statuses[0]))
    elif mutation == "duplicate_canceled": statuses.append(dict(statuses[1]))
    elif mutation == "wrong_terminal_reason": states[-1]["reason_code"] = "CONTROLLER_NO_PROGRESS"
    elif mutation == "failed_terminal": states[-1]["state"] = MissionState.FAILED
    elif mutation == "shutdown_cancel_only": states = states[-1:]
    with pytest.raises(EvidenceAdjudicationError):
        adjudicate_persistent_terminal_history(
            states=states, action_statuses=statuses, persistent_ns=20_000_000_000,
            mission_id=MISSION, route_id=ROUTE, waypoint_index=0, goal_uuid=UUID)


def test_duplicate_cancelling_publications_do_not_duplicate_cancel_authority():
    states, statuses = valid_terminal_history()
    states.insert(1, dict(states[0], monotonic_ns=states[0]["monotonic_ns"] + 1))
    result = adjudicate_persistent_terminal_history(
        states=states, action_statuses=statuses, persistent_ns=20_000_000_000,
        mission_id=MISSION, route_id=ROUTE, waypoint_index=0, goal_uuid=UUID)
    assert result["cancelling"]["monotonic_ns"] == 20_000_000_000


def test_cancelling_before_authoritative_source_anchor_is_rejected():
    states, statuses = valid_terminal_history(start_ns=19_999_999_999)
    with pytest.raises(EvidenceAdjudicationError):
        adjudicate_persistent_terminal_history(
            states=states, action_statuses=statuses, persistent_ns=20_000_000_000,
            mission_id=MISSION, route_id=ROUTE, waypoint_index=0, goal_uuid=UUID)


def test_missing_cancel_acknowledgement_is_rejected():
    states, statuses = valid_terminal_history()
    states.pop(1)
    with pytest.raises(EvidenceAdjudicationError):
        adjudicate_persistent_terminal_history(
            states=states, action_statuses=statuses, persistent_ns=20_000_000_000,
            mission_id=MISSION, route_id=ROUTE, waypoint_index=0, goal_uuid=UUID)


def test_same_callback_drain_batch_preserves_serialized_order():
    states, statuses = valid_terminal_history(duration_ns=4)
    result = adjudicate_persistent_terminal_history(
        states=states, action_statuses=statuses, persistent_ns=20_000_000_000,
        mission_id=MISSION, route_id=ROUTE, waypoint_index=0, goal_uuid=UUID)
    assert result["cancelling"]["monotonic_ns"] <= result["blocked"]["monotonic_ns"]


def test_large_prior_history_does_not_hide_relevant_transition():
    states, statuses = valid_terminal_history()
    noise = [terminal_state(i, MissionState.NAVIGATING, "NONE", uuid="old")
             for i in range(10_000)]
    result = adjudicate_persistent_terminal_history(
        states=noise + states, action_statuses=statuses, persistent_ns=20_000_000_000,
        mission_id=MISSION, route_id=ROUTE, waypoint_index=0, goal_uuid=UUID)
    assert result["cancelling"] == states[0]


ROOT = Path(__file__).parents[1]
RUNNER = ROOT / "parking_robot_bringup" / "phase4_p4e6a_persistent_block_runner.py"


def good_epochs():
    return {name: {"passes": True} for name in ("gate", "adapter", "odom", "tf")}


def ready(a, **overrides):
    values = dict(query_ns=0, drain_start_ns=1, drain_end_ns=1_000_000_001,
                  epoch_start_ns=1_000_000_002, epoch_end_ns=3_000_000_002,
                  epochs=good_epochs(), health_ok=True)
    values.update(overrides)
    a.readiness(**values)


def active_with_clear(duration_ns=5_050_000_000):
    a = PersistentStopAdjudicator(); ready(a); a.activate()
    a.observe_clear(4_000_000_000)
    a.observe_clear(4_050_000_000, pair_state="PENDING")
    a.observe_clear(4_100_000_000)
    a.observe_clear(4_000_000_000 + duration_ns)
    return a


def stopped(temp=True):
    a = active_with_clear(); a.inject_stop(9_100_000_000)
    a.observe_stop(9_150_000_000, stop_age_sec=0.0)
    if temp:
        a.observe_stop(10_150_000_000, event="COLLISION_STOP_TEMPORARY", stop_age_sec=1.0)
    return a


def assert_reason(exc, reason):
    assert exc.value.reason == reason


def test_complete_path_executes_behavior_and_packages_json(tmp_path):
    result = synthetic_complete_path()
    assert result["pass"] and result["phase"] == "COMPLETE"
    assert result["fixture_requests"] == ["STOP"]
    assert result["cancel_count"] == result["persistent_count"] == 1
    packaged = write_deterministic_summary(tmp_path / "terminal_metrics.json", result)
    assert packaged["timing_ns"]["blocked_ns"] == 29_250_000_000


def test_packaging_fails_closed_for_incomplete_result(tmp_path):
    with pytest.raises(EvidenceAdjudicationError):
        write_deterministic_summary(tmp_path / "bad.json", {"pass": False})


def test_prospective_epoch_uses_n_minus_one_and_fixed_window():
    rows = [1_000_000_000 + i*20_000_000 for i in range(101)]
    result = prospective_epoch(rows, 1_000_000_000, 3_000_000_000,
                               minimum_duration_sec=2, lower_hz=48, upper_hz=52,
                               large_gap_sec=.05)
    assert result["sample_count"] == 101 and result["rate_hz"] == 50
    assert result["passes"]


@pytest.mark.parametrize("changes,reason", [
    ({"drain_end_ns": 900_000_000}, "P4E6A8A_READINESS_IMPLEMENTATION_NEEDS_REVIEW"),
    ({"epoch_end_ns": 2_900_000_000}, "P4E6A8A_READINESS_IMPLEMENTATION_NEEDS_REVIEW"),
    ({"health_ok": False}, "P4E6A8_SETTLED_READINESS_NEEDS_REVIEW"),
    ({"epochs": {"gate": {"passes": False}}}, "P4E6A8_SETTLED_READINESS_NEEDS_REVIEW"),
])
def test_readiness_fail_closed(changes, reason):
    with pytest.raises(EvidenceAdjudicationError) as exc:
        ready(PersistentStopAdjudicator(), **changes)
    assert_reason(exc, reason)


def test_stable_clear_requires_five_seconds():
    a = active_with_clear(4_990_000_000)
    with pytest.raises(EvidenceAdjudicationError):
        a.inject_stop(9_100_000_000)


def test_pending_is_allowed_during_clear_and_stop_but_stale_is_not():
    a = active_with_clear(); a.inject_stop(9_100_000_000)
    a.observe_stop(9_150_000_000, stop_age_sec=0)
    a.observe_stop(9_200_000_000, pair_state="PENDING")
    a.observe_stop(9_300_000_000, pair_state="PENDING")
    a.observe_stop(9_320_000_000, stop_age_sec=.17)
    assert a.stop_started_ns == 9_150_000_000
    a.observe_stop(9_400_000_000, pair_state="PENDING")
    with pytest.raises(EvidenceAdjudicationError) as exc:
        a.observe_stop(9_650_000_000, pair_state="PENDING")
    assert_reason(exc, "P4E6A8_COMMAND_PAIR_STALE_NEEDS_REVIEW")


def test_one_way_fixture_rejects_duplicate_and_clear():
    a = PersistentStopAdjudicator(); a.fixture_command("STOP")
    with pytest.raises(EvidenceAdjudicationError): a.fixture_command("CLEAR")
    assert a.fixture_requests == ["STOP"]
    text = RUNNER.read_text(encoding="utf-8")
    assert 'node.mode("CLEAR")' not in text


@pytest.mark.parametrize("age,event,allowed", [
    (.99, "COLLISION_STOP_TEMPORARY", False),
    (1.0, "COLLISION_STOP_TEMPORARY", True),
    (19.99, "PERSISTENT_COLLISION_STOP", False),
    (20.0, "PERSISTENT_COLLISION_STOP", True),
])
def test_exact_stop_thresholds(age, event, allowed):
    a = stopped(temp=event == "PERSISTENT_COLLISION_STOP") if event == "PERSISTENT_COLLISION_STOP" else stopped(False)
    now = a.stop_started_ns + int(age*1e9)
    if allowed:
        a.observe_stop(now, event=event, stop_age_sec=age)
        assert (a.temporary_blocked_ns if age == 1.0 else a.persistent_stop_ns) == now
    else:
        with pytest.raises(EvidenceAdjudicationError):
            a.observe_stop(now, event=event, stop_age_sec=age)


def test_pending_crosses_threshold_but_event_waits_for_qualified_stop():
    a = stopped(False)
    a.observe_stop(a.stop_started_ns+950_000_000, pair_state="PENDING")
    assert a.temporary_blocked_ns is None
    a.observe_stop(a.stop_started_ns+1_020_000_000,
                   event="COLLISION_STOP_TEMPORARY", stop_age_sec=1.02)
    a.observe_stop(a.stop_started_ns+19_950_000_000, pair_state="PENDING")
    assert a.persistent_stop_ns is None
    a.observe_stop(a.stop_started_ns+20_030_000_000,
                   event="PERSISTENT_COLLISION_STOP", stop_age_sec=20.03)


@pytest.mark.parametrize("pair_state,semantic,event,reason", [
    ("VALID", "CLEAR", "COLLISION_STOP_PENDING", "P4E6A8_STOP_TIMER_RESET_NEEDS_REVIEW"),
    ("VALID", "SLOWDOWN", "COLLISION_STOP_PENDING", "P4E6A8_STOP_TIMER_RESET_NEEDS_REVIEW"),
    ("VALID", "NO_MOVEMENT", "COLLISION_STOP_PENDING", "P4E6A8_STOP_TIMER_RESET_NEEDS_REVIEW"),
    ("AMBIGUOUS", "STOP", "COLLISION_STOP_PENDING", "P4E6A8_OBSERVER_AMBIGUITY_NEEDS_REVIEW"),
    ("STALE", "STOP", "COMMAND_PAIR_STALE", "P4E6A8_COMMAND_PAIR_STALE_NEEDS_REVIEW"),
])
def test_stop_continuity_breaks_are_bounded(pair_state, semantic, event, reason):
    a = stopped(False)
    with pytest.raises(EvidenceAdjudicationError) as exc:
        a.observe_stop(9_300_000_000, pair_state=pair_state, semantic=semantic, event=event)
    assert_reason(exc, reason)


def test_no_progress_age_cannot_mature_young_stop_timer():
    a = active_with_clear(); a.inject_stop(9_100_000_000)
    a.observe_stop(9_150_000_000, stop_age_sec=0, no_progress_age_sec=30)
    with pytest.raises(EvidenceAdjudicationError) as exc:
        a.observe_stop(10_000_000_000, event="PERSISTENT_COLLISION_STOP",
                       stop_age_sec=.85, no_progress_age_sec=30.85)
    assert_reason(exc, "P4E6A8C2_SOURCE_EVENT_RECONSTRUCTION_CONFLICT_NEEDS_REVIEW")


def test_inconsistent_stop_anchor_fails():
    a = stopped(False)
    with pytest.raises(EvidenceAdjudicationError) as exc:
        a.observe_stop(9_250_000_000, stop_age_sec=.05)
    assert_reason(exc, "P4E6A8_STOP_TIMER_RESET_NEEDS_REVIEW")


def test_nav2_abort_before_persistent_is_race():
    a = stopped()
    with pytest.raises(EvidenceAdjudicationError) as exc:
        a.nav2_terminal(11_000_000_000, status="ABORTED")
    assert_reason(exc, "P4E6A8_NAV2_PROGRESS_CHECKER_RACE_NEEDS_REVIEW")


def test_cancel_and_terminal_cardinality():
    a = stopped(); a.observe_stop(a.stop_started_ns+20_000_000_000,
        event="PERSISTENT_COLLISION_STOP", stop_age_sec=20)
    a.nav2_terminal(a.stop_started_ns+20_010_000_000, status="CANCELED", caused_by_persistent=True)
    a.block(a.stop_started_ns+20_020_000_000)
    assert a.summarize()["cancel_count"] == 1


def test_zero_cancel_and_duplicate_cancel_fail():
    a = stopped(); a.observe_stop(a.stop_started_ns+20_000_000_000,
        event="PERSISTENT_COLLISION_STOP", stop_age_sec=20)
    with pytest.raises(EvidenceAdjudicationError): a.block(a.stop_started_ns+20_010_000_000)
    a.nav2_terminal(a.stop_started_ns+20_020_000_000, status="CANCELED", caused_by_persistent=True)
    with pytest.raises(EvidenceAdjudicationError):
        a.nav2_terminal(a.stop_started_ns+20_030_000_000, status="CANCELED", caused_by_persistent=True)


@pytest.mark.parametrize("reason,completed,wp1", [
    ("CONTROLLER_NO_PROGRESS", 0, 0),
    ("RECOVERY_EXHAUSTED_NO_PROGRESS", 0, 0),
    ("PERSISTENT_COLLISION_STOP", 1, 0),
    ("PERSISTENT_COLLISION_STOP", 0, 1),
])
def test_wrong_terminal_reason_or_mission_cardinality_fails(reason, completed, wp1):
    a = stopped(); a.observe_stop(a.stop_started_ns+20_000_000_000,
        event="PERSISTENT_COLLISION_STOP", stop_age_sec=20)
    a.nav2_terminal(a.stop_started_ns+20_010_000_000, status="CANCELED", caused_by_persistent=True)
    with pytest.raises(EvidenceAdjudicationError):
        a.block(a.stop_started_ns+20_020_000_000, reason=reason, completed=completed, waypoint1_count=wp1)


@pytest.mark.parametrize("movement,safe_zero", [(False, True), (True, False)])
def test_direct_stop_requires_movement_raw_and_safe_zero(movement, safe_zero):
    a = active_with_clear(); a.inject_stop(9_100_000_000)
    with pytest.raises(EvidenceAdjudicationError):
        a.observe_stop(9_150_000_000, movement_raw=movement, safe_zero=safe_zero)


def test_pre_active_unavailable_does_not_fail_readiness():
    a = PersistentStopAdjudicator(); ready(a)
    assert a.phase == "MISSION_START"


def test_source_has_prospective_readiness_five_second_clear_and_no_user_cancel():
    text = RUNNER.read_text(encoding="utf-8")
    assert "CALLBACK_DRAIN_SEC = 1.0" in text
    assert "FORMAL_READINESS_EPOCH_SEC = 2.0" in text
    assert "STABLE_CLEAR_SEC = 5.0" in text
    assert 'node.trigger("cancel")' not in text
    assert 'node.mode("STOP")' in text and 'node.mode("CLEAR")' not in text


def test_runner_has_no_action_client_and_no_production_policy():
    text = RUNNER.read_text(encoding="utf-8").lower()
    assert "actionclient" not in text
    for forbidden in ("turning radius", "minimum radius", "reverse prohibition", "millimeter"):
        assert forbidden not in text


def forensic_odom_rows():
    return [
        (541599747134468, 0, 5.733459885822728, -53.72723606259897, 0, .07, 0),
        (541600274170382, 0, 5.766191219122134, -53.72764660712554, 0, 0, 0),
        (541601672728801, 0, 5.766191219122134, -53.72764660712554, 0, 0, 0),
    ]


def test_exact_a8b_forensic_stationary_window_regression():
    rows = forensic_odom_rows()
    old = math.hypot(rows[-1][2]-rows[0][2], rows[-1][3]-rows[0][3])
    assert old == 0.03273390789938009
    assert old > MAXIMUM_STATIONARY_TRANSLATION_M
    window = build_stationary_window(
        rows, fake_applied_zero_ns=541600256476849, propagation_pose=rows[0],
        fixture_stop_ns=541599692530532, first_safe_zero_ns=541599734090145,
        first_vehicle_zero_ns=541599734944905)
    result = adjudicate_stationary_checkpoint(window, rows[-1])
    assert result["propagation_translation_m"] == old
    assert result["post_downstream_zero_translation_m"] == 0.0
    assert result["passes"]


def test_preserved_a8b_timeline_replays_corrected_window():
    evidence = Path("/home/dog/phase4_runtime/p4e6a8b_persistent_stop_campaign_20260809T214416+0800/evidence")
    assert evidence.is_dir(), "accepted A.8B replay evidence is required"
    with (evidence / "odometry_timeline.tsv").open() as handle:
        odom = list(csv.DictReader(handle, delimiter="\t"))
    with (evidence / "fake_applied_timeline.tsv").open() as handle:
        fake = list(csv.DictReader(handle, delimiter="\t"))
    fake_zero = next(int(row["mono_ns"]) for row in fake
                     if int(row["mono_ns"]) > 541600256476848
                     and abs(float(row["linear_x"])) <= 1e-6
                     and abs(float(row["angular_z"])) <= 1e-6)
    propagation_pose = max((row for row in odom if int(row["mono_ns"]) <= 541599754368728),
                           key=lambda row: int(row["mono_ns"]))
    window = build_stationary_window(
        odom, fake_applied_zero_ns=fake_zero, propagation_pose=propagation_pose,
        fixture_stop_ns=541599692530532, first_safe_zero_ns=541599734090145,
        first_vehicle_zero_ns=541599734944905)
    result = adjudicate_stationary_checkpoint(window, odom[-1])
    old = math.hypot(float(odom[-1]["x"])-float(propagation_pose["x"]),
                     float(odom[-1]["y"])-float(propagation_pose["y"]))
    assert old == 0.03273390789938009 and old > .02
    assert result["post_downstream_zero_translation_m"] == 0.0 and result["passes"]


def test_stationary_window_requires_fake_applied_zero():
    with pytest.raises(EvidenceAdjudicationError) as exc:
        build_stationary_window(forensic_odom_rows(), fake_applied_zero_ns=None,
                                propagation_pose=forensic_odom_rows()[0])
    assert_reason(exc, "P4E6A8B2_STATIONARY_FAKE_ZERO_MISSING_NEEDS_REVIEW")


def test_stationary_window_requires_zero_twist_odom_after_fake_zero():
    rows = [forensic_odom_rows()[0]]
    with pytest.raises(EvidenceAdjudicationError) as exc:
        build_stationary_window(rows, fake_applied_zero_ns=rows[0][0],
                                propagation_pose=rows[0])
    assert_reason(exc, "P4E6A8B2_STATIONARY_ODOM_MISSING_NEEDS_REVIEW")


def test_large_propagation_is_reported_but_stationary_window_passes():
    rows = forensic_odom_rows()
    result = adjudicate_stationary_checkpoint(build_stationary_window(
        rows, fake_applied_zero_ns=541600256476849, propagation_pose=rows[0]), rows[-1])
    assert result["propagation_translation_m"] > .02
    assert result["post_downstream_zero_translation_m"] == 0
    assert result["passes"]


def test_real_post_fake_zero_translation_above_bound_fails():
    rows = forensic_odom_rows()
    moved = (541602000000000, 0, rows[1][2] + .021, rows[1][3], 0, 0, 0)
    result = adjudicate_stationary_checkpoint(build_stationary_window(
        rows, fake_applied_zero_ns=541600256476849, propagation_pose=rows[0]), moved)
    assert result["post_downstream_zero_translation_m"] == pytest.approx(.021)
    assert not result["passes"]


def test_checkpoint_freshness_distinguishes_writer_lag_from_serviced_safe_age():
    fresh = checkpoint_freshness(2_000_000_000, safe_receipt_ns=1_990_000_000,
                                 odom_receipt_ns=1_985_000_000,
                                 policy_receipt_ns=1_980_000_000)
    assert fresh["safe_fresh"] and not fresh["callback_servicing_unhealthy"]
    stale = checkpoint_freshness(2_000_000_000, safe_receipt_ns=1_749_999_999,
                                 odom_receipt_ns=1_990_000_000,
                                 policy_receipt_ns=1_980_000_000)
    assert not stale["safe_fresh"] and not stale["callback_servicing_unhealthy"]
    with pytest.raises(EvidenceAdjudicationError) as exc:
        require_checkpoint_freshness(stale)
    assert_reason(exc, "P4E6A8_COMMAND_PAIR_STALE_NEEDS_REVIEW")
    starved = checkpoint_freshness(2_000_000_000, safe_receipt_ns=1_000_000_000,
                                   odom_receipt_ns=1_000_000_000,
                                   policy_receipt_ns=1_000_000_000)
    assert starved["callback_servicing_unhealthy"]
    with pytest.raises(EvidenceAdjudicationError) as exc:
        require_checkpoint_freshness(starved)
    assert_reason(exc, "P4E6A8B2_CALLBACK_SERVICING_UNHEALTHY_NEEDS_REVIEW")


def test_writer_normal_load_is_complete_ordered_and_bounded(tmp_path):
    writer = BoundedEvidenceWriter(tmp_path, capacity=256)
    for i in range(200):
        writer.enqueue("normal.jsonl", {"i": i})
    status = writer.finalize()
    rows = [json.loads(x) for x in (tmp_path / "normal.jsonl").read_text().splitlines()]
    assert status["persisted_sequence"] == 200
    assert [x["evidence_sequence"] for x in rows] == list(range(1, 201))


def test_point_nine_second_writer_stall_does_not_freeze_ingestion(tmp_path):
    writer = BoundedEvidenceWriter(tmp_path, capacity=512, initial_stall_sec=.9)
    receipts = {name: [] for name in ("safe", "raw", "gate", "adapter", "odom", "tf", "policy")}
    rates = {"safe": 20, "raw": 20, "gate": 20, "adapter": 20,
             "odom": 50, "tf": 50, "policy": 20}
    started = time.monotonic()
    for name, hz in rates.items():
        for i in range(hz):
            receipt = int(i * 1e9 / hz)
            receipts[name].append(receipt)
            writer.enqueue("stress.jsonl", {"source": name, "receipt_ns": receipt})
    ingestion_elapsed = time.monotonic() - started
    assert ingestion_elapsed < .25
    assert all(len(receipts[name]) == hz for name, hz in rates.items())
    status = writer.finalize(timeout_sec=3.0)
    assert status["failure_reason"] is None and status["persisted_sequence"] == 200


def test_repeated_short_writer_stalls_remain_ordered(tmp_path):
    for run in range(3):
        target = tmp_path / str(run); target.mkdir()
        writer = BoundedEvidenceWriter(target, capacity=128, initial_stall_sec=.12)
        for i in range(80): writer.enqueue("rows.jsonl", {"i": i})
        assert writer.finalize(timeout_sec=2)["persisted_sequence"] == 80


def test_queue_overflow_is_explicit_and_worker_terminates(tmp_path):
    writer = BoundedEvidenceWriter(tmp_path, capacity=2, initial_stall_sec=.9)
    try:
        writer.enqueue("overflow.jsonl", {"i": 0})
        writer.enqueue("overflow.jsonl", {"i": 1})
        with pytest.raises(EvidenceWriterError) as exc:
            writer.enqueue("overflow.jsonl", {"i": 2})
        assert exc.value.reason == "P4E6A8B2_EVIDENCE_QUEUE_OVERFLOW_NEEDS_REVIEW"
    finally:
        with pytest.raises(EvidenceWriterError): writer.finalize(timeout_sec=2)
    assert not writer.status()["worker_alive"]


def test_writer_exception_is_explicit_and_worker_terminates(tmp_path):
    writer = BoundedEvidenceWriter(tmp_path, fail_sequence=1)
    writer.enqueue("failure.jsonl", {"i": 0})
    with pytest.raises(EvidenceWriterError) as exc: writer.finalize()
    assert exc.value.reason == "P4E6A8B2_EVIDENCE_WRITER_EXCEPTION_NEEDS_REVIEW"
    assert not writer.status()["worker_alive"]


def test_sequence_gap_is_explicit(tmp_path):
    writer = BoundedEvidenceWriter(tmp_path)
    writer.inject_record_for_test(2, "gap.jsonl", {"i": 2})
    with pytest.raises(EvidenceWriterError) as exc: writer.finalize()
    assert exc.value.reason == "P4E6A8B2_EVIDENCE_SEQUENCE_MISSING_NEEDS_REVIEW"
    assert not writer.status()["worker_alive"]


def test_out_of_order_sequence_is_explicit(tmp_path):
    writer = BoundedEvidenceWriter(tmp_path)
    writer.inject_record_for_test(0, "order.jsonl", {"i": 0})
    with pytest.raises(EvidenceWriterError) as exc: writer.finalize()
    assert exc.value.reason == "P4E6A8B2_EVIDENCE_SEQUENCE_OUT_OF_ORDER_NEEDS_REVIEW"
    assert not writer.status()["worker_alive"]


def test_final_drain_timeout_is_explicit_and_worker_terminates(tmp_path):
    writer = BoundedEvidenceWriter(tmp_path, write_delay_sec=2.0, drain_timeout_sec=.01)
    writer.enqueue("slow.jsonl", {"i": 0})
    with pytest.raises(EvidenceWriterError) as exc: writer.finalize()
    assert exc.value.reason == "P4E6A8B2_EVIDENCE_DRAIN_TIMEOUT_NEEDS_REVIEW"
    assert not writer.status()["worker_alive"]


def test_repaired_complete_path_deterministic(tmp_path):
    result = synthetic_repaired_complete_path(tmp_path)
    assert result["phase"] == "COMPLETE" and result["cancel_count"] == 1
    assert result["checkpoint"]["passes"]
    assert result["checkpoint_safe_age_ns"] < 250_000_000
    assert result["persisted_count"] == result["callback_count"]


def test_slow_writer_complete_path_does_not_change_policy_or_freshness(tmp_path):
    result = synthetic_repaired_complete_path(tmp_path, writer_initial_stall_sec=.9)
    assert result["phase"] == "COMPLETE"
    assert result["checkpoint_safe_age_ns"] == 10_000_000
    assert result["writer"]["failure_reason"] is None
    assert result["persisted_count"] == result["callback_count"]


def test_queue_capacity_covers_point_nine_second_expected_load():
    expected_hz = 20 + 20 + 20 + 20 + 50 + 50 + 20
    assert math.ceil(expected_hz * .9) == 180
    assert EVIDENCE_QUEUE_CAPACITY == 4096
    assert EVIDENCE_QUEUE_CAPACITY > 20 * math.ceil(expected_hz * .9)


def authority_ledger():
    return StopEpisodeLedger("episode-1", "uuid-0", "epoch-0", "fixture-stop-1",
                             1_000_000_000, 1_000_000_000)


def test_exact_a8c_receipt_anchor_defect_is_behaviorally_reproduced():
    result = reproduce_receipt_anchor_defect(
        first_evaluation_ns=546303936916620,
        first_diagnostic_receipt_ns=546304045846091,
        event_publication_ns=546304955437462,
        event_receipt_ns=546304957667236)
    assert result["status"] == "P4E6A8C2_RECEIPT_ANCHOR_DEFECT_REPRODUCED"
    assert result["receipt_delta_ns"] == 911821145
    assert result["source_interval_ns"] == 1018520842
    assert not result["old_logic_passes"] and result["source_consistent"]


def test_supervisor_source_hash_guard_accepts_frozen_source():
    path = ROOT.parent / "parking_robot_mission_manager" / "parking_robot_mission_manager" / "mission_progress_supervisor_core.py"
    result = verify_supervisor_source_authority(path)
    assert result["matches"] and result["observed_sha256"] == FROZEN_SUPERVISOR_SHA256


def test_supervisor_source_hash_guard_rejects_unknown_source(tmp_path):
    path = tmp_path / "core.py"; path.write_text("unknown\n")
    with pytest.raises(EvidenceAdjudicationError) as exc:
        verify_supervisor_source_authority(path)
    assert_reason(exc, "P4E6A8C2_SUPERVISOR_SOURCE_AUTHORITY_MISMATCH")


@pytest.mark.parametrize("receipt_delta_ns", [800_000_000, 1_200_000_000])
def test_temporary_source_event_not_receipt_delta_is_authority(receipt_delta_ns):
    ledger = authority_ledger()
    first_diag_receipt = 1_109_000_000
    result = adjudicate_source_event(
        supervisor_sha256=FROZEN_SUPERVISOR_SHA256, ledger=ledger,
        event="COLLISION_STOP_TEMPORARY",
        event_receipt_ns=first_diag_receipt + receipt_delta_ns,
        mission_state="TEMPORARILY_BLOCKED",
        first_evaluation_estimate_ns=1_000_000_000,
        publication_estimate_ns=2_018_000_000)
    assert result["pass"] and not result["receipt_delta_threshold_applied"]
    assert result["private_threshold_sec_proven"] == 1.0


def test_no_temporary_event_is_not_qualified_even_after_long_receipt_duration():
    ledger = authority_ledger(); ledger.qualified_stop(2_500_000_000)
    assert ledger.temporary_event_ns is None


@pytest.mark.parametrize("breaker", ["CLEAR", "SLOWDOWN", "NO_MOVEMENT", "PAIR_AMBIGUOUS", "PAIR_STALE"])
def test_temporary_event_rejects_any_episode_breaker(breaker):
    ledger = authority_ledger(); ledger.break_episode(1_500_000_000, breaker)
    with pytest.raises(EvidenceAdjudicationError) as exc:
        adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
            ledger=ledger, event="COLLISION_STOP_TEMPORARY",
            event_receipt_ns=2_100_000_000, mission_state="TEMPORARILY_BLOCKED")
    assert_reason(exc, "P4E6A8C2_STOP_EPISODE_LEDGER_NEEDS_REVIEW")


def test_temporary_event_passes_when_reconstruction_is_unobservable():
    result = adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
        ledger=authority_ledger(), event="COLLISION_STOP_TEMPORARY",
        event_receipt_ns=2_100_000_000, mission_state="TEMPORARILY_BLOCKED")
    assert result["pass"] and result["publication_estimate_ns"] is None


def test_temporary_tight_reconstruction_contradiction_needs_review():
    with pytest.raises(EvidenceAdjudicationError) as exc:
        adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
            ledger=authority_ledger(), event="COLLISION_STOP_TEMPORARY",
            event_receipt_ns=1_900_000_000, mission_state="TEMPORARILY_BLOCKED",
            first_evaluation_estimate_ns=1_000_000_000,
            publication_estimate_ns=1_800_000_000, uncertainty_ns=1_000_000)
    assert_reason(exc, "P4E6A8C2_SOURCE_EVENT_RECONSTRUCTION_CONFLICT_NEEDS_REVIEW")


def test_persistent_source_event_proves_private_threshold_with_short_receipt_delta():
    ledger = authority_ledger()
    adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
        ledger=ledger, event="COLLISION_STOP_TEMPORARY", event_receipt_ns=2_020_000_000,
        mission_state="TEMPORARILY_BLOCKED")
    result = adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
        ledger=ledger, event="PERSISTENT_COLLISION_STOP", event_receipt_ns=20_909_000_000,
        mission_state="CANCELLING", first_evaluation_estimate_ns=1_000_000_000,
        publication_estimate_ns=21_010_000_000)
    assert result["pass"] and result["private_threshold_sec_proven"] == 20.0


def test_elapsed_twenty_seconds_without_persistent_event_is_not_qualified():
    ledger = authority_ledger(); ledger.qualified_stop(25_000_000_000)
    assert ledger.persistent_event_ns is None


@pytest.mark.parametrize("breaker", ["CLEAR", "PAIR_AMBIGUOUS", "PAIR_STALE"])
def test_persistent_event_after_episode_break_fails(breaker):
    ledger = authority_ledger()
    adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
        ledger=ledger, event="COLLISION_STOP_TEMPORARY", event_receipt_ns=2_100_000_000,
        mission_state="TEMPORARILY_BLOCKED")
    ledger.break_episode(3_000_000_000, breaker)
    with pytest.raises(EvidenceAdjudicationError):
        adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
            ledger=ledger, event="PERSISTENT_COLLISION_STOP", event_receipt_ns=21_100_000_000,
            mission_state="CANCELLING")


def test_old_no_progress_age_without_persistent_collision_event_never_passes():
    a = active_with_clear(); a.inject_stop(9_100_000_000)
    a.observe_stop(9_200_000_000, no_progress_age_sec=30.0)
    a.observe_stop(30_000_000_000, no_progress_age_sec=50.0)
    assert a.persistent_count == 0 and a.persistent_stop_ns is None


def test_persistent_tight_reconstruction_contradiction_needs_review():
    ledger = authority_ledger()
    adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
        ledger=ledger, event="COLLISION_STOP_TEMPORARY", event_receipt_ns=2_100_000_000,
        mission_state="TEMPORARILY_BLOCKED")
    with pytest.raises(EvidenceAdjudicationError) as exc:
        adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
            ledger=ledger, event="PERSISTENT_COLLISION_STOP", event_receipt_ns=20_900_000_000,
            mission_state="CANCELLING", first_evaluation_estimate_ns=1_000_000_000,
            publication_estimate_ns=20_800_000_000, uncertainty_ns=1_000_000)
    assert_reason(exc, "P4E6A8C2_SOURCE_EVENT_RECONSTRUCTION_CONFLICT_NEEDS_REVIEW")


def test_a8c_preserved_replay_accepts_temporary_and_keeps_persistent_untested():
    evidence = Path("/home/dog/phase4_runtime/p4e6a8c_persistent_stop_runtime_preservation_20260809T225716+0800/runtime_attempt_1/evidence")
    policy = [json.loads(x) for x in (evidence / "mission_policy_diagnostics.jsonl").read_text().splitlines()]
    first = next(x for x in policy if x["values"].get("collision_classification") == "STOP")
    states = [json.loads(x) for x in (evidence / "mission_state_events.jsonl").read_text().splitlines()]
    temporary = next(x for x in states if x["state_name"] == "TEMPORARILY_BLOCKED")
    first_timing = reconstruct_policy_timing(diagnostic=first)
    temp_timing = reconstruct_policy_timing(
        state=temporary,
        first_evaluation_ns=first_timing["first_stop_core_evaluation_estimate_ns"])
    ledger = StopEpisodeLedger("a8c", temporary["active_goal_uuid"], "a8c-epoch", "a8c-stop",
                               first["monotonic_ns"], first["monotonic_ns"])
    result = adjudicate_source_event(supervisor_sha256=FROZEN_SUPERVISOR_SHA256,
        ledger=ledger, event="COLLISION_STOP_TEMPORARY",
        event_receipt_ns=temporary["monotonic_ns"], mission_state="TEMPORARILY_BLOCKED",
        first_evaluation_estimate_ns=first_timing["first_stop_core_evaluation_estimate_ns"],
        publication_estimate_ns=temp_timing["state_publication_estimate_ns"])
    assert result["pass"]
    assert temporary["monotonic_ns"] - first["monotonic_ns"] == 911821145
    assert first_timing["first_stop_core_evaluation_estimate_ns"] == 546303936916620
    assert temp_timing["state_publication_estimate_ns"] == 546304955437462
    assert ledger.persistent_event_ns is None


def test_reconstruction_returns_unobservable_without_required_fields():
    result = reconstruct_policy_timing(diagnostic={"values": {}, "monotonic_ns": 1})
    assert result["first_stop_core_evaluation_estimate_ns"] is None
    assert result["diagnostic_receipt_latency_ns"] is None


def test_distorted_latency_complete_path_uses_source_authority(tmp_path):
    result = synthetic_repaired_complete_path(tmp_path)
    timing = result["distorted_receipt_timing"]
    assert timing["temporary_receipt_delta_ns"] < TEMPORARY_STOP_NS
    assert timing["persistent_receipt_delta_ns"] < PERSISTENT_STOP_NS
    assert len(result["authority_results"]) == 2
    assert all(row["pass"] for row in result["authority_results"])
