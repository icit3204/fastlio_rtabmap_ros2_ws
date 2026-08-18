import pytest

from parking_robot_bringup.phase4_p4e6b_terminal_closure import (
    POST_FAILED_DRAIN_SEC, WitnessContract, adjudicate_terminal_v2,
    controller_lifetime_decision, physical_closure, physical_closure_from_rows,
)
from phase4_p4e6b_terminal_closure_witness import contract


def state(ns, name, reason, uuid="u", mission="m"):
    return {"monotonic_ns": ns, "state_name": name, "reason_code": reason,
            "active_goal_uuid": uuid, "mission_id": mission, "route_id": "r",
            "waypoint_index": 0}


def status(ns, name, uuid="u"):
    return {"monotonic_ns": ns, "status_name": name, "goal_uuid": uuid}


def history(canceled_ns=15, uuid="u"):
    return ([state(10, "CANCELLING", "FEEDBACK_STALE", uuid),
             state(12, "CANCELLING", "HEALTH_CANCEL_ACK_ACCEPTED", uuid),
             state(20, "FAILED", "FEEDBACK_STALE", "", "m")],
            [status(11, "CANCELING", uuid), status(canceled_ns, "CANCELED", uuid)])


def adjudicate(statuses, source=True):
    states, _ = history()
    return adjudicate_terminal_v2(origin_ns=9, reason="FEEDBACK_STALE", mission_id="m",
                                  route_id="r", waypoint=0, uuid="u", states=states,
                                  statuses=statuses, drain_start_ns=20, drain_end_ns=270_000_020,
                                  source_result_canceled=source)


def test_receipt_order_normal_passes():
    result = adjudicate([status(11, "CANCELING"), status(15, "CANCELED")])
    assert result["pass"]


def test_failed_receipt_before_canceled_receipt_is_accepted():
    result = adjudicate([status(11, "CANCELING"), status(25, "CANCELED")])
    assert result["pass"]
    assert result["receipt_order_observed"]


def test_missing_canceled_fails():
    result = adjudicate([status(11, "CANCELING")])
    assert not result["pass"]


def test_wrong_uuid_fails():
    result = adjudicate([status(11, "CANCELING"), status(15, "CANCELED", "old")])
    assert not result["pass"]


def test_duplicate_canceled_fails():
    result = adjudicate([status(11, "CANCELING"), status(15, "CANCELED"), status(16, "CANCELED")])
    assert not result["pass"]


def test_missing_ack_fails():
    states, _ = history()
    states = [states[0], states[2]]
    result = adjudicate_terminal_v2(origin_ns=9, reason="FEEDBACK_STALE", mission_id="m",
                                    route_id="r", waypoint=0, uuid="u", states=states,
                                    statuses=[status(11, "CANCELING"), status(15, "CANCELED")])
    assert not result["pass"]


def test_aborted_fails():
    result = adjudicate([status(11, "CANCELING"), status(15, "ABORTED")])
    assert not result["pass"]


def test_physical_missing_is_not_pass():
    assert physical_closure(translation_m=None, rotation_rad=None)["classification"] == "NOT_ADJUDICATED"


def test_physical_boundaries():
    assert physical_closure(translation_m=.02, rotation_rad=.2)["pass"]
    assert physical_closure(translation_m=.01, rotation_rad=.5)["pass"]
    assert not physical_closure(translation_m=.020001, rotation_rad=0)["pass"]


def test_rotation_is_finite_report_only():
    assert physical_closure(translation_m=.01, rotation_rad=float("nan"))["pass"] is False
    assert physical_closure(translation_m=.01, rotation_rad=None)["pass"] is False


def test_controller_waits_for_witness_after_runner_failure():
    assert controller_lifetime_decision(runner_exit_code=1, failed_seen=True,
                                        drain_complete=False) == "WITNESS_DRAIN_INCOMPLETE"
    assert controller_lifetime_decision(runner_exit_code=1, failed_seen=True,
                                        drain_complete=True) == "CONTINUE_WITNESS_THEN_TEARDOWN"


def test_witness_has_zero_application_authority():
    c = contract()
    assert c["passive"]
    assert c["publishers"] == [] and c["services"] == [] and c["action_clients"] == []
    assert c["post_failed_drain_sec"] == POST_FAILED_DRAIN_SEC


@pytest.mark.parametrize("mutation", ["no_canceled", "wrong_uuid", "duplicate_canceled",
                                       "aborted", "no_ack", "competing_reason"])
def test_runtime_terminal_negative_matrix(mutation):
    states, statuses = history()
    if mutation == "no_canceled": statuses = [statuses[0]]
    elif mutation == "wrong_uuid": statuses[1]["goal_uuid"] = "old"
    elif mutation == "duplicate_canceled": statuses.append(dict(statuses[1]))
    elif mutation == "aborted": statuses[1]["status_name"] = "ABORTED"
    elif mutation == "no_ack": states = [states[0], states[2]]
    elif mutation == "competing_reason": states[0]["reason_code"] = "GATE_FAULT"
    result = adjudicate_terminal_v2(origin_ns=9, reason="FEEDBACK_STALE", mission_id="m",
                                    route_id="r", waypoint=0, uuid="u", states=states,
                                    statuses=statuses)
    assert not result["pass"]


def test_physical_rows_require_all_streams_and_accept_stationary_window():
    vehicle = [{"monotonic_ns": 10, "linear_x": .1, "linear_y": 0, "linear_z": 0,
                "angular_x": 0, "angular_y": 0, "angular_z": 0},
               {"monotonic_ns": 20, "linear_x": 0, "linear_y": 0, "linear_z": 0,
                "angular_x": 0, "angular_y": 0, "angular_z": 0}]
    applied = [dict(vehicle[1])]
    odom = [{"monotonic_ns": 20, "x": 1., "y": 2., "yaw": 0.},
            {"monotonic_ns": 30, "x": 1., "y": 2., "yaw": 0.}]
    result = physical_closure_from_rows(vehicle_rows=vehicle, applied_rows=applied,
                                        odometry_rows=odom, terminal_ns=10, drain_end_ns=40)
    assert result["pass"] and result["translation_m"] == 0.
    assert physical_closure_from_rows(vehicle_rows=vehicle, applied_rows=[], odometry_rows=odom,
                                      terminal_ns=10, drain_end_ns=40)["classification"] != "PASS"


def test_physical_rows_motion_and_missing_pose_fail_closed():
    vehicle = [{"monotonic_ns": 10, "linear_x": 0, "linear_y": 0, "linear_z": 0,
                "angular_x": 0, "angular_y": 0, "angular_z": 0}]
    moved = [{"monotonic_ns": 10, "x": 0., "y": 0., "yaw": 0.},
             {"monotonic_ns": 20, "x": .021, "y": 0., "yaw": 0.}]
    assert not physical_closure_from_rows(vehicle_rows=vehicle, applied_rows=vehicle,
                                          odometry_rows=moved, terminal_ns=10,
                                          drain_end_ns=30)["pass"]
    assert physical_closure_from_rows(vehicle_rows=vehicle, applied_rows=vehicle,
                                      odometry_rows=[], terminal_ns=10,
                                      drain_end_ns=30)["classification"] == "NOT_ADJUDICATED"
